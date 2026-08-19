#include <Arduino.h>
#include <BootMotion.h>

#if BOOT_MOTION_SELF_TEST

#include <math.h>

namespace {

volatile uint8_t testSink = 0;
volatile uint8_t testsComplete = 0;

void require(bool condition) {
    if (!condition && testSink != 0xFFU) {
        ++testSink;
    }
}

bool nearValue(float actual, float expected, float tolerance) {
    return fabs(actual - expected) <= tolerance;
}

void testNaturalTimeline() {
    BootMotion motion;
    motion.reset(0.0f, 0.0f);
    require(motion.state() == BOOT_IDLE);
    motion.start(1000UL, 0.0f, 0.0f, 0.0f, 0.0f);
    require(motion.state() == BOOT_OPENING);

    for (uint32_t elapsed = 0; elapsed <= 25000UL; elapsed += 100UL) {
        motion.update(1000UL + elapsed);
        const BootMotionSample& sample = motion.sample();
        require(isfinite(sample.panDegrees));
        require(isfinite(sample.tiltDegrees));
        require(sample.panDegrees >= -30.0f && sample.panDegrees <= 30.0f);
        require(sample.tiltDegrees >= BootMotion::TILT_MIN_DEGREES && sample.tiltDegrees <= BootMotion::TILT_MAX_DEGREES);
        if (elapsed < 24000UL) {
            require(sample.panDegrees >= -18.001f && sample.panDegrees <= 18.001f);
            require(sample.tiltDegrees >= -0.001f && sample.tiltDegrees <= 40.001f);
        }
        if (elapsed < 14700UL) {
            require(motion.state() == BOOT_OPENING);
        } else if (elapsed < 24000UL) {
            require(motion.state() == BOOT_WAIT_LOOP);
        } else if (elapsed < 25000UL) {
            require(motion.state() == BOOT_FINISHING);
            require(motion.finishReason() == BOOT_REASON_TIMEOUT);
            require(motion.timedOut());
        } else {
            require(motion.state() == BOOT_DONE);
        }
    }
    require(motion.state() == BOOT_DONE);
    require(nearValue(motion.sample().panVelocity, 0.0f, 0.001f));
    require(nearValue(motion.sample().tiltVelocity, 0.0f, 0.001f));
    const BootMotionSample stopped = motion.sample();
    motion.update(61000UL);
    require(nearValue(motion.sample().panDegrees, stopped.panDegrees, 0.001f));
    require(nearValue(motion.sample().tiltDegrees, stopped.tiltDegrees, 0.001f));
}

void testFinishAndCancel() {
    BootMotion motion;
    motion.start(0UL, 0.0f, 0.0f, 0.0f, 0.0f);
    motion.update(4321UL);
    const BootMotionSample before = motion.sample();
    const uint16_t duration = motion.finishTo(
        4321UL,
        before.panDegrees,
        before.tiltDegrees,
        before.panVelocity,
        before.tiltVelocity,
        1000.0f,
        -1000.0f
    );
    require(duration >= 800U && duration <= 3000U);
    require(motion.state() == BOOT_FINISHING);
    require(nearValue(motion.sample().panDegrees, before.panDegrees, 0.001f));
    require(nearValue(motion.sample().tiltDegrees, before.tiltDegrees, 0.001f));
    require(nearValue(motion.sample().panVelocity, before.panVelocity, 0.001f));
    require(nearValue(motion.sample().tiltVelocity, before.tiltVelocity, 0.001f));
    motion.update(4321UL);
    require(nearValue(motion.sample().panDegrees, before.panDegrees, 0.001f));
    require(nearValue(motion.sample().panVelocity, before.panVelocity, 0.001f));
    motion.update(4321UL + duration - 1U);
    require(motion.state() == BOOT_FINISHING);
    motion.update(4321UL + duration);
    require(motion.state() == BOOT_DONE);
    require(nearValue(motion.sample().panDegrees, 30.0f, 0.001f));
    require(nearValue(motion.sample().tiltDegrees, -30.0f, 0.001f));
    require(nearValue(motion.sample().panVelocity, 0.0f, 0.001f));
    require(nearValue(motion.sample().tiltVelocity, 0.0f, 0.001f));

    motion.start(9000UL, 0.0f, 0.0f, 0.0f, 0.0f);
    motion.update(10234UL);
    const BootMotionSample moving = motion.sample();
    const uint16_t cancelDuration = motion.cancel(
        10234UL,
        moving.panDegrees,
        moving.tiltDegrees,
        moving.panVelocity,
        moving.tiltVelocity
    );
    const float expectedPan = BootMotion::clampPan(
        moving.panDegrees + 0.5f * moving.panVelocity * 0.8f
    );
    const float expectedTilt = BootMotion::clampTilt(
        moving.tiltDegrees + 0.5f * moving.tiltVelocity * 0.8f
    );
    require(cancelDuration == 800U);
    motion.update(10234UL + 799UL);
    require(motion.state() == BOOT_FINISHING);
    motion.update(10234UL + cancelDuration);
    require(motion.state() == BOOT_DONE);
    require(motion.finishReason() == BOOT_REASON_CANCELLED);
    require(nearValue(motion.sample().panDegrees, expectedPan, 0.001f));
    require(nearValue(motion.sample().tiltDegrees, expectedTilt, 0.001f));
}

void testMillisRollover() {
    BootMotion motion;
    const uint32_t start = 0xFFFFF000UL;
    motion.start(start, 0.0f, 0.0f, 0.0f, 0.0f);
    motion.update(start + 14700UL);
    require(motion.state() == BOOT_WAIT_LOOP);
    motion.update(start + 24000UL);
    require(motion.state() == BOOT_FINISHING);
    motion.update(start + 25000UL);
    require(motion.state() == BOOT_DONE);
}

void testBoundaryVelocityFallback() {
    BootMotion motion;
    motion.reset(30.0f, 30.0f);
    motion.finishTo(0UL, 30.0f, 30.0f, 10.0f, 10.0f, 30.0f, 30.0f);
    const BootMotionSample before = motion.sample();
    motion.update(0UL);
    require(nearValue(motion.sample().panDegrees, before.panDegrees, 0.001f));
    require(nearValue(motion.sample().panVelocity, before.panVelocity, 0.001f));
    for (uint16_t elapsed = 0; elapsed <= motion.finishingDurationMs(); elapsed += 10U) {
        motion.update(elapsed);
        require(motion.sample().panDegrees <= 30.0f);
        require(motion.sample().tiltDegrees <= BootMotion::TILT_MAX_DEGREES);
    }
    motion.update(motion.finishingDurationMs());
    require(motion.state() == BOOT_DONE);
}

void runTests() {
    testNaturalTimeline();
    testFinishAndCancel();
    testMillisRollover();
    testBoundaryVelocityFallback();
}

}  // namespace

extern "C" void bootMotionTestsFinished() __attribute__((noinline, used));

extern "C" void bootMotionTestsFinished() {
    __asm__ __volatile__("nop");
}

void setup() {
    runTests();
    testsComplete = 1;
    bootMotionTestsFinished();
}

void loop() {
    testSink = testSink;
    testsComplete = testsComplete;
}

#endif  // BOOT_MOTION_SELF_TEST
