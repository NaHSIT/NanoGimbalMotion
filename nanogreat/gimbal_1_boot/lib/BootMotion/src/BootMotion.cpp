#include "BootMotion.h"

#include <math.h>

#ifndef BOOT_TILT_MIN_DEGREES
#define BOOT_TILT_MIN_DEGREES -30.0f
#endif
#ifndef BOOT_TILT_MAX_DEGREES
#define BOOT_TILT_MAX_DEGREES 45.0f
#endif

namespace {

const uint8_t OPENING_FRAME_COUNT = 7;
const uint32_t OPENING_TOTAL_MS = 14700UL;
const uint32_t WAIT_BLEND_MS = 5000UL;
const uint32_t WAIT_PERIOD_MS = 5000UL;
const uint32_t BOOT_BRAKE_START_MS = 24000UL;
const uint16_t BOOT_BRAKE_DURATION_MS = 1000U;
const float WAIT_CENTER_PAN = 0.0f;
const float WAIT_CENTER_TILT = 12.0f;
const float WAIT_RADIUS_PAN = 4.0f;
const float WAIT_RADIUS_TILT = 2.0f;
const float TWO_PI = 6.28318530718f;

float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

float larger(float first, float second) {
    return first > second ? first : second;
}

float minimumJerk(float u) {
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

float minimumJerkDerivative(float u) {
    const float oneMinusU = 1.0f - u;
    return 30.0f * u * u * oneMinusU * oneMinusU;
}

}  // namespace

const float BootMotion::PAN_MIN_DEGREES = -30.0f;
const float BootMotion::PAN_MAX_DEGREES = 30.0f;
const float BootMotion::TILT_MIN_DEGREES = BOOT_TILT_MIN_DEGREES;
const float BootMotion::TILT_MAX_DEGREES = BOOT_TILT_MAX_DEGREES;

BootMotion::BootMotion()
    : state_(BOOT_IDLE),
      finishReason_(BOOT_REASON_NONE),
      bootStartMs_(0),
      segmentStartMs_(0),
      segmentDurationMs_(0),
      finishingDurationMs_(0),
      openingIndex_(0),
      timedOut_(false) {
    sample_.panDegrees = 0.0f;
    sample_.tiltDegrees = 0.0f;
    sample_.panVelocity = 0.0f;
    sample_.tiltVelocity = 0.0f;
}

void BootMotion::reset(float panDegrees, float tiltDegrees) {
    state_ = BOOT_IDLE;
    finishReason_ = BOOT_REASON_NONE;
    timedOut_ = false;
    finishingDurationMs_ = 0;
    sample_.panDegrees = clampPan(panDegrees);
    sample_.tiltDegrees = clampTilt(tiltDegrees);
    sample_.panVelocity = 0.0f;
    sample_.tiltVelocity = 0.0f;
}

void BootMotion::start(
    uint32_t now,
    float panDegrees,
    float tiltDegrees,
    float panVelocity,
    float tiltVelocity
) {
    bootStartMs_ = now;
    finishReason_ = BOOT_REASON_NONE;
    timedOut_ = false;
    finishingDurationMs_ = 0;
    beginOpeningSegment(
        0,
        now,
        clampPan(panDegrees),
        clampTilt(tiltDegrees),
        panVelocity,
        tiltVelocity
    );
}

uint16_t BootMotion::finishTo(
    uint32_t now,
    float panDegrees,
    float tiltDegrees,
    float panVelocity,
    float tiltVelocity,
    float targetPanDegrees,
    float targetTiltDegrees
) {
    const float startPan = clampPan(panDegrees);
    const float startTilt = clampTilt(tiltDegrees);
    const float targetPan = clampPan(targetPanDegrees);
    const float targetTilt = clampTilt(targetTiltDegrees);
    const uint16_t durationMs = chooseFinishDuration(
        targetPan - startPan,
        targetTilt - startTilt,
        panVelocity,
        tiltVelocity
    );

    timedOut_ = false;
    beginFinishing(
        now,
        BOOT_REASON_READY,
        durationMs,
        startPan,
        startTilt,
        panVelocity,
        tiltVelocity,
        targetPan,
        targetTilt
    );
    return durationMs;
}

uint16_t BootMotion::cancel(
    uint32_t now,
    float panDegrees,
    float tiltDegrees,
    float panVelocity,
    float tiltVelocity
) {
    const uint16_t durationMs = 800U;
    const float durationSeconds = durationMs * 0.001f;
    const float startPan = clampPan(panDegrees);
    const float startTilt = clampTilt(tiltDegrees);
    const float targetPan = clampPan(startPan + 0.5f * panVelocity * durationSeconds);
    const float targetTilt = clampTilt(startTilt + 0.5f * tiltVelocity * durationSeconds);

    timedOut_ = false;
    beginFinishing(
        now,
        BOOT_REASON_CANCELLED,
        durationMs,
        startPan,
        startTilt,
        panVelocity,
        tiltVelocity,
        targetPan,
        targetTilt
    );
    return durationMs;
}

void BootMotion::abortImmediately(float panDegrees, float tiltDegrees) {
    state_ = BOOT_DONE;
    finishReason_ = BOOT_REASON_DRIVER_STOP;
    timedOut_ = false;
    finishingDurationMs_ = 0;
    sample_.panDegrees = clampPan(panDegrees);
    sample_.tiltDegrees = clampTilt(tiltDegrees);
    sample_.panVelocity = 0.0f;
    sample_.tiltVelocity = 0.0f;
}

void BootMotion::update(uint32_t now) {
    if (state_ == BOOT_OPENING) {
        while (state_ == BOOT_OPENING &&
               static_cast<uint32_t>(now - segmentStartMs_) >= segmentDurationMs_) {
            evaluateActiveCurve(segmentStartMs_ + segmentDurationMs_, true);
            const uint32_t nextStartMs = segmentStartMs_ + segmentDurationMs_;
            ++openingIndex_;
            if (openingIndex_ >= OPENING_FRAME_COUNT) {
                state_ = BOOT_WAIT_LOOP;
                segmentStartMs_ = nextStartMs;
                sample_.panDegrees = WAIT_CENTER_PAN;
                sample_.tiltDegrees = WAIT_CENTER_TILT;
                sample_.panVelocity = 0.0f;
                sample_.tiltVelocity = 0.0f;
                break;
            }
            beginOpeningSegment(
                openingIndex_,
                nextStartMs,
                sample_.panDegrees,
                sample_.tiltDegrees,
                0.0f,
                0.0f
            );
        }
        if (state_ == BOOT_OPENING) {
            evaluateActiveCurve(now);
        }
    }

    if (state_ == BOOT_WAIT_LOOP) {
        const uint32_t bootElapsedMs = static_cast<uint32_t>(now - bootStartMs_);
        if (bootElapsedMs >= BOOT_BRAKE_START_MS) {
            beginTimeout(bootStartMs_ + BOOT_BRAKE_START_MS);
        } else {
            evaluateWaitLoop(static_cast<uint32_t>(now - segmentStartMs_));
        }
    }

    if (state_ == BOOT_FINISHING) {
        const uint32_t elapsedMs = static_cast<uint32_t>(now - segmentStartMs_);
        if (elapsedMs >= segmentDurationMs_) {
            evaluateActiveCurve(segmentStartMs_ + segmentDurationMs_, true);
            state_ = BOOT_DONE;
            sample_.panVelocity = 0.0f;
            sample_.tiltVelocity = 0.0f;
        } else {
            evaluateActiveCurve(now);
        }
    }

    clampSample();
}

BootMotionState BootMotion::state() const {
    return state_;
}

BootFinishReason BootMotion::finishReason() const {
    return finishReason_;
}

const BootMotionSample& BootMotion::sample() const {
    return sample_;
}

bool BootMotion::isDriving() const {
    return state_ == BOOT_OPENING ||
           state_ == BOOT_WAIT_LOOP ||
           state_ == BOOT_FINISHING;
}

bool BootMotion::timedOut() const {
    return timedOut_;
}

uint16_t BootMotion::finishingDurationMs() const {
    return finishingDurationMs_;
}

uint32_t BootMotion::remainingMs(uint32_t now) const {
    if (state_ != BOOT_OPENING && state_ != BOOT_FINISHING) {
        return 0;
    }
    const uint32_t elapsedMs = static_cast<uint32_t>(now - segmentStartMs_);
    if (elapsedMs >= segmentDurationMs_) {
        return 0;
    }
    return static_cast<uint32_t>(segmentDurationMs_) - elapsedMs;
}

float BootMotion::clampPan(float degrees) {
    return clampValue(degrees, PAN_MIN_DEGREES, PAN_MAX_DEGREES);
}

float BootMotion::clampTilt(float degrees) {
    return clampValue(degrees, TILT_MIN_DEGREES, TILT_MAX_DEGREES);
}

float BootMotion::clampValue(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void BootMotion::openingFrame(
    uint8_t index,
    float& panDegrees,
    float& tiltDegrees,
    uint16_t& durationMs
) {
    switch (index) {
#if BOOT_UNIT_ID == 1
        case 0: panDegrees = -6.0f; tiltDegrees = 8.0f; durationMs = 1200U; break;
        case 1: panDegrees = 10.0f; tiltDegrees = 16.0f; durationMs = 2200U; break;
        case 2: panDegrees = -8.0f; tiltDegrees = 28.0f; durationMs = 2800U; break;
        case 3: panDegrees = 12.0f; tiltDegrees = 34.0f; durationMs = 2600U; break;
        case 4: panDegrees = -6.0f; tiltDegrees = 22.0f; durationMs = 2400U; break;
        case 5: panDegrees = 4.0f; tiltDegrees = 12.0f; durationMs = 2200U; break;
        default: panDegrees = 0.0f; tiltDegrees = 16.0f; durationMs = 1300U; break;
#elif BOOT_UNIT_ID == 2
        case 0: panDegrees = 0.0f; tiltDegrees = 8.0f; durationMs = 1200U; break;
        case 1: panDegrees = -4.0f; tiltDegrees = 20.0f; durationMs = 2200U; break;
        case 2: panDegrees = 5.0f; tiltDegrees = 30.0f; durationMs = 2800U; break;
        case 3: panDegrees = -4.0f; tiltDegrees = 38.0f; durationMs = 2600U; break;
        case 4: panDegrees = 4.0f; tiltDegrees = 26.0f; durationMs = 2400U; break;
        case 5: panDegrees = -2.0f; tiltDegrees = 16.0f; durationMs = 2200U; break;
        default: panDegrees = 0.0f; tiltDegrees = 16.0f; durationMs = 1300U; break;
#else
        case 0: panDegrees = 6.0f; tiltDegrees = 8.0f; durationMs = 1200U; break;
        case 1: panDegrees = -10.0f; tiltDegrees = 16.0f; durationMs = 2200U; break;
        case 2: panDegrees = 8.0f; tiltDegrees = 28.0f; durationMs = 2800U; break;
        case 3: panDegrees = -12.0f; tiltDegrees = 34.0f; durationMs = 2600U; break;
        case 4: panDegrees = 6.0f; tiltDegrees = 22.0f; durationMs = 2400U; break;
        case 5: panDegrees = -4.0f; tiltDegrees = 12.0f; durationMs = 2200U; break;
        default: panDegrees = 0.0f; tiltDegrees = 16.0f; durationMs = 1300U; break;
#endif
        /*
        case 0:
            panDegrees = 8.0f;
            tiltDegrees = 4.0f;
            durationMs = 1200U;
            break;
        case 1:
            panDegrees = -12.0f;
            tiltDegrees = 10.0f;
            durationMs = 2200U;
            break;
        case 2:
            panDegrees = 14.0f;
            tiltDegrees = 19.0f;
            durationMs = 2800U;
            break;
        case 3:
            panDegrees = -10.0f;
            tiltDegrees = 24.0f;
            durationMs = 2600U;
            break;
        case 4:
            panDegrees = 12.0f;
            tiltDegrees = 16.0f;
            durationMs = 2400U;
            break;
        case 5:
            panDegrees = -8.0f;
            tiltDegrees = 7.0f;
            durationMs = 2200U;
            break;
        default:
            panDegrees = WAIT_CENTER_PAN;
            tiltDegrees = WAIT_CENTER_TILT;
            durationMs = 1300U;
            break;
        */
    }
}

void BootMotion::configureCurve(
    AxisCurve& curve,
    float startPosition,
    float startVelocity,
    float endPosition,
    float durationSeconds
) {
    const float scaledVelocity = startVelocity * durationSeconds;
    const float distance = endPosition - startPosition;
    curve.coefficient[0] = startPosition;
    curve.coefficient[1] = scaledVelocity;
    curve.coefficient[2] = 0.0f;
    curve.coefficient[3] = 10.0f * distance - 6.0f * scaledVelocity;
    curve.coefficient[4] = -15.0f * distance + 8.0f * scaledVelocity;
    curve.coefficient[5] = 6.0f * distance - 3.0f * scaledVelocity;
}

void BootMotion::evaluateCurve(
    const AxisCurve& curve,
    float durationSeconds,
    float normalizedTime,
    float& position,
    float& velocity
) {
    const float u = clampValue(normalizedTime, 0.0f, 1.0f);
    position = curve.coefficient[0] +
        u * (curve.coefficient[1] +
        u * (curve.coefficient[2] +
        u * (curve.coefficient[3] +
        u * (curve.coefficient[4] +
        u * curve.coefficient[5]))));
    velocity = (curve.coefficient[1] +
        u * (2.0f * curve.coefficient[2] +
        u * (3.0f * curve.coefficient[3] +
        u * (4.0f * curve.coefficient[4] +
        u * 5.0f * curve.coefficient[5])))) / durationSeconds;
}

bool BootMotion::curveIsInside(
    const AxisCurve& curve,
    float durationSeconds,
    float minimum,
    float maximum
) {
    float position;
    float velocity;
    evaluateCurve(curve, durationSeconds, 0.0f, position, velocity);
    if (position < minimum - 0.0005f || position > maximum + 0.0005f) {
        return false;
    }
    evaluateCurve(curve, durationSeconds, 1.0f, position, velocity);
    if (position < minimum - 0.0005f || position > maximum + 0.0005f) {
        return false;
    }

    // With zero start acceleration and zero end velocity/acceleration, the
    // derivative factors as (1-u)^2 * (a*u^2 + b*u + c).  Checking the roots
    // of that quadratic covers every interior position extremum exactly.
    const float a = 5.0f * curve.coefficient[5];
    const float b = 2.0f * curve.coefficient[1];
    const float c = curve.coefficient[1];
    float roots[2];
    uint8_t rootCount = 0;
    if (fabsf(a) < 0.000001f) {
        if (fabsf(b) >= 0.000001f) {
            roots[rootCount++] = -c / b;
        }
    } else {
        const float discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f) {
            const float squareRoot = sqrtf(discriminant);
            roots[rootCount++] = (-b - squareRoot) / (2.0f * a);
            roots[rootCount++] = (-b + squareRoot) / (2.0f * a);
        }
    }

    for (uint8_t index = 0; index < rootCount; ++index) {
        if (roots[index] <= 0.0f || roots[index] >= 1.0f) {
            continue;
        }
        evaluateCurve(curve, durationSeconds, roots[index], position, velocity);
        if (position < minimum - 0.0005f || position > maximum + 0.0005f) {
            return false;
        }
    }
    return true;
}

uint16_t BootMotion::chooseFinishDuration(
    float panDistance,
    float tiltDistance,
    float panVelocity,
    float tiltVelocity
) {
    const float distance = larger(absoluteValue(panDistance), absoluteValue(tiltDistance));
    const float speed = larger(absoluteValue(panVelocity), absoluteValue(tiltVelocity));
    float durationMs = 800.0f + 35.0f * distance + 8.0f * speed;
    if (durationMs < 800.0f) {
        durationMs = 800.0f;
    }
    if (durationMs > 3000.0f) {
        durationMs = 3000.0f;
    }
    return static_cast<uint16_t>(durationMs + 0.5f);
}

void BootMotion::beginOpeningSegment(
    uint8_t index,
    uint32_t startMs,
    float startPan,
    float startTilt,
    float startPanVelocity,
    float startTiltVelocity
) {
    float endPan;
    float endTilt;
    uint16_t durationMs;
    openingFrame(index, endPan, endTilt, durationMs);
    beginFinishing(
        startMs,
        BOOT_REASON_NONE,
        durationMs,
        startPan,
        startTilt,
        startPanVelocity,
        startTiltVelocity,
        clampPan(endPan),
        clampTilt(endTilt)
    );
    state_ = BOOT_OPENING;
    finishReason_ = BOOT_REASON_NONE;
    finishingDurationMs_ = 0;
    openingIndex_ = index;
}

void BootMotion::evaluateActiveCurve(uint32_t now, bool exactEnd) {
    if (exactEnd) {
        sample_ = segmentEndSample_;
        return;
    }
    const float durationSeconds = segmentDurationMs_ * 0.001f;
    const float normalizedTime = static_cast<uint32_t>(now - segmentStartMs_) /
        static_cast<float>(segmentDurationMs_);
    evaluateCurve(
        panCurve_,
        durationSeconds,
        normalizedTime,
        sample_.panDegrees,
        sample_.panVelocity
    );
    evaluateCurve(
        tiltCurve_,
        durationSeconds,
        normalizedTime,
        sample_.tiltDegrees,
        sample_.tiltVelocity
    );
}

void BootMotion::evaluateWaitLoop(uint32_t waitElapsedMs) {
    const float elapsedSeconds = waitElapsedMs * 0.001f;
    const float phase = TWO_PI * elapsedSeconds / (WAIT_PERIOD_MS * 0.001f);
    float envelope = 1.0f;
    float envelopeVelocity = 0.0f;
    if (waitElapsedMs < WAIT_BLEND_MS) {
        const float u = waitElapsedMs / static_cast<float>(WAIT_BLEND_MS);
        envelope = minimumJerk(u);
        envelopeVelocity = minimumJerkDerivative(u) / (WAIT_BLEND_MS * 0.001f);
    }
    const float phaseVelocity = TWO_PI / (WAIT_PERIOD_MS * 0.001f);
    const float phaseCosine = cosf(phase);
    const float phaseSine = sinf(phase);

    sample_.panDegrees = WAIT_CENTER_PAN + envelope * WAIT_RADIUS_PAN * phaseCosine;
    sample_.tiltDegrees = WAIT_CENTER_TILT + envelope * WAIT_RADIUS_TILT * phaseSine;
    sample_.panVelocity = WAIT_RADIUS_PAN * (
        envelopeVelocity * phaseCosine - envelope * phaseVelocity * phaseSine
    );
    sample_.tiltVelocity = WAIT_RADIUS_TILT * (
        envelopeVelocity * phaseSine + envelope * phaseVelocity * phaseCosine
    );
}

void BootMotion::beginFinishing(
    uint32_t now,
    BootFinishReason reason,
    uint16_t durationMs,
    float panDegrees,
    float tiltDegrees,
    float panVelocity,
    float tiltVelocity,
    float targetPanDegrees,
    float targetTiltDegrees
) {
    const float durationSeconds = durationMs * 0.001f;
    float safePanVelocity = panVelocity;
    float safeTiltVelocity = tiltVelocity;
    const float startPan = clampPan(panDegrees);
    const float startTilt = clampTilt(tiltDegrees);
    const float targetPan = clampPan(targetPanDegrees);
    const float targetTilt = clampTilt(targetTiltDegrees);

    bool panCurveAccepted = false;
    bool tiltCurveAccepted = false;
    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        if (!panCurveAccepted) {
            configureCurve(
                panCurve_, startPan, safePanVelocity, targetPan, durationSeconds
            );
            panCurveAccepted = curveIsInside(
                panCurve_, durationSeconds, PAN_MIN_DEGREES, PAN_MAX_DEGREES
            );
            if (!panCurveAccepted) {
                safePanVelocity *= 0.5f;
            }
        }
        if (!tiltCurveAccepted) {
            configureCurve(
                tiltCurve_, startTilt, safeTiltVelocity, targetTilt, durationSeconds
            );
            tiltCurveAccepted = curveIsInside(
                tiltCurve_, durationSeconds, TILT_MIN_DEGREES, TILT_MAX_DEGREES
            );
            if (!tiltCurveAccepted) {
                safeTiltVelocity *= 0.5f;
            }
        }
        if (panCurveAccepted && tiltCurveAccepted) {
            break;
        }
    }
    if (!panCurveAccepted) {
        safePanVelocity = 0.0f;
        configureCurve(panCurve_, startPan, 0.0f, targetPan, durationSeconds);
    }
    if (!tiltCurveAccepted) {
        safeTiltVelocity = 0.0f;
        configureCurve(tiltCurve_, startTilt, 0.0f, targetTilt, durationSeconds);
    }

    segmentStartMs_ = now;
    segmentDurationMs_ = durationMs;
    finishingDurationMs_ = durationMs;
    finishReason_ = reason;
    state_ = BOOT_FINISHING;
    sample_.panDegrees = startPan;
    sample_.tiltDegrees = startTilt;
    sample_.panVelocity = safePanVelocity;
    sample_.tiltVelocity = safeTiltVelocity;
    segmentEndSample_.panDegrees = targetPan;
    segmentEndSample_.tiltDegrees = targetTilt;
    segmentEndSample_.panVelocity = 0.0f;
    segmentEndSample_.tiltVelocity = 0.0f;
}

void BootMotion::beginTimeout(uint32_t timeoutStartMs) {
    const uint32_t waitElapsedAtTimeout = BOOT_BRAKE_START_MS - OPENING_TOTAL_MS;
    evaluateWaitLoop(waitElapsedAtTimeout);
    const float durationSeconds = BOOT_BRAKE_DURATION_MS * 0.001f;
    const float targetPan = clampPan(
        sample_.panDegrees + 0.5f * sample_.panVelocity * durationSeconds
    );
    const float targetTilt = clampTilt(
        sample_.tiltDegrees + 0.5f * sample_.tiltVelocity * durationSeconds
    );
    const float pan = sample_.panDegrees;
    const float tilt = sample_.tiltDegrees;
    const float panVelocity = sample_.panVelocity;
    const float tiltVelocity = sample_.tiltVelocity;

    timedOut_ = true;
    beginFinishing(
        timeoutStartMs,
        BOOT_REASON_TIMEOUT,
        BOOT_BRAKE_DURATION_MS,
        pan,
        tilt,
        panVelocity,
        tiltVelocity,
        targetPan,
        targetTilt
    );
    timedOut_ = true;
}

void BootMotion::clampSample() {
    const float clampedPan = clampPan(sample_.panDegrees);
    const float clampedTilt = clampTilt(sample_.tiltDegrees);
    if (clampedPan != sample_.panDegrees) {
        sample_.panDegrees = clampedPan;
        sample_.panVelocity = 0.0f;
    }
    if (clampedTilt != sample_.tiltDegrees) {
        sample_.tiltDegrees = clampedTilt;
        sample_.tiltVelocity = 0.0f;
    }
}
