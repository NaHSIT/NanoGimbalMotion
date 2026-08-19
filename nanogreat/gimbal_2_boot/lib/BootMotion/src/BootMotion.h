#pragma once

#include <stdint.h>

enum BootMotionState : uint8_t {
    BOOT_IDLE = 0,
    BOOT_OPENING,
    BOOT_WAIT_LOOP,
    BOOT_FINISHING,
    BOOT_DONE
};

enum BootFinishReason : uint8_t {
    BOOT_REASON_NONE = 0,
    BOOT_REASON_READY,
    BOOT_REASON_CANCELLED,
    BOOT_REASON_TIMEOUT,
    BOOT_REASON_DRIVER_STOP
};

struct BootMotionSample {
    float panDegrees;
    float tiltDegrees;
    float panVelocity;
    float tiltVelocity;
};

class BootMotion {
public:
    static const float PAN_MIN_DEGREES;
    static const float PAN_MAX_DEGREES;
    static const float TILT_MIN_DEGREES;
    static const float TILT_MAX_DEGREES;

    BootMotion();

    void reset(float panDegrees, float tiltDegrees);
    void start(
        uint32_t now,
        float panDegrees,
        float tiltDegrees,
        float panVelocity,
        float tiltVelocity
    );
    uint16_t finishTo(
        uint32_t now,
        float panDegrees,
        float tiltDegrees,
        float panVelocity,
        float tiltVelocity,
        float targetPanDegrees,
        float targetTiltDegrees
    );
    uint16_t cancel(
        uint32_t now,
        float panDegrees,
        float tiltDegrees,
        float panVelocity,
        float tiltVelocity
    );
    void abortImmediately(float panDegrees, float tiltDegrees);
    void update(uint32_t now);

    BootMotionState state() const;
    BootFinishReason finishReason() const;
    const BootMotionSample& sample() const;
    bool isDriving() const;
    bool timedOut() const;
    uint16_t finishingDurationMs() const;
    uint32_t remainingMs(uint32_t now) const;

    static float clampPan(float degrees);
    static float clampTilt(float degrees);

private:
    struct AxisCurve {
        float coefficient[6];
    };

    BootMotionState state_;
    BootFinishReason finishReason_;
    BootMotionSample sample_;
    BootMotionSample segmentEndSample_;
    AxisCurve panCurve_;
    AxisCurve tiltCurve_;
    uint32_t bootStartMs_;
    uint32_t segmentStartMs_;
    uint16_t segmentDurationMs_;
    uint16_t finishingDurationMs_;
    uint8_t openingIndex_;
    bool timedOut_;

    static float clampValue(float value, float minimum, float maximum);
    static void openingFrame(
        uint8_t index,
        float& panDegrees,
        float& tiltDegrees,
        uint16_t& durationMs
    );
    static void configureCurve(
        AxisCurve& curve,
        float startPosition,
        float startVelocity,
        float endPosition,
        float durationSeconds
    );
    static void evaluateCurve(
        const AxisCurve& curve,
        float durationSeconds,
        float normalizedTime,
        float& position,
        float& velocity
    );
    static bool curveIsInside(
        const AxisCurve& curve,
        float durationSeconds,
        float minimum,
        float maximum
    );
    static uint16_t chooseFinishDuration(
        float panDistance,
        float tiltDistance,
        float panVelocity,
        float tiltVelocity
    );

    void beginOpeningSegment(
        uint8_t index,
        uint32_t startMs,
        float startPan,
        float startTilt,
        float startPanVelocity,
        float startTiltVelocity
    );
    void evaluateActiveCurve(uint32_t now, bool exactEnd = false);
    void evaluateWaitLoop(uint32_t waitElapsedMs);
    void beginFinishing(
        uint32_t now,
        BootFinishReason reason,
        uint16_t durationMs,
        float panDegrees,
        float tiltDegrees,
        float panVelocity,
        float tiltVelocity,
        float targetPanDegrees,
        float targetTiltDegrees
    );
    void beginTimeout(uint32_t timeoutStartMs);
    void clampSample();
};
