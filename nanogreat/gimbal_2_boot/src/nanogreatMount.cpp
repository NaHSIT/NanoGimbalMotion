#include "nanogreatMount.h"

#include "panTiltMount.h"

#include <AccelStepper.h>
#include <BootMotion.h>
#include <math.h>
#include <stdlib.h>

namespace {

AccelStepper pan(AccelStepper::DRIVER, PIN_STEP_PAN, PIN_DIRECTION_PAN);
AccelStepper tilt(AccelStepper::DRIVER, PIN_STEP_TILT, PIN_DIRECTION_TILT);
AccelStepper slider(AccelStepper::DRIVER, PIN_STEP_SLIDER, PIN_DIRECTION_SLIDER);

BootMotion bootMotion;

long targets[3] = {0, 0, 0};
int stepMode = SIXTEENTH_STEP;
bool driversEnabled = true;
bool bootTargetsPending = false;
bool sliderMotionPending = false;
float sliderTargetMm = 0.0f;
long sliderTargetSteps = 0;

float panMaxDegPerSecond = 18.0f;
float tiltMaxDegPerSecond = 10.0f;
float sliderMaxMmPerSecond = 20.0f;
float panStepsPerDegree = 1.0f;
float tiltStepsPerDegree = 1.0f;
float sliderStepsPerMm = 1.0f;
float panDriveVelocity = 0.0f;
float tiltDriveVelocity = 0.0f;

bool textCommandPending = false;
bool pendingValueOverflow = false;
char pendingCommand = 0;
char pendingValue[32] = {0};
uint8_t pendingValueLength = 0;

// Q 的完整响应超过 ATmega328P 的 64 字节串口发送缓冲区。这里先生成
// 固定长度快照，再按 availableForWrite() 分段发送，避免查询状态时阻塞步进。
char healthTxBuffer[112] = {0};
uint8_t healthTxLength = 0;
uint8_t healthTxOffset = 0;
bool sliderDoneReplyDeferred = false;
bool driverStoppedReplyDeferred = false;

const float BOOT_AXIS_MAX_DEGREES_PER_SECOND = 80.0f;
const float WAIT_LOOP_FOLLOW_GAIN = 20.0f;
const float FINAL_SETTLE_DEGREES_PER_SECOND = 12.0f;

float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

float clampValue(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool finiteNumber(float value) {
    return !isnan(value) && !isinf(value);
}

long safeAngleToSteps(float degrees, float stepsPerDegree) {
    const float exactSteps = degrees * stepsPerDegree;
    if (exactSteps > 0.0f) {
        return static_cast<long>(floorf(exactSteps));
    }
    if (exactSteps < 0.0f) {
        return static_cast<long>(ceilf(exactSteps));
    }
    return 0;
}

long sliderToSteps(float millimeters) {
    return lroundf(millimeters * sliderStepsPerMm);
}

float currentPanDegrees() {
    return pan.currentPosition() / panStepsPerDegree - PAN_ANGLE_OFFSET_DEGREES;
}

float signedTiltStepsPerDegree() {
    return tiltStepsPerDegree * TILT_DIRECTION;
}

float currentTiltDegrees() {
    return tilt.currentPosition() / signedTiltStepsPerDegree() - TILT_ANGLE_OFFSET_DEGREES;
}

float currentPanVelocity() {
    return panDriveVelocity;
}

float currentTiltVelocity() {
    return tiltDriveVelocity;
}

bool axesMoving() {
    if (!driversEnabled) {
        return false;
    }
    return bootMotion.isDriving() ||
           pan.currentPosition() != targets[0] ||
           tilt.currentPosition() != targets[1] ||
           slider.currentPosition() != targets[2];
}

BootMotionState reportedBootState() {
    // 轨迹规划到时不等于电机已经走完最后的量化步。F/C 收尾期间继续对外
    // 报告 FINISHING，直到 Pan/Tilt 两轴的物理步数都到达最终目标。
    if (bootMotion.state() == BOOT_DONE && bootTargetsPending) {
        return BOOT_FINISHING;
    }
    return bootMotion.state();
}

void appendHealthChar(char value) {
    if (healthTxLength < sizeof(healthTxBuffer)) {
        healthTxBuffer[healthTxLength++] = value;
    }
}

void appendHealthText(PGM_P text) {
    char value;
    while ((value = static_cast<char>(pgm_read_byte(text++))) != '\0') {
        appendHealthChar(value);
    }
}

void appendHealthUnsigned(unsigned long value) {
    char digits[10];
    uint8_t length = 0;
    do {
        digits[length++] = static_cast<char>('0' + value % 10UL);
        value /= 10UL;
    } while (value > 0UL && length < sizeof(digits));
    while (length > 0) {
        appendHealthChar(digits[--length]);
    }
}

void appendHealthLong(long value) {
    if (value < 0) {
        appendHealthChar('-');
        const unsigned long magnitude =
            static_cast<unsigned long>(-(value + 1L)) + 1UL;
        appendHealthUnsigned(magnitude);
        return;
    }
    appendHealthUnsigned(static_cast<unsigned long>(value));
}

void appendHealthFixed2(float value) {
    const bool negative = value < 0.0f;
    const float magnitude = negative ? -value : value;
    const unsigned long scaled = static_cast<unsigned long>(
        floorf(magnitude * 100.0f + 0.5f)
    );
    if (negative && scaled > 0UL) {
        appendHealthChar('-');
    }
    appendHealthUnsigned(scaled / 100UL);
    appendHealthChar('.');
    appendHealthChar(static_cast<char>('0' + (scaled / 10UL) % 10UL));
    appendHealthChar(static_cast<char>('0' + scaled % 10UL));
}

void appendBootStateText(BootMotionState state) {
    switch (state) {
        case BOOT_IDLE:
            appendHealthText(PSTR("IDLE"));
            break;
        case BOOT_OPENING:
            appendHealthText(PSTR("OPENING"));
            break;
        case BOOT_WAIT_LOOP:
            appendHealthText(PSTR("WAIT_LOOP"));
            break;
        case BOOT_FINISHING:
            appendHealthText(PSTR("FINISHING"));
            break;
        default:
            appendHealthText(PSTR("DONE"));
            break;
    }
}

bool healthReplyPending() {
    return healthTxOffset < healthTxLength;
}

void queueNanoHealth() {
    healthTxLength = 0;
    healthTxOffset = 0;
    appendHealthText(PSTR("NANO_OK v=2 en="));
    appendHealthUnsigned(driversEnabled ? 1UL : 0UL);
    appendHealthText(PSTR(" moving="));
    appendHealthUnsigned(axesMoving() ? 1UL : 0UL);
    appendHealthText(PSTR(" slider_steps="));
    appendHealthLong(slider.currentPosition());
    appendHealthText(PSTR(" boot="));
    appendBootStateText(reportedBootState());
    appendHealthText(PSTR(" pan="));
    appendHealthFixed2(currentPanDegrees());
    appendHealthText(PSTR(" tilt="));
    appendHealthFixed2(currentTiltDegrees());
    appendHealthText(PSTR(" timeout="));
    appendHealthUnsigned(bootMotion.timedOut() ? 1UL : 0UL);
    appendHealthChar('\n');
}

void serviceHealthReply() {
    if (!healthReplyPending()) {
        return;
    }
    const int available = Serial.availableForWrite();
    if (available <= 0) {
        return;
    }
    const uint8_t remaining = healthTxLength - healthTxOffset;
    const uint8_t chunk = static_cast<uint8_t>(
        available < remaining ? available : remaining
    );
    const size_t written = Serial.write(
        reinterpret_cast<const uint8_t*>(healthTxBuffer + healthTxOffset),
        chunk
    );
    healthTxOffset += static_cast<uint8_t>(written);
    if (healthTxOffset >= healthTxLength) {
        healthTxOffset = 0;
        healthTxLength = 0;
    }
}

void printSliderDone() {
    if (healthReplyPending()) {
        sliderDoneReplyDeferred = true;
        return;
    }
    Serial.print(F("SLIDER_DONE target_mm="));
    Serial.print(sliderTargetMm, 2);
    Serial.print(F(" pos_steps="));
    Serial.println(slider.currentPosition());
}

void serviceDeferredReplies() {
    if (healthReplyPending()) {
        return;
    }
    if (driverStoppedReplyDeferred) {
        driverStoppedReplyDeferred = false;
        Serial.println(F("ERR driver stopped"));
    }
    if (sliderDoneReplyDeferred) {
        sliderDoneReplyDeferred = false;
        printSliderDone();
    }
}

void printStatus() {
    Serial.println(F("Status"));
    Serial.print(F("Enable state: "));
    Serial.println(driversEnabled ? 1 : 0);
    Serial.print(F("POS pan="));
    Serial.print(pan.currentPosition());
    Serial.print(F(" tilt="));
    Serial.print(tilt.currentPosition());
    Serial.print(F(" slider="));
    Serial.println(slider.currentPosition());
}

void applyMaxSpeeds() {
    pan.setMaxSpeed(absoluteValue(panMaxDegPerSecond * panStepsPerDegree));
    tilt.setMaxSpeed(absoluteValue(tiltMaxDegPerSecond * tiltStepsPerDegree));
    slider.setMaxSpeed(absoluteValue(sliderMaxMmPerSecond * sliderStepsPerMm));
}

void setStepMode(int mode) {
    if (mode != HALF_STEP && mode != QUARTER_STEP &&
        mode != EIGHTH_STEP && mode != SIXTEENTH_STEP) {
        Serial.println(F("ERR step mode: use 2,4,8,16"));
        return;
    }

    digitalWrite(PIN_MS1, mode == HALF_STEP || mode == SIXTEENTH_STEP);
    digitalWrite(PIN_MS2, mode == QUARTER_STEP || mode == SIXTEENTH_STEP);
    stepMode = mode;
    panStepsPerDegree = (200.0f * stepMode * PAN_GEAR_RATIO) / 360.0f;
    tiltStepsPerDegree = (200.0f * stepMode * TILT_GEAR_RATIO) / 360.0f;
    sliderStepsPerMm = (200.0f * stepMode) / (SLIDER_PULLEY_TEETH * 2.0f);
    applyMaxSpeeds();
    Serial.print(F("OK step mode="));
    Serial.println(stepMode);
}

void freezeAllAxes() {
    targets[0] = pan.currentPosition();
    targets[1] = tilt.currentPosition();
    targets[2] = slider.currentPosition();
    pan.moveTo(targets[0]);
    tilt.moveTo(targets[1]);
    slider.moveTo(targets[2]);
    pan.setSpeed(0.0f);
    tilt.setSpeed(0.0f);
    slider.setSpeed(0.0f);
    panDriveVelocity = 0.0f;
    tiltDriveVelocity = 0.0f;
    bootTargetsPending = false;
    sliderMotionPending = false;
    bootMotion.abortImmediately(currentPanDegrees(), currentTiltDegrees());
}

void applyBootTarget() {
    const BootMotionSample& sample = bootMotion.sample();
    const float safePan = BootMotion::clampPan(sample.panDegrees);
    const float safeTilt = BootMotion::clampTilt(sample.tiltDegrees);
    targets[0] = safeAngleToSteps(
        safePan + PAN_ANGLE_OFFSET_DEGREES,
        panStepsPerDegree
    );
    targets[1] = safeAngleToSteps(
        safeTilt + TILT_ANGLE_OFFSET_DEGREES,
        signedTiltStepsPerDegree()
    );
    pan.moveTo(targets[0]);
    tilt.moveTo(targets[1]);
}

void serviceBootAxis(
    AccelStepper& axis,
    float stepsPerDegree,
    float referenceVelocity,
    float& driveVelocity,
    uint32_t remainingMs,
    bool waitLoop
) {
    const long distance = axis.distanceToGo();
    if (distance == 0) {
        const float referenceSpeed = absoluteValue(referenceVelocity * stepsPerDegree);
        if (referenceSpeed >= 1.0f) {
            axis.setSpeed(referenceSpeed);
            driveVelocity = referenceVelocity;
        } else {
            axis.setSpeed(0.0f);
            driveVelocity = 0.0f;
        }
        return;
    }

    const float maximumSpeed = absoluteValue(
        BOOT_AXIS_MAX_DEGREES_PER_SECOND * stepsPerDegree
    );
    float stepSpeed = 0.0f;
    if (remainingMs > 0) {
        stepSpeed = absoluteValue(static_cast<float>(distance)) * 1000.0f /
            static_cast<float>(remainingMs);
    }
    const float referenceSpeed = absoluteValue(referenceVelocity * stepsPerDegree);
    if (referenceSpeed > stepSpeed) {
        stepSpeed = referenceSpeed;
    }
    if (waitLoop) {
        const float followSpeed = absoluteValue(static_cast<float>(distance)) *
            WAIT_LOOP_FOLLOW_GAIN;
        if (followSpeed > stepSpeed) {
            stepSpeed = followSpeed;
        }
    } else if (remainingMs == 0 && stepSpeed < 1.0f) {
        stepSpeed = absoluteValue(
            FINAL_SETTLE_DEGREES_PER_SECOND * stepsPerDegree
        );
    }
    stepSpeed = clampValue(stepSpeed, 1.0f, maximumSpeed);
    axis.setMaxSpeed(maximumSpeed);
    axis.setSpeed(stepSpeed);
    driveVelocity = (distance > 0 ? stepSpeed : -stepSpeed) / stepsPerDegree;
    axis.runSpeedToPosition();
}

void serviceLegacyAxis(
    AccelStepper& axis,
    float stepsPerDegree,
    float degreesPerSecond,
    float& driveVelocity
) {
    const long distance = axis.distanceToGo();
    const float stepSpeed = absoluteValue(degreesPerSecond * stepsPerDegree);
    if (distance == 0 || stepSpeed < 1.0f) {
        axis.setSpeed(0.0f);
        driveVelocity = 0.0f;
        return;
    }
    axis.setMaxSpeed(stepSpeed);
    axis.setSpeed(stepSpeed);
    driveVelocity = (distance > 0 ? stepSpeed : -stepSpeed) / stepsPerDegree;
    axis.runSpeedToPosition();
}

void servicePanTilt() {
    if (bootMotion.isDriving() || bootTargetsPending) {
        applyBootTarget();
        const uint32_t remainingMs = bootMotion.remainingMs(millis());
        const bool waitLoop = bootMotion.state() == BOOT_WAIT_LOOP;
        serviceBootAxis(
            pan,
            panStepsPerDegree,
            bootMotion.sample().panVelocity,
            panDriveVelocity,
            remainingMs,
            waitLoop
        );
        serviceBootAxis(
            tilt,
            signedTiltStepsPerDegree(),
            bootMotion.sample().tiltVelocity,
            tiltDriveVelocity,
            remainingMs,
            waitLoop
        );
        if (!bootMotion.isDriving() &&
            pan.currentPosition() == targets[0] &&
            tilt.currentPosition() == targets[1]) {
            bootTargetsPending = false;
        }
        return;
    }

    serviceLegacyAxis(pan, panStepsPerDegree, panMaxDegPerSecond, panDriveVelocity);
    serviceLegacyAxis(tilt, signedTiltStepsPerDegree(), tiltMaxDegPerSecond, tiltDriveVelocity);
}

void serviceSlider() {
    const long distance = slider.distanceToGo();
    const float stepSpeed = absoluteValue(sliderMaxMmPerSecond * sliderStepsPerMm);
    if (distance == 0 || stepSpeed < 1.0f) {
        slider.setSpeed(0.0f);
    } else {
        slider.setMaxSpeed(stepSpeed);
        slider.setSpeed(stepSpeed);
        slider.runSpeedToPosition();
    }

    if (sliderMotionPending && slider.currentPosition() == sliderTargetSteps) {
        sliderMotionPending = false;
        printSliderDone();
    }
}

void enforceBootTimeoutStop() {
    if (!bootTargetsPending ||
        bootMotion.state() != BOOT_DONE ||
        bootMotion.finishReason() != BOOT_REASON_TIMEOUT) {
        return;
    }
    targets[0] = pan.currentPosition();
    targets[1] = tilt.currentPosition();
    pan.moveTo(targets[0]);
    tilt.moveTo(targets[1]);
    pan.setSpeed(0.0f);
    tilt.setSpeed(0.0f);
    panDriveVelocity = 0.0f;
    tiltDriveVelocity = 0.0f;
    bootTargetsPending = false;
}

bool parseStrictNumber(const char* text, float& value) {
    if (text == 0 || *text == '\0') {
        return false;
    }
    char* end = 0;
    const double parsed = strtod(text, &end);
    if (end == text || *end != '\0') {
        return false;
    }
    value = static_cast<float>(parsed);
    return finiteNumber(value);
}

bool parseFinishTarget(const char* text, float& panTarget, float& tiltTarget) {
    if (text == 0 || text[0] != ',') {
        return false;
    }

    char* panEnd = 0;
    const double parsedPan = strtod(text + 1, &panEnd);
    if (panEnd == text + 1 || *panEnd != ',') {
        return false;
    }

    char* tiltEnd = 0;
    const double parsedTilt = strtod(panEnd + 1, &tiltEnd);
    if (tiltEnd == panEnd + 1 || *tiltEnd != '\0') {
        return false;
    }

    panTarget = static_cast<float>(parsedPan);
    tiltTarget = static_cast<float>(parsedTilt);
    return finiteNumber(panTarget) && finiteNumber(tiltTarget);
}

bool commandHasNoValue(const char* value) {
    return value != 0 && value[0] == '\0';
}

bool manualPanTiltAllowed() {
    if (bootMotion.isDriving() || bootTargetsPending) {
        Serial.println(F("ERR boot active: use F or C"));
        return false;
    }
    return true;
}

void setPanTarget(float degrees) {
    const float safeDegrees = BootMotion::clampPan(degrees);
    targets[0] = safeAngleToSteps(
        safeDegrees + PAN_ANGLE_OFFSET_DEGREES,
        panStepsPerDegree
    );
    pan.moveTo(targets[0]);
}

void setTiltTarget(float degrees) {
    const float safeDegrees = BootMotion::clampTilt(degrees);
    targets[1] = safeAngleToSteps(
        safeDegrees + TILT_ANGLE_OFFSET_DEGREES,
        signedTiltStepsPerDegree()
    );
    tilt.moveTo(targets[1]);
}

void setSliderTarget(float millimeters) {
    sliderTargetMm = millimeters;
    sliderTargetSteps = sliderToSteps(millimeters);
    targets[2] = sliderTargetSteps;
    slider.moveTo(targets[2]);
    sliderMotionPending = slider.currentPosition() != sliderTargetSteps;
    if (!sliderMotionPending) {
        printSliderDone();
    }
}

void startBootCommand(uint32_t now) {
    if (!driversEnabled) {
        Serial.println(F("ERR B drivers disabled"));
        return;
    }
    if (bootMotion.isDriving() || bootTargetsPending) {
        Serial.println(F("OK B active"));
        return;
    }
    if (absoluteValue(currentPanDegrees()) > 0.25f ||
        absoluteValue(currentTiltDegrees()) > 0.25f ||
        absoluteValue(currentPanVelocity()) > 0.25f ||
        absoluteValue(currentTiltVelocity()) > 0.25f) {
        Serial.println(F("ERR B not at power-on reference"));
        return;
    }
    bootMotion.start(
        now,
        currentPanDegrees(),
        currentTiltDegrees(),
        currentPanVelocity(),
        currentTiltVelocity()
    );
    bootTargetsPending = true;
    Serial.println(F("OK B state=OPENING"));
}

void finishBootCommand(uint32_t now, float targetPan, float targetTilt) {
    if (!driversEnabled) {
        Serial.println(F("ERR F drivers disabled"));
        return;
    }
    const float safePan = BootMotion::clampPan(targetPan);
    const float safeTilt = BootMotion::clampTilt(targetTilt);
    const uint16_t durationMs = bootMotion.finishTo(
        now,
        currentPanDegrees(),
        currentTiltDegrees(),
        currentPanVelocity(),
        currentTiltVelocity(),
        safePan,
        safeTilt
    );
    bootTargetsPending = true;
    Serial.print(F("OK F pan="));
    Serial.print(safePan, 2);
    Serial.print(F(" tilt="));
    Serial.print(safeTilt, 2);
    Serial.print(F(" ms="));
    Serial.println(durationMs);
}

void cancelBootCommand(uint32_t now) {
    if (!driversEnabled) {
        freezeAllAxes();
        Serial.println(F("OK C stopped"));
        return;
    }
    const uint16_t durationMs = bootMotion.cancel(
        now,
        currentPanDegrees(),
        currentTiltDegrees(),
        currentPanVelocity(),
        currentTiltVelocity()
    );
    bootTargetsPending = true;
    Serial.print(F("OK C ms="));
    Serial.println(durationMs);
}

void toggleDrivers() {
    if (driversEnabled) {
        freezeAllAxes();
        driversEnabled = false;
        digitalWrite(PIN_ENABLE, HIGH);
        Serial.println(F("Disabled"));
    } else {
        targets[0] = pan.currentPosition();
        targets[1] = tilt.currentPosition();
        targets[2] = slider.currentPosition();
        pan.moveTo(targets[0]);
        tilt.moveTo(targets[1]);
        slider.moveTo(targets[2]);
        digitalWrite(PIN_ENABLE, LOW);
        driversEnabled = true;
        Serial.println(F("Enabled"));
    }
}

void executeTextCommand(char command, const char* value, uint32_t now) {
    float number = 0.0f;
    switch (command) {
        case 'B':
            if (!commandHasNoValue(value)) {
                Serial.println(F("ERR B format"));
            } else {
                startBootCommand(now);
            }
            break;

        case 'F': {
            float targetPan;
            float targetTilt;
            if (!parseFinishTarget(value, targetPan, targetTilt)) {
                Serial.println(F("ERR F format: F,<pan>,<tilt>"));
            } else {
                finishBootCommand(now, targetPan, targetTilt);
            }
            break;
        }

        case 'C':
            if (!commandHasNoValue(value)) {
                Serial.println(F("ERR C format"));
            } else {
                cancelBootCommand(now);
            }
            break;

        case 'Q':
            if (!commandHasNoValue(value)) {
                Serial.println(F("ERR Q format"));
            } else {
                queueNanoHealth();
            }
            break;

        case 'e':
            if (!commandHasNoValue(value)) {
                Serial.println(F("ERR e format"));
            } else {
                toggleDrivers();
            }
            break;

        case 'm':
            if (!manualPanTiltAllowed()) {
                break;
            }
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR m value"));
            } else {
                setStepMode(static_cast<int>(number));
            }
            break;

        case 'p':
            if (!manualPanTiltAllowed()) {
                break;
            }
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR p value"));
            } else {
                setPanTarget(number);
            }
            break;

        case 't':
            if (!manualPanTiltAllowed()) {
                break;
            }
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR t value"));
            } else {
                setTiltTarget(number);
            }
            break;

        case 'x':
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR x value"));
            } else {
                setSliderTarget(number);
            }
            break;

        case 's':
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR s value"));
            } else {
                panMaxDegPerSecond = clampValue(absoluteValue(number), 0.0f, 60.0f);
                applyMaxSpeeds();
            }
            break;

        case 'S':
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR S value"));
            } else {
                tiltMaxDegPerSecond = clampValue(absoluteValue(number), 0.0f, 60.0f);
                applyMaxSpeeds();
            }
            break;

        case 'X':
            if (!parseStrictNumber(value, number)) {
                Serial.println(F("ERR X value"));
            } else {
                sliderMaxMmPerSecond = absoluteValue(number);
                applyMaxSpeeds();
            }
            break;

        case 'R':
            if (!commandHasNoValue(value)) {
                Serial.println(F("ERR R format"));
            } else {
                printStatus();
            }
            break;

        default:
            Serial.println(F("ERR unknown command"));
            break;
    }
}

void resetTextCommand() {
    textCommandPending = false;
    pendingValueOverflow = false;
    pendingCommand = 0;
    pendingValueLength = 0;
    pendingValue[0] = '\0';
}

void finishTextCommand(uint32_t now) {
    pendingValue[pendingValueLength] = '\0';
    if (pendingValueOverflow) {
        Serial.println(F("ERR command too long"));
    } else {
        executeTextCommand(pendingCommand, pendingValue, now);
    }
    resetTextCommand();
}

void serviceSerial(uint32_t now) {
    // Q 尚未完整进入硬件发送缓冲区时，暂缓解析下一条命令，确保响应行不会
    // 与后续命令回执交叉；Nano 的运动服务仍会在每轮正常执行。
    if (healthReplyPending()) {
        return;
    }
    while (Serial.available()) {
        const char value = static_cast<char>(Serial.read());
        if (!textCommandPending) {
            if (value == '\r' || value == '\n') {
                continue;
            }
            pendingCommand = value;
            textCommandPending = true;
            pendingValueLength = 0;
            pendingValueOverflow = false;
            continue;
        }

        if (value == '\r' || value == '\n') {
            finishTextCommand(now);
            if (healthReplyPending()) {
                return;
            }
            continue;
        }

        if (pendingValueLength < sizeof(pendingValue) - 1) {
            pendingValue[pendingValueLength++] = value;
        } else {
            pendingValueOverflow = true;
        }
    }
}

}  // namespace

void initNanogreat() {
    Serial.begin(BAUD_RATE);

    pinMode(PIN_MS1, OUTPUT);
    pinMode(PIN_MS2, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_STEP_PAN, OUTPUT);
    pinMode(PIN_DIRECTION_PAN, OUTPUT);
    pinMode(PIN_STEP_TILT, OUTPUT);
    pinMode(PIN_DIRECTION_TILT, OUTPUT);
    pinMode(PIN_STEP_SLIDER, OUTPUT);
    pinMode(PIN_DIRECTION_SLIDER, OUTPUT);

    pan.setCurrentPosition(0);
    tilt.setCurrentPosition(0);
    slider.setCurrentPosition(0);
    targets[0] = targets[1] = targets[2] = 0;
    setStepMode(SIXTEENTH_STEP);

    digitalWrite(PIN_ENABLE, LOW);
    driversEnabled = true;
    bootMotion.reset(0.0f, 0.0f);

    // nanogreat 上电建立零点后立即启动开场动画，不依赖 ESP8266 发送 B。
    // 保留 B 指令用于兼容：动画运行期间收到 B 会回复 "OK B active"。
    bootMotion.start(millis(), 0.0f, 0.0f, 0.0f, 0.0f);
    bootTargetsPending = true;
    Serial.println(F("READY zero=0"));
}

void nanogreatLoop() {
    const uint32_t now = millis();

    serviceHealthReply();
    bootMotion.update(now);
    serviceSerial(now);
    bootMotion.update(now);
    enforceBootTimeoutStop();

    if (driversEnabled && digitalRead(PIN_ENABLE) != LOW) {
        freezeAllAxes();
        driversEnabled = false;
        if (healthReplyPending()) {
            driverStoppedReplyDeferred = true;
        } else {
            Serial.println(F("ERR driver stopped"));
        }
    }

    if (!driversEnabled) {
        serviceHealthReply();
        serviceDeferredReplies();
        return;
    }

    servicePanTilt();
    serviceSlider();
    serviceHealthReply();
    serviceDeferredReplies();
}
