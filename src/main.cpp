#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "motion/AntennaController.h"

AntennaController antenna;

namespace {

enum class OperatingMode {
    IDLE,
    MANUAL,
    TRACK,
    HOME
};

enum class StreamChannel {
    AZEL,
    STATUS,
    SENSORS
};

struct Parameters {
    float maxSpeedDegPerSec = 60.0f;
    float maxAccelDegPerSec2 = 120.0f;
    float minElevDeg = -10.0f;
    float homeAzDeg = 0.0f;
    float homeElDeg = 0.0f;
    float streamAzelHz = 0.0f;
    float streamStatusHz = 0.0f;
};

struct TelemetryStream {
    uint16_t rateHz = 0;
    uint32_t lastEmitMs = 0;
};

struct FaultFlags {
    bool drvAz = false;
    bool drvEl = false;
    bool imu = false;
    bool gps = false;
    bool enc = false;
};

OperatingMode g_mode = OperatingMode::IDLE;
bool g_softEstop = false;
bool g_homeActive = false;
bool g_homeDoneSent = false;
FaultFlags g_faults{};
Parameters g_params{};
TelemetryStream g_streamAzel{};
TelemetryStream g_streamStatus{};
TelemetryStream g_streamSensors{};

float g_trackTargetLat = 0.0f;
float g_trackTargetLon = 0.0f;
float g_trackTargetAlt = 0.0f;
uint32_t g_trackLossTimeoutMs = 3000;
uint32_t g_trackLastTargetUpdateMs = 0;

float g_baseLat = 0.0f;
float g_baseLon = 0.0f;
float g_baseAlt = 0.0f;
uint8_t g_baseFix = 0;
float g_baseHdop = 99.9f;

float g_imuRoll = 0.0f;
float g_imuPitch = 0.0f;
float g_imuYaw = 0.0f;

char g_rxLine[256];
size_t g_rxLen = 0;

const char* modeToStr(OperatingMode mode) {
    switch (mode) {
        case OperatingMode::IDLE:
            return "IDLE";
        case OperatingMode::MANUAL:
            return "MANUAL";
        case OperatingMode::TRACK:
            return "TRACK";
        case OperatingMode::HOME:
            return "HOME";
        default:
            return "IDLE";
    }
}

bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return false;
    }

    while (*a && *b) {
        char ca = *a;
        char cb = *b;
        if (ca >= 'a' && ca <= 'z') {
            ca = static_cast<char>(ca - 'a' + 'A');
        }
        if (cb >= 'a' && cb <= 'z') {
            cb = static_cast<char>(cb - 'a' + 'A');
        }
        if (ca != cb) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

void replyOk(const char* payload = nullptr) {
    if (payload == nullptr || payload[0] == '\0') {
        Serial.println("OK");
        return;
    }

    Serial.print("OK ");
    Serial.println(payload);
}

void replyErr(const char* code, const char* reason) {
    Serial.print("ERR ");
    Serial.print(code);
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print(' ');
        Serial.print(reason);
    }
    Serial.println();
}

bool parseFloat(const char* text, float& outVal) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    outVal = strtof(text, &end);
    return end != text && *end == '\0';
}

bool parseUInt(const char* text, uint32_t& outVal) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    const unsigned long val = strtoul(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    outVal = static_cast<uint32_t>(val);
    return true;
}

bool parseInt(const char* text, int32_t& outVal) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    const long val = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    outVal = static_cast<int32_t>(val);
    return true;
}

bool hardwareEstopActive() {
    // TODO: Wire and read a hardware ESTOP GPIO here.
    return false;
}

bool hasFaults() {
    return g_faults.drvAz || g_faults.drvEl || g_faults.imu || g_faults.gps || g_faults.enc;
}

bool motionAllowed() {
    if (g_softEstop) {
        replyErr("estop_active", "motion blocked");
        return false;
    }
    if (hasFaults()) {
        replyErr("fault", "fault latched");
        return false;
    }
    return true;
}

bool getRateHz(StreamChannel channel, uint16_t& rateHzOut) {
    switch (channel) {
        case StreamChannel::AZEL:
            rateHzOut = g_streamAzel.rateHz;
            return true;
        case StreamChannel::STATUS:
            rateHzOut = g_streamStatus.rateHz;
            return true;
        case StreamChannel::SENSORS:
            rateHzOut = g_streamSensors.rateHz;
            return true;
        default:
            return false;
    }
}

TelemetryStream* getStream(StreamChannel channel) {
    switch (channel) {
        case StreamChannel::AZEL:
            return &g_streamAzel;
        case StreamChannel::STATUS:
            return &g_streamStatus;
        case StreamChannel::SENSORS:
            return &g_streamSensors;
        default:
            return nullptr;
    }
}

bool parseAxisName(const char* axisToken, bool& useAz, bool& useEl) {
    if (equalsIgnoreCase(axisToken, "AZ")) {
        useAz = true;
        useEl = false;
        return true;
    }
    if (equalsIgnoreCase(axisToken, "EL")) {
        useAz = false;
        useEl = true;
        return true;
    }
    if (equalsIgnoreCase(axisToken, "BOTH")) {
        useAz = true;
        useEl = true;
        return true;
    }
    return false;
}

void emitTelemAzel() {
    Serial.print("TELEM AZEL az=");
    Serial.print(antenna.getAzimuthMeasured(), 3);
    Serial.print(" el=");
    Serial.print(antenna.getElevationMeasured(), 3);
    Serial.print(" cmd_az=");
    Serial.print(antenna.getAzimuthCommanded(), 3);
    Serial.print(" cmd_el=");
    Serial.print(antenna.getElevationCommanded(), 3);
    Serial.print(" mode=");
    Serial.println(modeToStr(g_mode));
}

void emitTelemStatus() {
    uint8_t faultsByte = 0;
    if (g_faults.drvAz) faultsByte |= 0x01;
    if (g_faults.drvEl) faultsByte |= 0x02;
    if (g_faults.imu) faultsByte |= 0x04;
    if (g_faults.gps) faultsByte |= 0x08;
    if (g_faults.enc) faultsByte |= 0x10;

    Serial.print("TELEM STATUS mode=");
    Serial.print(modeToStr(g_mode));
    Serial.print(" estop=");
    Serial.print(g_softEstop ? 1 : 0);
    Serial.print(" moving=");
    Serial.print(antenna.isAnyAxisMoving() ? 1 : 0);
    Serial.print(" faults=0x");
    if (faultsByte < 16) {
        Serial.print('0');
    }
    Serial.println(faultsByte, HEX);
}

void emitTelemSensors() {
    Serial.print("TELEM SENSORS roll=");
    Serial.print(g_imuRoll, 2);
    Serial.print(" pitch=");
    Serial.print(g_imuPitch, 2);
    Serial.print(" yaw=");
    Serial.print(g_imuYaw, 2);
    Serial.print(" lat=");
    Serial.print(g_baseLat, 6);
    Serial.print(" lon=");
    Serial.print(g_baseLon, 6);
    Serial.print(" alt=");
    Serial.print(g_baseAlt, 1);
    Serial.print(" fix=");
    Serial.print(g_baseFix);
    Serial.print(" hdop=");
    Serial.println(g_baseHdop, 1);
}

void emitTelemetryIfDue(TelemetryStream& stream, void (*emitFn)(), uint32_t nowMs) {
    if (stream.rateHz == 0) {
        return;
    }
    const uint32_t periodMs = 1000u / stream.rateHz;
    if (periodMs == 0) {
        emitFn();
        stream.lastEmitMs = nowMs;
        return;
    }

    if (nowMs - stream.lastEmitMs >= periodMs) {
        emitFn();
        stream.lastEmitMs = nowMs;
    }
}

enum class ParamSetResult {
    OK,
    BAD_VALUE,
    UNKNOWN_PARAM
};

ParamSetResult setParam(const char* name, const char* valueText) {
    float val = 0.0f;
    if (!parseFloat(valueText, val)) {
        return ParamSetResult::BAD_VALUE;
    }

    if (equalsIgnoreCase(name, "MAX_SPEED_DEG_S")) {
        g_params.maxSpeedDegPerSec = val;
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "MAX_ACCEL_DEG_S2")) {
        g_params.maxAccelDegPerSec2 = val;
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "MIN_ELEV_DEG")) {
        g_params.minElevDeg = val;
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "HOME_AZ_DEG")) {
        g_params.homeAzDeg = val;
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "HOME_EL_DEG")) {
        g_params.homeElDeg = val;
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "STREAM_AZEL_HZ")) {
        g_params.streamAzelHz = val;
        const int32_t rate = static_cast<int32_t>(val);
        g_streamAzel.rateHz = static_cast<uint16_t>(constrain(rate, 0, 100));
        return ParamSetResult::OK;
    }
    if (equalsIgnoreCase(name, "STREAM_STATUS_HZ")) {
        g_params.streamStatusHz = val;
        const int32_t rate = static_cast<int32_t>(val);
        g_streamStatus.rateHz = static_cast<uint16_t>(constrain(rate, 0, 100));
        return ParamSetResult::OK;
    }
    return ParamSetResult::UNKNOWN_PARAM;
}

bool getParam(const char* name, float& outVal) {
    if (equalsIgnoreCase(name, "MAX_SPEED_DEG_S")) {
        outVal = g_params.maxSpeedDegPerSec;
        return true;
    }
    if (equalsIgnoreCase(name, "MAX_ACCEL_DEG_S2")) {
        outVal = g_params.maxAccelDegPerSec2;
        return true;
    }
    if (equalsIgnoreCase(name, "MIN_ELEV_DEG")) {
        outVal = g_params.minElevDeg;
        return true;
    }
    if (equalsIgnoreCase(name, "HOME_AZ_DEG")) {
        outVal = g_params.homeAzDeg;
        return true;
    }
    if (equalsIgnoreCase(name, "HOME_EL_DEG")) {
        outVal = g_params.homeElDeg;
        return true;
    }
    if (equalsIgnoreCase(name, "STREAM_AZEL_HZ")) {
        outVal = g_params.streamAzelHz;
        return true;
    }
    if (equalsIgnoreCase(name, "STREAM_STATUS_HZ")) {
        outVal = g_params.streamStatusHz;
        return true;
    }
    return false;
}

void printParamLine(const char* name, float value) {
    Serial.print("PARAM name=");
    Serial.print(name);
    Serial.print(" value=");
    Serial.println(value, 3);
}

void handleCommand(char* line) {
    while (*line == ' ' || *line == '\t') {
        ++line;
    }

    if (*line == '\0' || *line == '#') {
        return;
    }

    char* argv[8] = {nullptr};
    int argc = 0;
    char* token = strtok(line, " \t");
    while (token != nullptr && argc < 8) {
        argv[argc++] = token;
        token = strtok(nullptr, " \t");
    }

    if (argc == 0) {
        return;
    }

    if (equalsIgnoreCase(argv[0], "PING")) {
        replyOk("PONG");
        return;
    }

    if (equalsIgnoreCase(argv[0], "VERSION")) {
        replyOk("fw=1.0.0 board=ANTPTR_S3");
        return;
    }

    if (equalsIgnoreCase(argv[0], "MODE")) {
        if (argc < 2) {
            replyErr("bad_args", "MODE requires GET or SET");
            return;
        }

        if (equalsIgnoreCase(argv[1], "GET")) {
            Serial.print("OK mode=");
            Serial.println(modeToStr(g_mode));
            return;
        }

        if (equalsIgnoreCase(argv[1], "SET")) {
            if (argc != 3) {
                replyErr("bad_args", "MODE SET requires one value");
                return;
            }

            if (g_softEstop) {
                replyErr("estop_active", "cannot change mode while estop");
                return;
            }
            if (hasFaults()) {
                replyErr("fault", "cannot change mode with fault latched");
                return;
            }

            if (equalsIgnoreCase(argv[2], "IDLE")) {
                g_mode = OperatingMode::IDLE;
                antenna.stopAll();
                replyOk();
                return;
            }
            if (equalsIgnoreCase(argv[2], "MANUAL")) {
                g_mode = OperatingMode::MANUAL;
                replyOk();
                return;
            }
            if (equalsIgnoreCase(argv[2], "TRACK")) {
                g_mode = OperatingMode::TRACK;
                replyOk();
                return;
            }
            if (equalsIgnoreCase(argv[2], "HOME")) {
                g_mode = OperatingMode::HOME;
                g_homeActive = true;
                g_homeDoneSent = false;
                antenna.homeAll();
                replyOk();
                return;
            }

            replyErr("bad_args", "invalid mode");
            return;
        }

        replyErr("bad_args", "MODE requires GET or SET");
        return;
    }

    if (equalsIgnoreCase(argv[0], "GOTO_AZEL")) {
        if (argc < 3 || argc > 4) {
            replyErr("bad_args", "usage: GOTO_AZEL <az> <el> [speed]");
            return;
        }
        if (g_mode != OperatingMode::MANUAL) {
            replyErr("bad_mode", "requires mode MANUAL");
            return;
        }
        if (!motionAllowed()) {
            return;
        }

        float az = 0.0f;
        float el = 0.0f;
        if (!parseFloat(argv[1], az) || !parseFloat(argv[2], el)) {
            replyErr("bad_args", "invalid az or el");
            return;
        }
        if (el < g_params.minElevDeg) {
            replyErr("bad_args", "elevation below configured minimum");
            return;
        }

        // TODO: Apply per-command speed override into axis profile if provided.
        antenna.pointTo(az, el);
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "JOG")) {
        if (argc < 3 || argc > 4) {
            replyErr("bad_args", "usage: JOG <AZ|EL|BOTH> <delta> [speed]");
            return;
        }
        if (g_mode != OperatingMode::MANUAL) {
            replyErr("bad_mode", "requires mode MANUAL");
            return;
        }
        if (!motionAllowed()) {
            return;
        }

        bool useAz = false;
        bool useEl = false;
        if (!parseAxisName(argv[1], useAz, useEl)) {
            replyErr("bad_args", "axis must be AZ EL or BOTH");
            return;
        }

        float delta = 0.0f;
        if (!parseFloat(argv[2], delta)) {
            replyErr("bad_args", "invalid delta");
            return;
        }

        // TODO: Apply optional speed argument when axis profile supports run-time override.
        if (useAz) {
            antenna.jogAzimuth(delta);
        }
        if (useEl) {
            antenna.jogElevation(delta);
        }
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "MOTOR_ENABLE")) {
        if (argc != 3) {
            replyErr("bad_args", "usage: MOTOR_ENABLE <AZ|EL|BOTH> <0|1>");
            return;
        }

        bool useAz = false;
        bool useEl = false;
        if (!parseAxisName(argv[1], useAz, useEl)) {
            replyErr("bad_args", "axis must be AZ EL or BOTH");
            return;
        }

        int32_t enableInt = 0;
        if (!parseInt(argv[2], enableInt) || (enableInt != 0 && enableInt != 1)) {
            replyErr("bad_args", "enable must be 0 or 1");
            return;
        }
        const bool enable = (enableInt == 1);

        if (useAz && useEl) {
            antenna.setMotorEnableBoth(enable);
        } else if (useAz) {
            antenna.setMotorEnableAzimuth(enable);
        } else {
            antenna.setMotorEnableElevation(enable);
        }
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "TRACK_TARGET_GPS")) {
        if (argc != 4) {
            replyErr("bad_args", "usage: TRACK_TARGET_GPS <lat> <lon> <alt>");
            return;
        }
        if (g_mode != OperatingMode::TRACK) {
            replyErr("bad_mode", "requires mode TRACK");
            return;
        }
        if (!motionAllowed()) {
            return;
        }

        if (!parseFloat(argv[1], g_trackTargetLat) ||
            !parseFloat(argv[2], g_trackTargetLon) ||
            !parseFloat(argv[3], g_trackTargetAlt)) {
            replyErr("bad_args", "invalid gps tuple");
            return;
        }

        g_trackLastTargetUpdateMs = millis();
        // TODO: Replace with actual TRACK solve using base GPS + BNO085 and pointTo(solutionAz, solutionEl).
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "TRACK_SET_LOSS_TIMEOUT")) {
        if (argc != 2) {
            replyErr("bad_args", "usage: TRACK_SET_LOSS_TIMEOUT <ms>");
            return;
        }

        uint32_t timeoutMs = 0;
        if (!parseUInt(argv[1], timeoutMs)) {
            replyErr("bad_args", "invalid timeout");
            return;
        }

        g_trackLossTimeoutMs = timeoutMs;
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "TRACK_PARK")) {
        if (g_mode != OperatingMode::TRACK) {
            replyErr("bad_mode", "requires mode TRACK");
            return;
        }
        if (!motionAllowed()) {
            return;
        }

        antenna.pointTo(0.0f, 0.0f);
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "GET_AZEL")) {
        Serial.print("OK az=");
        Serial.print(antenna.getAzimuthMeasured(), 3);
        Serial.print(" el=");
        Serial.println(antenna.getElevationMeasured(), 3);
        return;
    }

    if (equalsIgnoreCase(argv[0], "GET_ENCODERS")) {
        Serial.print("OK az_counts=");
        Serial.print(antenna.getAzimuthEncoderCounts());
        Serial.print(" el_counts=");
        Serial.println(antenna.getElevationEncoderCounts());
        return;
    }

    if (equalsIgnoreCase(argv[0], "GET_IMU")) {
        Serial.print("OK roll=");
        Serial.print(g_imuRoll, 2);
        Serial.print(" pitch=");
        Serial.print(g_imuPitch, 2);
        Serial.print(" yaw=");
        Serial.println(g_imuYaw, 2);
        return;
    }

    if (equalsIgnoreCase(argv[0], "GET_GPS_BASE")) {
        Serial.print("OK lat=");
        Serial.print(g_baseLat, 6);
        Serial.print(" lon=");
        Serial.print(g_baseLon, 6);
        Serial.print(" alt=");
        Serial.print(g_baseAlt, 1);
        Serial.print(" fix=");
        Serial.print(g_baseFix);
        Serial.print(" hdop=");
        Serial.println(g_baseHdop, 1);
        return;
    }

    if (equalsIgnoreCase(argv[0], "HOME_START")) {
        if (!motionAllowed()) {
            return;
        }
        g_mode = OperatingMode::HOME;
        g_homeActive = true;
        g_homeDoneSent = false;
        antenna.homeAll();
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "HOME_SET_AZ_ZERO")) {
        antenna.setAzimuthZeroNow();
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "HOME_SET_EL_ZERO")) {
        antenna.setElevationZeroNow();
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "ESTOP")) {
        if (g_softEstop) {
            replyErr("already_estop", "estop already active");
            return;
        }
        g_softEstop = true;
        antenna.stopAll();
        antenna.setMotorEnableBoth(false);
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "ESTOP_CLEAR")) {
        if (hardwareEstopActive()) {
            replyErr("estop_active_hw", "hardware estop is active");
            return;
        }
        if (hasFaults()) {
            replyErr("fault", "cannot clear with fault latched");
            return;
        }
        g_softEstop = false;
        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "FAULTS")) {
        Serial.print("OK estop=");
        Serial.print(g_softEstop ? 1 : 0);
        Serial.print(" drv_az=");
        Serial.print(g_faults.drvAz ? 1 : 0);
        Serial.print(" drv_el=");
        Serial.print(g_faults.drvEl ? 1 : 0);
        Serial.print(" imu=");
        Serial.print(g_faults.imu ? 1 : 0);
        Serial.print(" gps=");
        Serial.print(g_faults.gps ? 1 : 0);
        Serial.print(" enc=");
        Serial.println(g_faults.enc ? 1 : 0);
        return;
    }

    if (equalsIgnoreCase(argv[0], "PARAM_GET")) {
        if (argc != 2) {
            replyErr("bad_args", "usage: PARAM_GET <name>");
            return;
        }

        float value = 0.0f;
        if (!getParam(argv[1], value)) {
            replyErr("unknown_param", "parameter not recognized");
            return;
        }

        Serial.print("OK value=");
        Serial.println(value, 3);
        return;
    }

    if (equalsIgnoreCase(argv[0], "PARAM_SET")) {
        if (argc != 3) {
            replyErr("bad_args", "usage: PARAM_SET <name> <value>");
            return;
        }

        const ParamSetResult result = setParam(argv[1], argv[2]);
        if (result == ParamSetResult::BAD_VALUE) {
            replyErr("bad_args", "invalid parameter value");
            return;
        }
        if (result == ParamSetResult::UNKNOWN_PARAM) {
            replyErr("unknown_param", "parameter not recognized");
            return;
        }

        replyOk();
        return;
    }

    if (equalsIgnoreCase(argv[0], "PARAM_LIST")) {
        Serial.println("OK BEGIN");
        printParamLine("MAX_SPEED_DEG_S", g_params.maxSpeedDegPerSec);
        printParamLine("MAX_ACCEL_DEG_S2", g_params.maxAccelDegPerSec2);
        printParamLine("MIN_ELEV_DEG", g_params.minElevDeg);
        printParamLine("HOME_AZ_DEG", g_params.homeAzDeg);
        printParamLine("HOME_EL_DEG", g_params.homeElDeg);
        printParamLine("STREAM_AZEL_HZ", g_params.streamAzelHz);
        printParamLine("STREAM_STATUS_HZ", g_params.streamStatusHz);
        Serial.println("OK END");
        return;
    }

    if (equalsIgnoreCase(argv[0], "STREAM")) {
        if (argc != 3) {
            replyErr("bad_args", "usage: STREAM <AZEL|STATUS|SENSORS> <rate_hz>");
            return;
        }

        StreamChannel channel = StreamChannel::AZEL;
        if (equalsIgnoreCase(argv[1], "AZEL")) {
            channel = StreamChannel::AZEL;
        } else if (equalsIgnoreCase(argv[1], "STATUS")) {
            channel = StreamChannel::STATUS;
        } else if (equalsIgnoreCase(argv[1], "SENSORS")) {
            channel = StreamChannel::SENSORS;
        } else {
            replyErr("bad_args", "unknown stream channel");
            return;
        }

        int32_t rate = 0;
        if (!parseInt(argv[2], rate)) {
            replyErr("bad_args", "invalid stream rate");
            return;
        }
        if (rate < 0 || rate > 100) {
            replyErr("bad_args", "rate must be 0..100");
            return;
        }

        TelemetryStream* stream = getStream(channel);
        if (stream == nullptr) {
            replyErr("bad_args", "unknown stream channel");
            return;
        }

        stream->rateHz = static_cast<uint16_t>(rate);
        stream->lastEmitMs = millis();
        replyOk();
        return;
    }

    replyErr("unknown_cmd", "command not recognized");
}

void serviceSerialInput() {
    while (Serial.available() > 0) {
        const int raw = Serial.read();
        if (raw < 0) {
            return;
        }
        const char c = static_cast<char>(raw);

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            g_rxLine[g_rxLen] = '\0';
            handleCommand(g_rxLine);
            g_rxLen = 0;
            continue;
        }

        if (g_rxLen < sizeof(g_rxLine) - 1) {
            g_rxLine[g_rxLen++] = c;
        } else {
            g_rxLen = 0;
            replyErr("bad_args", "line too long");
        }
    }
}

void serviceHomeAndTrack() {
    const uint32_t nowMs = millis();

    if (g_homeActive) {
        const bool azDone = antenna.getAzimuthState() == AxisState::IDLE;
        const bool elDone = antenna.getElevationState() == AxisState::IDLE;
        const bool azErr = antenna.getAzimuthState() == AxisState::ERROR;
        const bool elErr = antenna.getElevationState() == AxisState::ERROR;

        if (azErr || elErr) {
            if (!g_homeDoneSent) {
                Serial.println("TELEM STATUS home=error reason=axis_error");
                g_homeDoneSent = true;
            }
            g_homeActive = false;
        } else if (azDone && elDone) {
            if (!g_homeDoneSent) {
                Serial.println("TELEM STATUS home=done");
                g_homeDoneSent = true;
            }
            g_homeActive = false;
            if (g_mode == OperatingMode::HOME) {
                g_mode = OperatingMode::IDLE;
            }
        }
    }

    if (g_mode == OperatingMode::TRACK && g_trackLossTimeoutMs > 0) {
        if (g_trackLastTargetUpdateMs > 0 &&
            nowMs - g_trackLastTargetUpdateMs > g_trackLossTimeoutMs) {
            antenna.pointTo(0.0f, 0.0f);
            g_mode = OperatingMode::IDLE;
            Serial.println("TELEM STATUS track=timeout action=park_idle");
            g_trackLastTargetUpdateMs = nowMs;
        }
    }
}

void serviceTelemetry() {
    const uint32_t nowMs = millis();
    emitTelemetryIfDue(g_streamAzel, emitTelemAzel, nowMs);
    emitTelemetryIfDue(g_streamStatus, emitTelemStatus, nowMs);
    emitTelemetryIfDue(g_streamSensors, emitTelemSensors, nowMs);
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Antenna controller starting...");

    antenna.begin();
    antenna.startTasks();
    g_mode = OperatingMode::IDLE;
}

void loop() {
    serviceSerialInput();
    serviceHomeAndTrack();
    serviceTelemetry();
    delay(2);
}