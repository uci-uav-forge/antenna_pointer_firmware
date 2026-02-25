#include "motion/AntennaController.h"

#include <Wire.h>

#include "config/MotorConfig.h"
#include "config/Pins.h"

AntennaController::AntennaController()
    : _azDriver(AZ_TMC_SERIAL,
                AZIMUTH_AXIS_CONFIG.tmcAddress,
                AZIMUTH_AXIS_CONFIG.enablePin,
                AZIMUTH_AXIS_CONFIG.diagPin),
      _elDriver(AZ_TMC_SERIAL,
                ELEVATION_AXIS_CONFIG.tmcAddress,
                ELEVATION_AXIS_CONFIG.enablePin,
                ELEVATION_AXIS_CONFIG.diagPin),
            _azEncoder(AZ_AS5600_I2C, AS5600_ADDR_AZ),
            _elEncoder(AZ_AS5600_I2C, AS5600_ADDR_EL),
      _azAxis(AZIMUTH_AXIS_CONFIG, _azDriver, _azEncoder),
      _elAxis(ELEVATION_AXIS_CONFIG, _elDriver, _elEncoder) {}

void AntennaController::begin() {
        AZ_AS5600_I2C.begin(AZ_AS5600_SDA_PIN, AZ_AS5600_SCL_PIN);
    _azAxis.begin();
    _elAxis.begin();
}

void AntennaController::startTasks() {
    _azAxis.startTask("AxisAZ", 4096, 2, 1);
    _elAxis.startTask("AxisEL", 4096, 2, 1);
}

void AntennaController::homeAll() {
    homeAzimuth();
    homeElevation();
}

void AntennaController::homeAzimuth() {
    _azAxis.homeSensorless();
}

void AntennaController::homeElevation() {
    _elAxis.homeSensorless();
}

void AntennaController::pointTo(float azDeg, float elDeg) {
    _azAxis.moveToAngle(azDeg);
    _elAxis.moveToAngle(elDeg);
}

void AntennaController::pointAzimuth(float azDeg) {
    _azAxis.moveToAngle(azDeg);
}

void AntennaController::pointElevation(float elDeg) {
    _elAxis.moveToAngle(elDeg);
}

void AntennaController::jogAzimuth(float deltaDeg) {
    _azAxis.moveByAngle(deltaDeg);
}

void AntennaController::jogElevation(float deltaDeg) {
    _elAxis.moveByAngle(deltaDeg);
}

void AntennaController::stopAll() {
    _azAxis.stop();
    _elAxis.stop();
}

void AntennaController::setMotorEnableAzimuth(bool enable) {
    _azAxis.setMotorEnabled(enable);
}

void AntennaController::setMotorEnableElevation(bool enable) {
    _elAxis.setMotorEnabled(enable);
}

void AntennaController::setMotorEnableBoth(bool enable) {
    _azAxis.setMotorEnabled(enable);
    _elAxis.setMotorEnabled(enable);
}

void AntennaController::setAzimuthZeroNow() {
    _azAxis.setCurrentAngleAsZero();
}

void AntennaController::setElevationZeroNow() {
    _elAxis.setCurrentAngleAsZero();
}

float AntennaController::getAzimuthMeasured() const {
    return _azAxis.getMeasuredAngleDeg();
}

float AntennaController::getElevationMeasured() const {
    return _elAxis.getMeasuredAngleDeg();
}

float AntennaController::getAzimuthCommanded() const {
    return _azAxis.getCommandedAngleDeg();
}

float AntennaController::getElevationCommanded() const {
    return _elAxis.getCommandedAngleDeg();
}

uint16_t AntennaController::getAzimuthEncoderCounts() const {
    return _azAxis.getEncoderRawCounts();
}

uint16_t AntennaController::getElevationEncoderCounts() const {
    return _elAxis.getEncoderRawCounts();
}

bool AntennaController::isAnyAxisMoving() const {
    return _azAxis.isMoving() || _elAxis.isMoving();
}

AxisState AntennaController::getAzimuthState() const {
    return _azAxis.getState();
}

AxisState AntennaController::getElevationState() const {
    return _elAxis.getState();
}
