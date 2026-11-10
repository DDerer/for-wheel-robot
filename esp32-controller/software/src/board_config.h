#pragma once
#include <stdint.h>

namespace BoardConfig {
constexpr int kCanTx = 4;
constexpr int kCanRx = 5;
constexpr int kImuTx = 11;
constexpr int kImuRx = 12;
constexpr int kButtonCenter = 18;
constexpr int kButtonStop = 15;
constexpr int kBatteryAdc = -1; // Supplied PCB has no battery divider.
constexpr float kBatteryDividerRatio = 1.0f;
constexpr float kNominalVoltage = 12.0f;
constexpr bool kImuMountingConfirmed = false;
constexpr bool kMotorCalibrationConfirmed = false;

// v_body = matrix * v_sensor. Use an orthonormal rotation with determinant +1.
// Rows are robot X/Y/Z, columns are sensor X/Y/Z. Confirm after installation.
constexpr float kSensorToBody[3][3] = {
    {0.0f, 1.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
// Left front/rear/wheel, right front/rear/wheel.
constexpr uint8_t kMotorIds[6] = {1, 2, 3, 5, 6, 7};
constexpr uint32_t kFeedbackTimeoutMs = 100;
constexpr uint32_t kImuTimeoutMs = 50;
constexpr float kMinimumImuRateHz = 80.0f;
constexpr uint32_t kStandupTimeoutMs = 5000;
}
