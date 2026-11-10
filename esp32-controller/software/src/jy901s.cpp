#include "jy901s.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t kBaudRates[] = {9600, 115200, 57600, 38400, 19200, 230400, 4800};
constexpr size_t kBaudCount = sizeof(kBaudRates) / sizeof(kBaudRates[0]);
int16_t readInt16(const uint8_t *data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                               (static_cast<uint16_t>(data[1]) << 8));
}
}

void Jy901s::begin(int rxPin, int txPin)
{
    rxPin_ = rxPin;
    txPin_ = txPin;
    serial_.setRxBufferSize(2048);
    startBaud(0);
}

void Jy901s::startBaud(size_t index)
{
    serial_.end();
    baudIndex_ = index;
    serial_.begin(kBaudRates[index], SERIAL_8N1, rxPin_, txPin_);
    frameSize_ = 0;
    sample_ = Jy901sSample{};
    sample_.baud = kBaudRates[index];
    memset(raw_, 0, sizeof(raw_));
    memset(rateFrames_, 0, sizeof(rateFrames_));
    yawInitialized_ = false;
    baudStartedAt_ = rateStartedAt_ = millis();
    lastFrameAt_ = 0;
    portENTER_CRITICAL(&mux_);
    published_ = sample_;
    portEXIT_CRITICAL(&mux_);
    Serial.printf("IMU searching at %lu baud\n", static_cast<unsigned long>(sample_.baud));
}

Jy901sSample Jy901s::snapshot()
{
    portENTER_CRITICAL(&mux_);
    const Jy901sSample result = published_;
    portEXIT_CRITICAL(&mux_);
    return result;
}

void Jy901s::requestConfiguration()
{
    portENTER_CRITICAL(&mux_);
    configurationRequested_ = true;
    published_.configuring = true;
    portEXIT_CRITICAL(&mux_);
}

void Jy901s::writeRegister(uint8_t address, uint16_t value)
{
    const uint8_t command[] = {0xFF, 0xAA, address,
                              static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
    serial_.write(command, sizeof(command));
    serial_.flush();
    delay(30);
}

void Jy901s::configure()
{
    if (sample_.frames[0] + sample_.frames[1] + sample_.frames[2] == 0) {
        Serial.println("IMU needs a detected sensor before configuration.");
        return;
    }
    Serial.println("IMU configuring 115200 baud, ACC/GYRO/ANGLE, 100 Hz...");
    // Switch baud before increasing the output rate.
    writeRegister(0x69, 0xB588);
    writeRegister(0x04, 0x0006);
    serial_.end();
    serial_.begin(115200, SERIAL_8N1, rxPin_, txPin_);
    writeRegister(0x69, 0xB588);
    writeRegister(0x02, 0x000E);
    writeRegister(0x03, 0x0009);
    writeRegister(0x00, 0x0000);
    startBaud(1);
    Serial.println("IMU commands sent; verify measured rates and power-cycle to check saving.");
}

void Jy901s::receiveByte(uint8_t value)
{
    sample_.bytes++;
    if (frameSize_ == 0 && value != 0x55) return;
    frame_[frameSize_++] = value;
    if (frameSize_ < sizeof(frame_)) return;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(frame_) - 1; i++) checksum += frame_[i];
    if (checksum == frame_[10]) {
        decodeFrame();
        frameSize_ = 0;
        return;
    }
    sample_.checksumErrors++;
    size_t nextHeader = 1;
    while (nextHeader < sizeof(frame_) && frame_[nextHeader] != 0x55) nextHeader++;
    frameSize_ = sizeof(frame_) - nextHeader;
    memmove(frame_, frame_ + nextHeader, frameSize_);
}

void Jy901s::decodeFrame()
{
    if (frame_[1] < 0x51 || frame_[1] > 0x53) return;
    const size_t type = frame_[1] - 0x51;
    constexpr float scales[] = {16.0f / 32768.0f,
                               2000.0f / 32768.0f * PI / 180.0f,
                               180.0f / 32768.0f * PI / 180.0f};
    for (size_t axis = 0; axis < 3; axis++) {
        raw_[type][axis] = readInt16(&frame_[2 + axis * 2]) * scales[type];
    }
    sample_.receivedAt[type] = millis();
    sample_.frames[type]++;
    lastFrameAt_ = sample_.receivedAt[type];
    publish();
}

void Jy901s::publish()
{
    const float cr = cosf(raw_[2][0]), sr = sinf(raw_[2][0]);
    const float cp = cosf(raw_[2][1]), sp = sinf(raw_[2][1]);
    const float cy = cosf(raw_[2][2]), sy = sinf(raw_[2][2]);
    const float worldFromSensor[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr}
    };
    float worldFromBody[3][3] = {};
    float acceleration[3] = {}, gyro[3] = {};
    for (size_t row = 0; row < 3; row++) {
        for (size_t column = 0; column < 3; column++) {
            acceleration[row] += BoardConfig::kSensorToBody[row][column] * raw_[0][column];
            gyro[row] += BoardConfig::kSensorToBody[row][column] * raw_[1][column];
            for (size_t axis = 0; axis < 3; axis++) {
                worldFromBody[row][column] += worldFromSensor[row][axis] *
                                             BoardConfig::kSensorToBody[column][axis];
            }
        }
    }
    sample_.roll = atan2f(worldFromBody[2][1], worldFromBody[2][2]);
    sample_.pitch = asinf(fmaxf(-1.0f, fminf(1.0f, -worldFromBody[2][0])));
    const float wrappedYaw = atan2f(worldFromBody[1][0], worldFromBody[0][0]);
    if (sample_.frames[2] > 0) {
        if (!yawInitialized_) {
            continuousYaw_ = wrappedYaw;
            yawInitialized_ = true;
        } else {
            continuousYaw_ += remainderf(wrappedYaw - lastYaw_, 2.0f * PI);
        }
        lastYaw_ = wrappedYaw;
    }
    sample_.yaw = continuousYaw_;
    sample_.rollSpeed = gyro[0];
    sample_.pitchSpeed = gyro[1];
    sample_.yawSpeed = gyro[2];
    sample_.zAcceleration = (acceleration[2] - worldFromBody[2][2]) * 9.80665f;
    portENTER_CRITICAL(&mux_);
    sample_.configuring = configurationRequested_;
    published_ = sample_;
    portEXIT_CRITICAL(&mux_);
}

void Jy901s::poll()
{
    portENTER_CRITICAL(&mux_);
    const bool configureNow = configurationRequested_;
    configurationRequested_ = false;
    portEXIT_CRITICAL(&mux_);
    if (configureNow) configure();
    while (serial_.available() > 0) receiveByte(static_cast<uint8_t>(serial_.read()));
    const uint32_t now = millis();
    const uint32_t count = sample_.frames[0] + sample_.frames[1] + sample_.frames[2];
    if ((count == 0 && now - baudStartedAt_ >= 6000) ||
        (count > 0 && now - lastFrameAt_ >= 15000)) {
        startBaud((baudIndex_ + 1) % kBaudCount);
        return;
    }
    const uint32_t elapsed = now - rateStartedAt_;
    if (elapsed >= 1000) {
        for (size_t type = 0; type < 3; type++) {
            sample_.rates[type] = (sample_.frames[type] - rateFrames_[type]) * 1000.0f / elapsed;
            rateFrames_[type] = sample_.frames[type];
        }
        rateStartedAt_ = now;
    }
    portENTER_CRITICAL(&mux_);
    sample_.configuring = configurationRequested_;
    published_ = sample_;
    portEXIT_CRITICAL(&mux_);
}
