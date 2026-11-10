#pragma once
#include <Arduino.h>

struct Jy901sSample {
    float roll = 0, pitch = 0, yaw = 0;
    float rollSpeed = 0, pitchSpeed = 0, yawSpeed = 0;
    float zAcceleration = 0;
    // Array order is acceleration, angular velocity, Euler angles.
    uint32_t receivedAt[3] = {};
    uint32_t frames[3] = {};
    float rates[3] = {};
    uint32_t bytes = 0, checksumErrors = 0, baud = 0;
    bool configuring = false;
};

class Jy901s {
public:
    void begin(int rxPin, int txPin);
    void poll();
    Jy901sSample snapshot();
    void requestConfiguration();
private:
    void startBaud(size_t index);
    void receiveByte(uint8_t value);
    void decodeFrame();
    void publish();
    void writeRegister(uint8_t address, uint16_t value);
    void configure();
    HardwareSerial serial_{1};
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    Jy901sSample published_, sample_;
    float raw_[3][3] = {};
    uint8_t frame_[11] = {};
    size_t frameSize_ = 0, baudIndex_ = 0;
    int rxPin_ = -1, txPin_ = -1;
    uint32_t baudStartedAt_ = 0, lastFrameAt_ = 0, rateStartedAt_ = 0;
    uint32_t rateFrames_[3] = {};
    float lastYaw_ = 0, continuousYaw_ = 0;
    bool yawInitialized_ = false, configurationRequested_ = false;
};
