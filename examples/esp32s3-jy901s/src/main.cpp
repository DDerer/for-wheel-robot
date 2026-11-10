#include <Arduino.h>
#include <math.h>
#include <string.h>

// JY901S TX -> GPIO12, JY901S RX -> GPIO11.
constexpr int IMU_RX_PIN = 12;
constexpr int IMU_TX_PIN = 11;
constexpr uint32_t BAUD_RATES[] = {
    9600, 115200, 57600, 38400, 19200, 230400, 4800, 2400, 460800, 921600
};
constexpr uint32_t SCAN_INTERVAL_MS = 6000;
constexpr uint32_t LINK_TIMEOUT_MS = 15000;
constexpr uint32_t PRINT_INTERVAL_MS = 500;

HardwareSerial imuSerial(1);

struct VectorSample {
    float x = NAN;
    float y = NAN;
    float z = NAN;
    uint32_t receivedAt = 0;
    uint32_t frames = 0;
};

VectorSample acceleration;
VectorSample angularVelocity;
VectorSample angles;
uint8_t frame[11];
size_t frameSize = 0;
size_t baudIndex = 0;
uint32_t baudStartedAt = 0;
uint32_t lastFrameAt = 0;
uint32_t lastPrintAt = 0;
uint32_t receivedBytes = 0;
uint32_t checksumErrors = 0;
bool connected = false;

int16_t readInt16(const uint8_t *data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8));
}

void startBaud(size_t index)
{
    imuSerial.end();
    baudIndex = index;
    imuSerial.begin(BAUD_RATES[index], SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
    frameSize = 0;
    acceleration = VectorSample{};
    angularVelocity = VectorSample{};
    angles = VectorSample{};
    connected = false;
    baudStartedAt = millis();
    Serial.printf("Searching JY901S at %lu baud (RX=%d, TX=%d)...\n",
                  static_cast<unsigned long>(BAUD_RATES[index]),
                  IMU_RX_PIN, IMU_TX_PIN);
}

void decodeFrame()
{
    VectorSample *sample;
    float scale;
    switch (frame[1]) {
        case 0x51:
            sample = &acceleration;
            scale = 16.0f / 32768.0f; // g, including gravity
            break;
        case 0x52:
            sample = &angularVelocity;
            scale = 2000.0f / 32768.0f; // degrees/s
            break;
        case 0x53:
            sample = &angles;
            scale = 180.0f / 32768.0f; // degrees, module roll/pitch/yaw
            break;
        default:
            return;
    }

    sample->x = readInt16(&frame[2]) * scale;
    sample->y = readInt16(&frame[4]) * scale;
    sample->z = readInt16(&frame[6]) * scale;
    sample->receivedAt = millis();
    sample->frames++;
    lastFrameAt = sample->receivedAt;
    if (!connected) {
        connected = true;
        Serial.printf("JY901S detected at %lu baud.\n",
                      static_cast<unsigned long>(BAUD_RATES[baudIndex]));
    }
}

void receiveByte(uint8_t value)
{
    receivedBytes++;
    if (frameSize == 0 && value != 0x55) {
        return;
    }
    frame[frameSize++] = value;
    if (frameSize < sizeof(frame)) {
        return;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(frame) - 1; i++) {
        checksum += frame[i];
    }
    if (checksum == frame[10]) {
        decodeFrame();
        frameSize = 0;
        return;
    }

    checksumErrors++;
    // Preserve a possible next header after a corrupt or truncated frame.
    size_t nextHeader = 1;
    while (nextHeader < sizeof(frame) && frame[nextHeader] != 0x55) {
        nextHeader++;
    }
    frameSize = sizeof(frame) - nextHeader;
    memmove(frame, frame + nextHeader, frameSize);
}

void printSample(const char *name, const VectorSample &sample, uint32_t now)
{
    if (sample.frames == 0) {
        Serial.printf("%s=waiting  ", name);
        return;
    }
    Serial.printf("%s=(%.3f,%.3f,%.3f) age=%lums frames=%lu  ",
                  name, sample.x, sample.y, sample.z,
                  static_cast<unsigned long>(now - sample.receivedAt),
                  static_cast<unsigned long>(sample.frames));
}

void setup()
{
    Serial.begin(115200);
    const uint32_t usbStartedAt = millis();
    while (!Serial && millis() - usbStartedAt < 3000) {
        delay(10);
    }
    Serial.println("\nESP32-S3 + JY901S UART test");
    Serial.println("ACC[g], GYRO[deg/s], RPY[deg]. Values use module axes.");
    imuSerial.setRxBufferSize(2048);
    startBaud(0);
}

void loop()
{
    while (imuSerial.available() > 0) {
        receiveByte(static_cast<uint8_t>(imuSerial.read()));
    }

    const uint32_t now = millis();
    if (connected && now - lastFrameAt >= LINK_TIMEOUT_MS) {
        Serial.println("IMU data timed out; restarting baud search.");
        startBaud(0);
    } else if (!connected && now - baudStartedAt >= SCAN_INTERVAL_MS) {
        startBaud((baudIndex + 1) % (sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0])));
    }

    if (now - lastPrintAt >= PRINT_INTERVAL_MS) {
        lastPrintAt = now;
        if (connected) {
            printSample("ACC", acceleration, now);
            printSample("GYRO", angularVelocity, now);
            printSample("RPY", angles, now);
        }
        Serial.printf("bytes=%lu checksum_errors=%lu\n",
                      static_cast<unsigned long>(receivedBytes),
                      static_cast<unsigned long>(checksumErrors));
    }
    delay(1);
}
