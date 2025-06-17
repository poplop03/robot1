// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

MPU6050 mpu;

// packet structure for InvenSense teapot demo (28 bytes)
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();

    // Optional: set offsets if needed (use calibration script or defaults)
    // mpu.setXAccelOffset(...);
    // mpu.setYAccelOffset(...);
    // mpu.setZAccelOffset(...);
    // mpu.setXGyroOffset(...);
    // mpu.setYGyroOffset(...);
    // mpu.setZGyroOffset(...);

    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int16_t temperature = mpu.getTemperature();

    // Quaternion: not available in raw mode, fill with zeros (or optionally estimated)
    teapotPacket[2] = 0; teapotPacket[3] = 0;   // q0 (w)
    teapotPacket[4] = 0; teapotPacket[5] = 0;   // q1 (x)
    teapotPacket[6] = 0; teapotPacket[7] = 0;   // q2 (y)
    teapotPacket[8] = 0; teapotPacket[9] = 0;   // q3 (z)

    // Gyro (x, y, z) [big endian]
    teapotPacket[10] = gx >> 8;
    teapotPacket[11] = gx & 0xFF;
    teapotPacket[12] = gy >> 8;
    teapotPacket[13] = gy & 0xFF;
    teapotPacket[14] = gz >> 8;
    teapotPacket[15] = gz & 0xFF;

    // Accel (x, y, z) [big endian]
    teapotPacket[16] = ax >> 8;
    teapotPacket[17] = ax & 0xFF;
    teapotPacket[18] = ay >> 8;
    teapotPacket[19] = ay & 0xFF;
    teapotPacket[20] = az >> 8;
    teapotPacket[21] = az & 0xFF;

    // Temperature [big endian]
    teapotPacket[22] = temperature >> 8;
    teapotPacket[23] = temperature & 0xFF;

    // Counter (optional, 8-bit, rolls over at 255)
    static uint8_t packetCount = 0;
    teapotPacket[24] = packetCount++;
    teapotPacket[25] = 0x00; // reserved or zero

    // [26] = '\r', [27] = '\n' already set

    Serial.write(teapotPacket, 28);

    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    delay(10); // ~100Hz output, adjust as needed
}