#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

#define LEDR LED_RED
#define LEDG LED_GREEN
#define LEDB LED_BLUE

#define PRESSURE_SENSOR_VCC_PIN D1
#define PRESSURE_SENSOR_SAADC_CHANNEL 0

const unsigned long delayMs = 7;
const unsigned long rate = 1000 / delayMs;
const unsigned long stayAwakeTimeMs = 1000 * 60;

struct IMUDataPacket {
  int16_t accel[3];
  int16_t gyro[3];
  uint16_t pressure;
};

BLEService bleService("16E5E7DE-21A7-B2B6-3975-D2AA2F3DB097"); // Replace with your BLE service UUID
BLECharacteristic imuCharacteristic("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, sizeof(IMUDataPacket));

LSM6DS3 imu(I2C_MODE, 0x6A); // I2C device address 0x6A

void setupImu() {
  imu.settings.accelRange = 4; // Can be: 2, 4, 8, 16
  imu.settings.gyroRange = 500; // Can be: 125, 245, 500, 1000, 2000
  imu.settings.accelSampleRate = 416; // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  imu.settings.gyroSampleRate = 416; // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu.settings.accelBandWidth = 200;
  imu.settings.gyroBandWidth = 200;
  imu.begin();
}

void setupPressureSensor() {
  pinMode(PRESSURE_SENSOR_VCC_PIN, OUTPUT);
  digitalWrite(PRESSURE_SENSOR_VCC_PIN, HIGH);
}

int16_t readPressure() {
  // Dummy pressure value for now
  return 1000;
}

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Arduino");
  BLE.setAdvertisedService(bleService);

  bleService.addCharacteristic(imuCharacteristic);
  BLE.addService(bleService);

  BLE.advertise();
  Serial.println("BLE service and characteristic set up.");
}

void runBLE() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Connected to central");

    while (central.connected()) {
      IMUDataPacket packet;
      packet.accel[0] = imu.readRawAccelX();
      packet.accel[1] = imu.readRawAccelY();
      packet.accel[2] = imu.readRawAccelZ();
      packet.gyro[0] = imu.readRawGyroX();
      packet.gyro[1] = imu.readRawGyroY();
      packet.gyro[2] = imu.readRawGyroZ();
      packet.pressure = readPressure();

      imuCharacteristic.writeValue((byte*)&packet, sizeof(packet));

      delay(delayMs);
    }

    Serial.println("Disconnected from central");
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  setupImu();
  setupPressureSensor();
  setupBLE();
}

void loop() {
  runBLE();
}