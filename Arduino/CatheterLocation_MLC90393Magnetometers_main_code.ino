#include <Wire.h>
#include <Adafruit_MLX90393.h>

#define BAUD_RATE 1000000  // Fast communication
#define DEBUG_MODE false

Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor3 = Adafruit_MLX90393();

bool sensor1Available = false;
bool sensor2Available = false;
bool sensor3Available = false;

void configureSensor(Adafruit_MLX90393 &sensor, uint8_t address, bool &available) {
  if (!sensor.begin_I2C(address, &Wire)) {
    available = false;
  } else {
    sensor.setGain(MLX90393_GAIN_1X);
    sensor.setResolution(MLX90393_X, MLX90393_RES_17);
    sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
    sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
    sensor.setOversampling(MLX90393_OSR_0);
    sensor.setFilter(MLX90393_FILTER_0);
    available = true;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);

  configureSensor(sensor1, 0x0C, sensor1Available);
  configureSensor(sensor2, 0x0D, sensor2Available);
  configureSensor(sensor3, 0x0E, sensor3Available);
}

void send_binary_data() {
  float x1 = 0, y1 = 0, z1 = 0;
  float x2 = 0, y2 = 0, z2 = 0;
  float x3 = 0, y3 = 0, z3 = 0;

  bool ok1 = sensor1Available && sensor1.readData(&x1, &y1, &z1);
  bool ok2 = sensor2Available && sensor2.readData(&x2, &y2, &z2);
  bool ok3 = sensor3Available && sensor3.readData(&x3, &y3, &z3);

  // 3-bit flags: bit0=sensor1, bit1=sensor2, bit2=sensor3
  uint8_t flags = (ok1 ? 1 : 0) | (ok2 ? 2 : 0) | (ok3 ? 4 : 0);
  Serial.write(flags);

  if (ok1) {
    Serial.write((uint8_t*)&x1, sizeof(float));
    Serial.write((uint8_t*)&y1, sizeof(float));
    Serial.write((uint8_t*)&z1, sizeof(float));
  }
  if (ok2) {
    Serial.write((uint8_t*)&x2, sizeof(float));
    Serial.write((uint8_t*)&y2, sizeof(float));
    Serial.write((uint8_t*)&z2, sizeof(float));
  }
  if (ok3) {
    Serial.write((uint8_t*)&x3, sizeof(float));
    Serial.write((uint8_t*)&y3, sizeof(float));
    Serial.write((uint8_t*)&z3, sizeof(float));
  }
}

void loop() {
  if (DEBUG_MODE) {
    send_binary_data();
    delay(20);
    return;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'U') {
      send_binary_data();
    }
  }
}
