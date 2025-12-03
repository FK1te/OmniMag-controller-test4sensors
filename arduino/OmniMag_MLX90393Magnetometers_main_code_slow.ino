#include <Wire.h>
#include <Adafruit_MLX90393.h>

#define DEBUG_MODE false // true: display in loop values read, false: use communication protocol (answer only if receive "UP")
#define BAUD_RATE 115200 // bps
#define SERIAL_TIMEOUT 1 // sec
#define LOOP_SLEEP_DURATION 10 // msec

Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor3 = Adafruit_MLX90393


bool sensor1Available = false;
bool sensor2Available = false;

void configureSensor(Adafruit_MLX90393 &sensor, uint8_t address, bool &available) {
  if (!sensor.begin_I2C(address, &Wire)) {
    Serial.print("Sensor not found at address 0x");
    Serial.println(address, HEX);
    available = false;
  } else {
    sensor.setGain(MLX90393_GAIN_1X);
    sensor.setResolution(MLX90393_X, MLX90393_RES_17);
    sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
    sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
    sensor.setOversampling(MLX90393_OSR_3);
    sensor.setFilter(MLX90393_FILTER_5);
    available = true;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);

  configureSensor(sensor1, 0x0C, sensor1Available);
  configureSensor(sensor2, 0x0F, sensor2Available);
}

void send_back_hall_sensors_values() {
  float x1 = 0, y1 = 0, z1 = 0;
  float x2 = 0, y2 = 0, z2 = 0;

  bool ok1 = sensor1Available && sensor1.readData(&x1, &y1, &z1);
  bool ok2 = sensor2Available && sensor2.readData(&x2, &y2, &z2);

  Serial.print("{");

  if (ok1) {
    Serial.print("\"x1\":");
    Serial.print(x1, 2);
    Serial.print(",\"y1\":");
    Serial.print(y1, 2);
    Serial.print(",\"z1\":");
    Serial.print(z1, 2);
  } else if (sensor1Available) {
    Serial.print("\"x1\":null,\"y1\":null,\"z1\":null");
  }

  if (ok1 && ok2) Serial.print(",");
  if (ok2) {
    Serial.print("\"x2\":");
    Serial.print(x2, 2);
    Serial.print(",\"y2\":");
    Serial.print(y2, 2);
    Serial.print(",\"z2\":");
    Serial.print(z2, 2);
  } else if (sensor2Available) {
    if (ok1) Serial.print(",");
    Serial.print("\"x2\":null,\"y2\":null,\"z2\":null");
  }

  Serial.println("}");
}

void loop() {
  if (DEBUG_MODE) {
    send_back_hall_sensors_values();
  } else {
    static String inputBuffer;
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      if (inChar == '\n' || inChar == '\r') {
        if (inputBuffer.startsWith("UP")) {
          send_back_hall_sensors_values();
        } else {
          Serial.println("ERR");
        }
        inputBuffer = ""; // Clear for next message
      } else {
        inputBuffer += inChar;
      }
    }
  }

  delay(LOOP_SLEEP_DURATION);
}
