// OmniMag Hall Sensor Reader
// This Arduino sketch reads magnetic field data from two MLX90393 sensors over I2C,
// and sends the data in binary format over Serial upon request or periodically in debug mode.

#include <Wire.h>
#include <Adafruit_MLX90393.h>

#define BAUD_RATE 1000000   // High-speed Serial communication
#define DEBUG_MODE false    // Set to true for continuous streaming

Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor3 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor4 = Adafruit_MLX90393();

bool sensor1Available = false;
bool sensor2Available = false;
bool sensor3Available = false;
bool sensor4Available = false;

// Initialize and configure MLX90393 sensor
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
  Wire.setClock(400000);    // Enables I2C Fast Mode (400 kHz)

  Serial.begin(BAUD_RATE);
  while (!Serial) delay(10);  // Wait for serial to initialize

  configureSensor(sensor1, 0x0C, sensor1Available);
  configureSensor(sensor2, 0x0D, sensor2Available);
  configureSensor(sensor3, 0x0E, sensor3Available);
  configureSensor(sensor4, 0x0F, sensor4Available);
}

// Read from both sensors and send raw binary float values with flags
void send_binary_data() {
  float x1 = 0, y1 = 0, z1 = 0;
  float x2 = 0, y2 = 0, z2 = 0;
  float x3 = 0, y3 = 0, z3 = 0;
  float x4 = 0, y4 = 0, z4 = 0;

  bool ok1 = sensor1Available && sensor1.readData(&x1, &y1, &z1);
  bool ok2 = sensor2Available && sensor2.readData(&x2, &y2, &z2);
  bool ok3 = sensor3Available && sensor3.readData(&x3, &y3, &z3);
  bool ok4 = sensor4Available && sensor4.readData(&x4, &y4, &z4);

  uint8_t flags = (ok1 ? 1 : 0) | (ok2 ? 2 : 0) | (ok3 ? 4 : 0) | (ok4 ? 8 : 0);
  Serial.write(flags);  // 1: sensor1 ok, 2: sensor2 ok, 3: sensor3 ok, 4: sensor4 ok

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
  if (ok4) {
    Serial.write((uint8_t*)&x4, sizeof(float));
    Serial.write((uint8_t*)&y4, sizeof(float));
    Serial.write((uint8_t*)&z4, sizeof(float));
  }
}


void send_readable_data() {
  float x1 = 0, y1 = 0, z1 = 0;
  float x2 = 0, y2 = 0, z2 = 0;
  float x3 = 0, y3 = 0, z3 = 0;
  float x4 = 0, y4 = 0, z4 = 0;

  bool ok1 = sensor1Available && sensor1.readData(&x1, &y1, &z1);
  bool ok2 = sensor2Available && sensor2.readData(&x2, &y2, &z2);
  bool ok3 = sensor3Available && sensor3.readData(&x3, &y3, &z3);
  bool ok4 = sensor4Available && sensor4.readData(&x4, &y4, &z4);

  uint8_t flags = (ok1 ? 1 : 0) | (ok2 ? 2 : 0) | (ok3 ? 4 : 0) | (ok4 ? 8 : 0);
  Serial.print("Flags: ");
  Serial.println(flags, BIN); // Print the flags in binary format for debugging

  if (ok1) {
    Serial.print("Sensor 1: ");
    Serial.print("X: "); Serial.print(x1); Serial.print("\t");
    Serial.print("Y: "); Serial.print(y1); Serial.print("\t");
    Serial.print("Z: "); Serial.println(z1);
  }
  if (ok2) {
    Serial.print("Sensor 2: ");
    Serial.print("X: "); Serial.print(x2); Serial.print("\t");
    Serial.print("Y: "); Serial.print(y2); Serial.print("\t");
    Serial.print("Z: "); Serial.println(z2);
  }
  if (ok3) {
    Serial.print("Sensor 3: ");
    Serial.print("X: "); Serial.print(x3); Serial.print("\t");
    Serial.print("Y: "); Serial.print(y3); Serial.print("\t");
    Serial.print("Z: "); Serial.println(z3);
  }
  if (ok4) {
    Serial.print("Sensor 4: ");
    Serial.print("X: "); Serial.print(x4); Serial.print("\t");
    Serial.print("Y: "); Serial.print(y4); Serial.print("\t");
    Serial.print("Z: "); Serial.println(z4);
  }
}




void loop() {
  if (DEBUG_MODE) {
    send_binary_data();
    delay(20);  // Roughly 50 Hz in debug mode
    return;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'U') send_binary_data();  // Triggered read when host sends 'U'
    if (c == 'P') send_readable_data(); 
  }
}
