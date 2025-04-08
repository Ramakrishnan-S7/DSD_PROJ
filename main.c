#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>

// LCD setup (address, columns, rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ADXL345 Register Addresses
#define ADXL345_ADDRESS 0x53
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define BW_RATE 0x2C

// Step counter parameters
#define WINDOW_SIZE 25
#define MIN_STEP_DISTANCE 0.04
#define MIN_STEP_TIME 300  // 0.3 seconds in milliseconds
#define VELOCITY_DECAY 0.95
#define STEP_LENGTH 0.762  // in meters (2.5 feet)
#define REQUIRED_SAMPLES 50  // Stabilization samples

// Arrays for storing acceleration data
float acc_x[WINDOW_SIZE];
float acc_y[WINDOW_SIZE];
float acc_z[WINDOW_SIZE];
int current_index = 0;

// Motion tracking variables
float vel_x = 0, vel_y = 0, vel_z = 0;
float pos_x = 0, pos_y = 0, pos_z = 0;
unsigned long last_step_time = 0;

// Step counting variables
int steps = 0;
float total_distance = 0.0;

// Stabilization flags
bool is_stabilized = false;
int stabilization_samples = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Step counter initializing...");
  
  // Initialize I2C communication
  Wire.begin(D2, D1);  // SDA = D2 (GPIO4), SCL = D1 (GPIO5)
  
  // Initialize ADXL345
  writeRegister(POWER_CTL, 0x08);     // Power on
  writeRegister(DATA_FORMAT, 0x0B);   // ±16g range, full resolution
  writeRegister(BW_RATE, 0x0A);       // 100Hz data rate
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
  
  // Initialize buffers
  fill_initial_buffers();
}

void loop() {
  if (!is_stabilized) {
    stabilization_samples++;
    if (stabilization_samples >= REQUIRED_SAMPLES) {
      is_stabilized = true;
      update_display();  // Show initial zeros
    }
  } else {
    update_step_counter();
  }
  
  delay(10);  // 10ms delay (100Hz)
}

void fill_initial_buffers() {
  // Fill buffers with initial readings to stabilize sensor
  for (int i = 0; i < WINDOW_SIZE; i++) {
    float x, y, z;
    read_accelerometer(&x, &y, &z);
    
    acc_x[i] = x;
    acc_y[i] = y;
    acc_z[i] = z;
    
    delay(10);
  }
}

void update_step_counter() {
  float x, y, z;
  read_accelerometer(&x, &y, &z);
  
  // Update circular buffers
  acc_x[current_index] = x;
  acc_y[current_index] = y;
  acc_z[current_index] = z;
  
  // Calculate average acceleration
  float avg_x = 0, avg_y = 0, avg_z = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    avg_x += acc_x[i];
    avg_y += acc_y[i];
    avg_z += acc_z[i];
  }
  avg_x /= WINDOW_SIZE;
  avg_y /= WINDOW_SIZE;
  avg_z /= WINDOW_SIZE;
  
  float dt = 0.01;  // 10ms
  
  // Update velocities with decay
  vel_x = (vel_x + (x - avg_x) * dt) * VELOCITY_DECAY;
  vel_y = (vel_y + (y - avg_y) * dt) * VELOCITY_DECAY;
  vel_z = (vel_z + (z - avg_z) * dt) * VELOCITY_DECAY;
  
  // Update positions
  pos_x += vel_x * dt;
  pos_y += vel_y * dt;
  pos_z += vel_z * dt;
  
  // Calculate displacement for step detection
  float displacement = sqrt(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z);
  unsigned long current_time = millis();
  
  // Detect step and update distance
  if (displacement > MIN_STEP_DISTANCE && 
      (current_time - last_step_time) > MIN_STEP_TIME) {
    steps++;
    total_distance += STEP_LENGTH;
    last_step_time = current_time;
    
    // Reset position tracking for next step detection
    pos_x = pos_y = pos_z = 0;
    
    // Update LCD display
    update_display();
    
    // Debug output
    Serial.print("Step detected! Count: ");
    Serial.print(steps);
    Serial.print(", Distance: ");
    Serial.print(total_distance * 3.28084);
    Serial.println(" ft");
  }
  
  current_index = (current_index + 1) % WINDOW_SIZE;
}

void update_display() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Steps: ");
  lcd.print(steps);
  
  lcd.setCursor(0, 1);
  float distance_ft = total_distance * 3.28084;  // Convert meters to feet
  lcd.print("Dist:");
  lcd.print(distance_ft, 1);  // 1 decimal place
  lcd.print("ft");
}

void read_accelerometer(float *x, float *y, float *z) {
  int16_t raw_x, raw_y, raw_z;
  
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDRESS, 6, true);
  
  raw_x = Wire.read() | (Wire.read() << 8);
  raw_y = Wire.read() | (Wire.read() << 8);
  raw_z = Wire.read() | (Wire.read() << 8);
  
  // Convert to m/s² (same scale as original code)
  *x = raw_x * 0.004 * 9.81;
  *y = raw_y * 0.004 * 9.81;
  *z = raw_z * 0.004 * 9.81;
}

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}
