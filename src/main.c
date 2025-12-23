#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include "MAX30100_PulseOximeter.h"

// LCD setup (address, columns, rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define RED_LED_PIN 1
#define GREEN_LED_PIN 16
#define BUZZER_PIN 13

// ADXL345 Register Addresses
#define ADXL345_ADDRESS 0x53
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define WINDOW_SIZE 20
#define ACCELERATION_THRESHOLD 0.3  // Adjust based on sensitivity needed
#define STEP_LENGTH 0.762  // in meters (2.5 feet)
#define REQUIRED_SAMPLES 50  // Stabilization samples

// MAX30100 setup
#define REPORTING_PERIOD_MS 2500  // Update pulse readings every 2.5 seconds
PulseOximeter pox;
unsigned long lastBeatCheck = 0;
unsigned long lastValidReading = 0;
int lastBpm = 0;
int bpm = 0;
bool sensor_initialized = false;

// Arrays for storing acceleration data
float acc_x[WINDOW_SIZE];
float acc_y[WINDOW_SIZE];
float acc_z[WINDOW_SIZE];
float acc_magnitude[WINDOW_SIZE];

// Indices for circular buffers
int current_index = 0;

// Step counter variables
int steps = 0;
bool step_detected = false;
float last_peak = 0.0;
float total_distance = 0.0;
bool is_stabilized = false;
int stabilization_samples = 0;

// Callback for MAX30100 beat detection
void onBeatDetected() {
  Serial.println("♥ Beat detected!");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Step counter and heart rate monitor initializing...");

  // Initialize I2C communication
  Wire.begin(4, 5);  // SDA = D2 (GPIO4), SCL = D1 (GPIO5)

  // Initialize ADXL345
  writeRegister(POWER_CTL, 0x08);     // Power on
  writeRegister(DATA_FORMAT, 0x0B);    // Full resolution, +/-16g
  delay(100);  // Give accelerometer time to start

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");

  // Setup LEDs and buzzer
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize both LEDs to off
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  delay(500);  // Give devices time to initialize

  // Initialize MAX30100 with multiple attempts if needed
  initializeMax30100();
  
  // Initialize buffers
  fill_initial_buffers();
}

void initializeMax30100() {
  int attempts = 0;
  const int maxAttempts = 5;
  
  while (attempts < maxAttempts) {
    Serial.print("Initializing pulse oximeter (attempt ");
    Serial.print(attempts + 1);
    Serial.print("/");
    Serial.print(maxAttempts);
    Serial.print(")...");
    
    // Reset I2C bus before trying
    Wire.endTransmission(true);
    delay(100);
    
    if (pox.begin()) {
      Serial.println("SUCCESS");
      
      // Configure MAX30100 with appropriate settings
      // Use moderate LED current for better readings
      pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
      
      // Register the beat detection callback
      pox.setOnBeatDetectedCallback(onBeatDetected);
      
      sensor_initialized = true;
      break;
    } else {
      Serial.println("FAILED");
      attempts++;
      delay(1000);  // Wait before trying again
    }
  }
  
  if (!sensor_initialized) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HR sensor fail!");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring");
    
    // Continue anyway with just step counter
    delay(2000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting up...");
  }
}

void loop() {
  // Update MAX30100 readings continuously
  if (sensor_initialized) {
    pox.update();
    
    // Check and update BPM readings every 2.5 seconds
    if (millis() - lastBeatCheck >= REPORTING_PERIOD_MS) {
      float currentBpm = pox.getHeartRate();
      
      // Only update if we have a valid reading
      if (currentBpm > 0 && currentBpm < 220) {  // Basic sanity check
        bpm = currentBpm;
        Serial.print("Heart rate: ");
        Serial.print(bpm);
        Serial.print(" bpm / SpO2: ");
        Serial.print(pox.getSpO2());
        Serial.println("%");
        
        lastValidReading = millis();
        update_display();  // Update display when we have a new valid reading
      } else {
        Serial.println("Waiting for valid heart rate reading...");
        // If no valid reading for 15 seconds, try to reset
        if (millis() - lastValidReading > 15000) {
          Serial.println("No valid readings. Resetting sensor...");
          initializeMax30100();
          lastValidReading = millis();
        }
      }
      
      lastBeatCheck = millis();
    }
  }
  
  if (!is_stabilized) {
    stabilization_samples++;
    if (stabilization_samples >= REQUIRED_SAMPLES) {
      is_stabilized = true;
      update_display();  // Show initial zeros
      updateLEDStatus();
    }
  } 
  else {
    update_step_counter();
    // Only update display for step count changes, BPM updates handled separately
    static int last_steps = 0;
    if (steps != last_steps) {
      update_display();
      last_steps = steps;
    }
    updateLEDStatus();
  }

  delay(10);  // 10ms delay (100Hz)
}

void updateLEDStatus() {
  if (bpm > 170 && bpm < 220) {  // Add upper sanity check
    // High heart rate - activate red LED and buzzer
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    tone(BUZZER_PIN, 2000);
  } 
  else if (bpm > 40 && bpm <= 170) {  // Add lower sanity check
    // Normal heart rate - activate green LED only
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }
  // If BPM is outside valid range, don't change LED status
}

void fill_initial_buffers() {
  // Fill buffers with initial readings to stabilize sensor
  for (int i = 0; i < WINDOW_SIZE; i++) {
    readAccelerometer();
    delay(10);
  }
}

void update_step_counter() {
  // Read accelerometer data
  readAccelerometer();
  
  // Calculate magnitude of acceleration
  float magnitude = sqrt(acc_x[current_index] * acc_x[current_index] +
                         acc_y[current_index] * acc_y[current_index] +
                         acc_z[current_index] * acc_z[current_index]);
  
  acc_magnitude[current_index] = magnitude;
  
  // Step detection algorithm
  float avg_magnitude = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    avg_magnitude += acc_magnitude[i];
  }
  avg_magnitude /= WINDOW_SIZE;
  
  // Detect peaks
  if (magnitude > avg_magnitude + ACCELERATION_THRESHOLD && !step_detected && magnitude > last_peak) {
    last_peak = magnitude;
    step_detected = true;
  } else if (magnitude < avg_magnitude - ACCELERATION_THRESHOLD && step_detected) {
    steps++;
    total_distance = steps * STEP_LENGTH;
    step_detected = false;
    last_peak = 0.0;
  }
  
  // Move to next position in circular buffer
  current_index = (current_index + 1) % WINDOW_SIZE;
}

void readAccelerometer() {
  // Read data from ADXL345
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(DATAX0);
  Wire.endTransmission();
  
  Wire.requestFrom(ADXL345_ADDRESS, 6);
  int x = (Wire.read() | (Wire.read() << 8));
  int y = (Wire.read() | (Wire.read() << 8));
  int z = (Wire.read() | (Wire.read() << 8));
  
  // Convert raw values to g forces
  acc_x[current_index] = x * 0.004;  // Scale factor for +/-16g range
  acc_y[current_index] = y * 0.004;
  acc_z[current_index] = z * 0.004;
}

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void update_display() {
  // Clear the entire display first
  lcd.clear();
  
  // Display steps
  lcd.setCursor(0, 0);
  lcd.print("Steps:");
  lcd.print(steps);

  // Display BPM (only show if we have a valid reading)
  lcd.setCursor(9, 0);
  lcd.print("BPM:");
  if (bpm > 0 && bpm < 220) {
    lcd.print(bpm);
  } else {
    lcd.print("--");  // Show dashes when no valid reading
  }
  
  // Display distance without the 'z' character
  lcd.setCursor(0, 1);
  float distance_ft = total_distance * 3.28084;  // Convert meters to feet
  lcd.print("Dist:");
  // Use print with 1 decimal place to prevent weird characters
  lcd.print(distance_ft, 1);
  lcd.print("ft");
}
