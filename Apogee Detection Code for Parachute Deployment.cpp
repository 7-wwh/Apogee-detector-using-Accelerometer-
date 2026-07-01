#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h> // Include the Servo library

// Initialize ADXL345 accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// Initialize Servo motor
#define SERVO_PIN 3 // Define the pin for the servo motor
#define SERVO_PWR_PIN 2 // NEW: Define the power control pin for the servo motor
Servo servoMotor;

// Flight detection parameters
#define LAUNCH_THRESHOLD 50.0
#define LAUNCH_DELTA_ACCEL_THRESHOLD 20.0 // The change in acceleration required to confirm launch
#define APOGEE_NEG_THRESHOLD -10.0
#define APOGEE_NEG_COUNT_REQUIRED 3
#define MIN_APOGEE_DELAY 8888 // The minimum time in milliseconds after launch before apogee can be detected
#define MAX_APOGEE_DELAY 15000
#define VELOCITY_THRESHOLD 2.0
#define SAMPLE_RATE 400
#define MIN_LOG_VELOCITY_THRESHOLD 2.0 // Minimum velocity for a flight to be considered valid for logging

// Burnout detection parameters
#define BURNOUT_DELTA_ACCEL_THRESHOLD 1.0 // The change in acceleration required to detect burnout

// Servo deployment angle
#define DEPLOYMENT_ANGLE 120 // New variable for the servo deployment angle

// Data structure for instantaneous logging
struct LogData {
  float time;
  // CHANGED: We are now logging acceleration instead of velocity
  float accel;
  float altitude;
};

// Combined data structure for one flight record
struct FlightRecord {
  uint32_t signature;
  float apogee;
  float maxVel;
  float timeToApogee;
  LogData firstThree[3];
  LogData lastThree[3];
  // New fields for burnout data
  LogData burnoutFront[2];
  LogData burnoutBack[2];
};

// EEPROM layout
#define EEPROM_ADDR 0
#define EEPROM_SIGNATURE 0xDEADBEEF
#define FLIGHT_RECORD_SIZE sizeof(FlightRecord)
#define MAX_FLIGHTS 10 // Now we can store 10 complete flight records
#define INDEX_ADDR (EEPROM_ADDR + (FLIGHT_RECORD_SIZE * MAX_FLIGHTS))

// In-RAM buffer to store data points during flight
#define MAX_LOG_ENTRIES 50
LogData logBuffer[MAX_LOG_ENTRIES];
int logIndex = 0;
int burnoutLogIndex = -1; // New global variable to store the log index at burnout

// Calibration offsets and vertical axis tracking
float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;
char verticalAxis = 'Y'; // 'X', 'Y', or 'Z'

// Flight state variables
bool launchDetected = false;
bool burnoutDetected = false;
bool apogeeDetected = false;
unsigned long launchTime = 0;
unsigned long preciseApogeeTime = 0;
int negCount = 0;

float maxVelocity = 0.0;
float currentVelocity = 0.0;
float maxAltitude = 0.0;
float currentAltitude = 0.0;
unsigned long lastUpdateTime = 0;
unsigned long lastPrintTime = 0;

// Enhanced velocity tracking for apogee time calculation
#define VELOCITY_HISTORY_SIZE 20
float velocityHistory[VELOCITY_HISTORY_SIZE];
unsigned long velocityTimeHistory[VELOCITY_HISTORY_SIZE];
int velocityHistoryIndex = 0;
int velocityHistoryCount = 0;

// Moving average filter
const int FILTER_SIZE = 10; // Increased to 10 for better noise reduction
float accel_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;
float lastSmoothedAccel = 0.0; // New variable for burnout detection

bool flightComplete = false;
unsigned long lastEmergencySignalTime = 0;

// ---- NeoPixel and Buzzer setup ----
#define LED_PIN 6
#define LED_PWR_PIN 4 // CHANGED: Now using pin 4 to control power to the NeoPixels
#define NUM_LEDS 8
#define BUZZER_PIN 5
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setAllLED(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) pixels.setPixelColor(i, color);
  pixels.show();
}

// Generates a short, clean tone with synchronized blue LED flashes to indicate data saved to EEPROM
void playDataSavedSignal() {
  pixels.setBrightness(255); // Make the flash super bright

  // Flash and beep for the first pulse
  setAllLED(pixels.Color(0, 0, 255)); // Blue for "data saved"
  tone(BUZZER_PIN, 1500, 150);
  delay(150);
  setAllLED(0);
  noTone(BUZZER_PIN);
  delay(50);

  // Flash and beep for the second pulse
  setAllLED(pixels.Color(0, 0, 255));
  tone(BUZZER_PIN, 2000, 150);
  delay(150);
  setAllLED(0);
  noTone(BUZZER_PIN);
  delay(50);

  // Flash and beep for the third pulse
  setAllLED(pixels.Color(0, 0, 255));
  tone(BUZZER_PIN, 2500, 150);
  delay(150);
  setAllLED(0);
  noTone(BUZZER_PIN);

  pixels.setBrightness(50); // Reset to default brightness
  pixels.show();
}

void ledBlinkPattern(uint32_t color, int times, int duration) {
  for (int i = 0; i < times; i++) {
    pixels.setBrightness(255); // Max brightness for prominent flash
    setAllLED(color);
    // NEW: Sweeping up and down in pitch for a more noticeable pattern
    for (int freq = 2000; freq <= 4000; freq += 200) {
      tone(BUZZER_PIN, freq, 5);
      delay(5);
    }
    pixels.setBrightness(50); // Reset to default brightness
    setAllLED(0);
    noTone(BUZZER_PIN);
    delay(duration);
  }
}

// New function for launch signal
void playLaunchSignal() {
  pixels.setBrightness(255);
  for (int i = 0; i < 3; i++) {
    setAllLED(pixels.Color(0, 255, 0)); // Green flash
    tone(BUZZER_PIN, 2000, 75);
    delay(100);
    setAllLED(0);
    noTone(BUZZER_PIN);
    delay(50);
  }
  pixels.setBrightness(50);
}

// Moving average filter
float smoothAcceleration(float newAccel) {
  accel_buffer[buffer_index] = newAccel;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;
  float sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) sum += accel_buffer[i];
  return sum / FILTER_SIZE;
}

// Update velocity history for apogee time calculation
void updateVelocityHistory(float velocity, unsigned long timestamp) {
  velocityHistory[velocityHistoryIndex] = velocity;
  velocityTimeHistory[velocityHistoryIndex] = timestamp;
  velocityHistoryIndex = (velocityHistoryIndex + 1) % VELOCITY_HISTORY_SIZE;
  if (velocityHistoryCount < VELOCITY_HISTORY_SIZE) velocityHistoryCount++;
}

// Calculate precise apogee timing using linear interpolation
float calculatePreciseApogeeTime(unsigned long detectionTime) {
  // Method 1: Basic detection time
  float basicTime = (detectionTime - launchTime) / 1000.0;

  // Method 2: Linear interpolation to find the zero-crossing of velocity
  float method2 = basicTime;
  if (velocityHistoryCount >= 3) {
    float prev_vel = 0.0;
    unsigned long prev_time = 0;
    float current_vel = 0.0;
    unsigned long current_time = 0;

    // Find the last positive velocity and the first negative velocity
    // by searching backwards from the apogee detection time
    for (int i = 0; i < velocityHistoryCount; i++) {
      int idx = (velocityHistoryIndex - 1 - i + VELOCITY_HISTORY_SIZE) % VELOCITY_HISTORY_SIZE;
      if (velocityHistory[idx] >= 0) {
        prev_vel = velocityHistory[idx];
        prev_time = velocityTimeHistory[idx];
        current_vel = velocityHistory[(idx + 1) % VELOCITY_HISTORY_SIZE];
        current_time = velocityTimeHistory[(idx + 1) % VELOCITY_HISTORY_SIZE];
        break; // Found the transition point
      }
    }

    if (current_vel < 0 && prev_vel >= 0) {
      // Linear interpolation formula:
      // time_at_zero = t_prev + (t_curr - t_prev) * abs(v_prev) / (abs(v_prev) + abs(v_curr))
      float time_diff = (float)(current_time - prev_time) / 1000.0;
      float velocity_diff = abs(prev_vel) + abs(current_vel);
      if (velocity_diff > 0.0) {
        float interpolation_fraction = abs(prev_vel) / velocity_diff;
        method2 = ((prev_time + (time_diff * 1000 * interpolation_fraction)) - launchTime) / 1000.0;
      }
    }
  }

  // Weighted average of methods
  float weightedTime = (basicTime * 0.4) + (method2 * 0.6);

  return weightedTime;
}

// Calibration (returns true if successful)
bool calibrateAccelerometer() {
  Serial.println(F("Calibrating accelerometer... Assuming upright position for 5s."));
  tone(BUZZER_PIN, 800, 500);
  float sum_x = 0, sum_y = 0, sum_z = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    if (isnan(event.acceleration.x) || isnan(event.acceleration.y) || isnan(event.acceleration.z)) {
      return false;
    }
    sum_x += event.acceleration.x;
    sum_y += event.acceleration.y;
    sum_z += event.acceleration.z;
    delay(10);
  }

  accel_offset_x = sum_x / samples;
  accel_offset_y = sum_y / samples;
  accel_offset_z = sum_z / samples;

  // Find the axis that is aligned with gravity
  float abs_x = abs(accel_offset_x);
  float abs_y = abs(accel_offset_y);
  float abs_z = abs(accel_offset_z);

  if (abs_x > abs_y && abs_x > abs_z) {
    verticalAxis = 'X';
  } else if (abs_y > abs_x && abs_y > abs_z) {
    verticalAxis = 'Y';
  } else { // 'Z' axis
    verticalAxis = 'Z';
  }

  Serial.print(F("Calibration successful. Vertical axis detected as: "));
  Serial.println(verticalAxis);
  tone(BUZZER_PIN, 1200, 500);
  return true;
}

// LED signals
void playStartupSignal() {
  pixels.setBrightness(255);
  // NEW: A short chirp for startup confirmation
  tone(BUZZER_PIN, 4000, 50);
  setAllLED(pixels.Color(0, 255, 0));
  delay(100);
  noTone(BUZZER_PIN);
  setAllLED(0);
  delay(100);

  tone(BUZZER_PIN, 4000, 50);
  setAllLED(pixels.Color(0, 255, 0));
  delay(100);
  noTone(BUZZER_PIN);
  setAllLED(0);
  delay(100);

  tone(BUZZER_PIN, 4000, 50);
  setAllLED(pixels.Color(0, 255, 0));
  delay(100);
  noTone(BUZZER_PIN);
  setAllLED(0);

  pixels.setBrightness(50);
}

// Plays a clean, distinct sound for apogee detected
void playApogeeSignal() {
  pixels.setBrightness(255);
  setAllLED(pixels.Color(255, 255, 0)); // Yellow for apogee
  tone(BUZZER_PIN, 2000, 100);
  delay(120);
  setAllLED(0);
  noTone(BUZZER_PIN);

  setAllLED(pixels.Color(255, 255, 0));
  tone(BUZZER_PIN, 3000, 100);
  delay(120);
  setAllLED(0);
  noTone(BUZZER_PIN);

  setAllLED(pixels.Color(255, 255, 0));
  tone(BUZZER_PIN, 2000, 100);
  delay(120);
  setAllLED(0);
  noTone(BUZZER_PIN);

  pixels.setBrightness(50);
}

// New warning signal for low-velocity apogee detection
void playWarningSignal() {
  pixels.setBrightness(255);
  for (int i = 0; i < 3; i++) {
    setAllLED(pixels.Color(255, 0, 255)); // Magenta flash
    tone(BUZZER_PIN, 1500, 75);
    delay(100);
    setAllLED(0);
    noTone(BUZZER_PIN);
    delay(100);
  }
  pixels.setBrightness(50);
}

void playShutdownSignal() {
  pixels.setBrightness(255);
  for (int i = 0; i < 3; i++) {
    setAllLED(pixels.Color(255, 0, 0)); // Red flash
    tone(BUZZER_PIN, 500, 100);
    delay(150);
    setAllLED(0);
    noTone(BUZZER_PIN);
    delay(100);
  }
  pixels.setBrightness(50);
}

// Read and print all stored flight records
void readAllFlightRecords() {
  Serial.println(F("\n--- Previous Flight Records ---"));
  byte flightIndex;
  EEPROM.get(INDEX_ADDR, flightIndex);

  for (int i = 0; i < flightIndex; i++) {
    FlightRecord fr;
    EEPROM.get(EEPROM_ADDR + i * FLIGHT_RECORD_SIZE, fr);
    if (fr.signature == EEPROM_SIGNATURE) {
      Serial.print(F("--- Flight ")); Serial.print(i + 1); Serial.println(F(" ---"));
      Serial.print(F("Apogee: ")); Serial.print(fr.apogee, 2); Serial.println(F(" m"));
      Serial.print(F("Max Velocity: ")); Serial.print(fr.maxVel, 2); Serial.println(F(" m/s"));
      Serial.print(F("Time to Apogee: ")); Serial.print(fr.timeToApogee, 4); Serial.println(F(" s"));

      // UPDATED: Printing acceleration instead of velocity
      Serial.println(F("\nInstantaneous Log Summary:"));
      Serial.println(F("Type\tTime (s)\tAcceleration (m/s^2)\tAltitude (m)"));

      // Print first three data points
      for (int j = 0; j < 3; j++) {
        if (fr.firstThree[j].time >= 0) {
          Serial.print(F("Start\t"));
          Serial.print(fr.firstThree[j].time, 2);
          Serial.print(F("\t\t"));
          Serial.print(fr.firstThree[j].accel, 2);
          Serial.print(F("\t\t\t"));
          Serial.println(fr.firstThree[j].altitude, 1);
        }
      }
      // Print last three data points
      for (int j = 0; j < 3; j++) {
        if (fr.lastThree[j].time >= 0) {
          Serial.print(F("End\t"));
          Serial.print(fr.lastThree[j].time, 2);
          Serial.print(F("\t\t"));
          Serial.print(fr.lastThree[j].accel, 2);
          Serial.print(F("\t\t\t"));
          Serial.println(fr.lastThree[j].altitude, 1);
        }
      }

      // Print burnout data
      if (fr.burnoutFront[0].time >= 0) {
        Serial.println(F("\nBurnout Data:"));
        Serial.println(F("Type\tTime (s)\tAcceleration (m/s^2)\tAltitude (m)"));
        Serial.print(F("Pre-1\t")); Serial.print(fr.burnoutFront[0].time, 2); Serial.print(F("\t\t"));
        Serial.print(fr.burnoutFront[0].accel, 2); Serial.print(F("\t\t\t"));
        Serial.println(fr.burnoutFront[0].altitude, 1);
        Serial.print(F("Pre-2\t")); Serial.print(fr.burnoutFront[1].time, 2); Serial.print(F("\t\t"));
        Serial.print(fr.burnoutFront[1].accel, 2); Serial.print(F("\t\t\t"));
        Serial.println(fr.burnoutFront[1].altitude, 1);
        Serial.print(F("Post-1\t")); Serial.print(fr.burnoutBack[0].time, 2); Serial.print(F("\t\t"));
        Serial.print(fr.burnoutBack[0].accel, 2); Serial.print(F("\t\t\t"));
        Serial.println(fr.burnoutBack[0].altitude, 1);
        Serial.print(F("Post-2\t")); Serial.print(fr.burnoutBack[1].time, 2); Serial.print(F("\t\t"));
        Serial.print(fr.burnoutBack[1].accel, 2); Serial.print(F("\t\t\t"));
        Serial.println(fr.burnoutBack[1].altitude, 1);
      }

      Serial.println(F("------------------------------------"));
    }
  }
}

// Write the latest flight data to EEPROM at the next available slot
void writeFlightRecord(float apogee, float maxVel, float timeToApogee) {
  byte index;
  EEPROM.get(INDEX_ADDR, index);
  if (index >= MAX_FLIGHTS) {
    index = 0; // Wrap around if full
  }

  // Play a clear signal and flash LEDs while writing to EEPROM
  playDataSavedSignal();

  FlightRecord fr;
  fr.signature = EEPROM_SIGNATURE;
  fr.apogee = apogee;
  fr.maxVel = maxVel;
  fr.timeToApogee = timeToApogee;

  // Copy the first 3 entries from the buffer
  for (int i = 0; i < 3; i++) {
    fr.firstThree[i] = logBuffer[i];
  }

  // Copy the last 3 entries from the buffer
  if (logIndex >= 6) {
    for (int i = 0; i < 3; i++) {
      fr.lastThree[i] = logBuffer[logIndex - 3 + i];
    }
  } else {
    // If not enough entries, fill with dummy data
    for (int i = 0; i < 3; i++) {
      fr.lastThree[i].time = -1.0;
    }
  }

  // Copy the burnout data
  if (burnoutLogIndex >= 2 && burnoutLogIndex + 2 < MAX_LOG_ENTRIES) {
    fr.burnoutFront[0] = logBuffer[burnoutLogIndex - 2];
    fr.burnoutFront[1] = logBuffer[burnoutLogIndex - 1];
    fr.burnoutBack[0] = logBuffer[burnoutLogIndex];
    fr.burnoutBack[1] = logBuffer[burnoutLogIndex + 1];
  } else {
    // Fill with dummy data if burnout data is not available
    fr.burnoutFront[0].time = -1.0;
    fr.burnoutBack[0].time = -1.0;
  }


  EEPROM.put(EEPROM_ADDR + index * FLIGHT_RECORD_SIZE, fr);

  index = (index + 1) % MAX_FLIGHTS;
  EEPROM.put(INDEX_ADDR, index);

  Serial.print(F("Flight record for flight "));
  Serial.print(index);
  Serial.println(F(" saved to EEPROM."));
}

// New function to clear all flight data from EEPROM
void clearEEPROMData() {
  Serial.println(F("Clearing all EEPROM flight data..."));
  byte index = 0;
  EEPROM.put(INDEX_ADDR, index);
  Serial.println(F("EEPROM cleared. Please reset the Arduino to see the changes."));
}

// Function to deploy the servo
void deployParachute() {
  Serial.println(F("Deploying parachute via servo..."));
  // NEW: Turn on the power pin for the servo motor
  digitalWrite(SERVO_PWR_PIN, HIGH);
  delay(50); // Wait for power to stabilize

  // Attach the servo and write the specific angle to deploy
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(DEPLOYMENT_ANGLE);

  delay(1000); // Wait for the servo to move

  // NEW: Detach the servo and turn off the power pin
  servoMotor.detach();
  digitalWrite(SERVO_PWR_PIN, LOW);
  Serial.println(F("Servo power shut down."));
}

// NEW: A centralized function to start the flight sequence.
void startFlight() {
  launchDetected = true;
  launchTime = millis();
  playLaunchSignal();
  Serial.println(F("\n--- LAUNCH DETECTED ---"));
  Serial.println(F("Time (s) | Acceleration (m/s^2) | Altitude (m)"));
  currentVelocity = 0.0; currentAltitude = 0.0;
  velocityHistoryIndex = 0;
  velocityHistoryCount = 0;
  logIndex = 0;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PWR_PIN, OUTPUT);
  digitalWrite(LED_PWR_PIN, HIGH);
  pinMode(BUZZER_PIN, OUTPUT);

  // NEW: Set up the power pin for the servo as an output and keep it low initially
  pinMode(SERVO_PWR_PIN, OUTPUT);
  digitalWrite(SERVO_PWR_PIN, LOW);

  delay(50);
  pixels.begin();
  pixels.setBrightness(50);
  // Using a short, clear signal for startup
  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  noTone(BUZZER_PIN);

  if (!accel.begin()) {
    Serial.println(F("ERROR: No ADXL345 detected"));
    setAllLED(pixels.Color(255, 0, 0));
    tone(BUZZER_PIN, 3500, 500);
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);

  if (!calibrateAccelerometer()) {
    Serial.println(F("ERROR: Calibration failed"));
    setAllLED(pixels.Color(255, 0, 0));
    tone(BUZZER_PIN, 3500, 500);
    while (1);
  }

  byte idx;
  EEPROM.get(INDEX_ADDR, idx);
  if (idx >= MAX_FLIGHTS) { idx = 0; EEPROM.put(INDEX_ADDR, idx); }

  for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++) {
    velocityHistory[i] = 0.0;
    velocityTimeHistory[i] = 0;
  }
  logIndex = 0;
  burnoutLogIndex = -1; // Reset burnout index on startup

  readAllFlightRecords();
  playStartupSignal();

  lastUpdateTime = millis();
  lastPrintTime = millis();
  Serial.println(F("\n--- Rocket Flight Monitor ---"));
  // CHANGED: Update serial header to reflect the new data being logged
  Serial.println(F("Ready for launch... Type 'launch' to begin, or wait for automatic detection."));
  Serial.println(F("Time (s) | Acceleration (m/s^2) | Altitude (m)"));
}

// NEW: Failsafe parameters
#define ENABLE_TIME_APOGEE_FAILSAFE // Comment this line to disable the failsafe
#define APOGEE_FAILSAFE_TIME 14000 // Time in milliseconds

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    if (command.equals("clear")) {
      clearEEPROMData();
      while(1);
    }
    // Manual launch trigger.
    else if (command.equals("launch") && !launchDetected) {
      startFlight();
    }
    // NEW CODE ADDED HERE:
    // Listen for the "180" command to manually turn the servo
    else if (command.equals("180")) {
      Serial.println("Moving servo to 180 degrees...");
      // Turn on power to the servo motor
      digitalWrite(SERVO_PWR_PIN, HIGH);
      delay(50); // Short delay to allow power to stabilize
      servoMotor.attach(SERVO_PIN);
      servoMotor.write(180);
      delay(1000); // Wait for the servo to move
      servoMotor.detach();
      digitalWrite(SERVO_PWR_PIN, LOW);
      Serial.println("Servo moved and power shut down.");
    }
  }

  sensors_event_t event;
  accel.getEvent(&event);

  // Correctly subtract the offset from the vertical axis
  float verticalAccel = 0.0;
  if (verticalAxis == 'X') {
    verticalAccel = event.acceleration.x - accel_offset_x;
  } else if (verticalAxis == 'Y') {
    verticalAccel = event.acceleration.y - accel_offset_y;
  } else { // 'Z' axis
    verticalAccel = event.acceleration.z - accel_offset_z;
  }

  float ax = event.acceleration.x - accel_offset_x;
  float ay = event.acceleration.y - accel_offset_y;
  float az = event.acceleration.z - accel_offset_z;

  float smoothed_a = smoothAcceleration(verticalAccel);
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  float deltaAccel = abs(smoothed_a - lastSmoothedAccel);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;
  lastSmoothedAccel = smoothed_a;

  // Reset velocity and altitude before launch is detected
  if (!launchDetected) {
    currentVelocity = 0.0;
    currentAltitude = 0.0;
    // Automatic accelerometer-based launch detection is back
    if (totalAccel > LAUNCH_THRESHOLD && deltaAccel > LAUNCH_DELTA_ACCEL_THRESHOLD) {
      startFlight();
    }
  }

  if (flightComplete) {
    unsigned long currentTime = millis();
    // The "data saved" signal now includes the flash
    if (currentTime - lastEmergencySignalTime >= 500) {
      lastEmergencySignalTime = currentTime;
      playDataSavedSignal();
    }
    return;
  }

  if (launchDetected && !apogeeDetected) {
    currentVelocity += smoothed_a * dt;
    currentAltitude += currentVelocity * dt;

    updateVelocityHistory(currentVelocity, currentTime);

    if (logIndex < MAX_LOG_ENTRIES) {
      logBuffer[logIndex].time = (currentTime - launchTime) / 1000.0;
      // CHANGED: Log acceleration instead of velocity
      logBuffer[logIndex].accel = smoothed_a;
      logBuffer[logIndex].altitude = currentAltitude;
      logIndex++;
    }

    if (currentVelocity > maxVelocity) maxVelocity = currentVelocity;
    if (currentAltitude > maxAltitude) maxAltitude = currentAltitude;

    // Detect motor burnout by looking for a sharp drop in acceleration.
    if (!burnoutDetected) {
      if ((lastSmoothedAccel - smoothed_a) > BURNOUT_DELTA_ACCEL_THRESHOLD) {
        burnoutDetected = true;
        burnoutLogIndex = logIndex; // Capture the log index at the moment of burnout
        Serial.println(F("--- BURNOUT DETECTED ---"));
      }
    }

    // Apogee detection logic
    // We'll check for a sustained period of negative acceleration, independent of burnout
    if (smoothed_a < APOGEE_NEG_THRESHOLD) {
      negCount++;
    } else if (smoothed_a > 0.0) { // Reset count only if accel becomes positive
      negCount = 0;
    }

    bool hasNegativeAccel = negCount >= APOGEE_NEG_COUNT_REQUIRED;
    bool hasNearZeroVelocity = abs(currentVelocity) < VELOCITY_THRESHOLD;
    bool hasAltitudeDecrease = currentAltitude < maxAltitude - 0.5;

    // *** MODIFICATION START ***
    // Check if the minimum time since launch has passed before checking for apogee.
    bool minTimePassed = (currentTime - launchTime) >= MIN_APOGEE_DELAY;

    // Apogee is detected if the apogee conditions are met AND the minimum delay has passed.
    if (minTimePassed && (hasNegativeAccel || (hasNearZeroVelocity && hasAltitudeDecrease))) {
    // *** MODIFICATION END ***
      apogeeDetected = true;
      preciseApogeeTime = currentTime;
      Serial.println(F("\n--- APOGEE DETECTED ---"));

      // Call the servo deployment function
      deployParachute();

      float apogeeEstimate = max(maxAltitude, currentAltitude);
      float preciseTimeToApogee = calculatePreciseApogeeTime(preciseApogeeTime);

      if (maxVelocity > MIN_LOG_VELOCITY_THRESHOLD) {
          writeFlightRecord(apogeeEstimate, maxVelocity, preciseTimeToApogee);
      } else {
          Serial.println(F("Apogee detected but flight velocity too low, not logging data."));
          // Flash a warning color if a low-velocity flight is detected
          playWarningSignal();
      }

      playApogeeSignal();

      Serial.print(F("Apogee: ")); Serial.print(apogeeEstimate, 2); Serial.println(F(" m"));
      Serial.print(F("Max Velocity: ")); Serial.print(maxVelocity, 2); Serial.println(F(" m/s"));
      Serial.print(F("Precise Time to Apogee: ")); Serial.print(preciseTimeToApogee, 4); Serial.println(F(" s"));
      Serial.println(F("--------------------------"));

      playShutdownSignal();
      flightComplete = true;
      lastEmergencySignalTime = millis();
      return;
    }

#ifdef ENABLE_TIME_APOGEE_FAILSAFE
    // NEW: Failsafe logic to deploy if apogee isn't detected in time
    if (currentTime - launchTime >= APOGEE_FAILSAFE_TIME) {
      apogeeDetected = true;
      Serial.println(F("\n--- APOGEE FAILSAFE TRIGGERED! ---"));
      deployParachute();
      playApogeeSignal();

      // We don't have a true apogee time, so we log the failsafe time
      float preciseTimeToApogee = (currentTime - launchTime) / 1000.0;
      writeFlightRecord(currentAltitude, maxVelocity, preciseTimeToApogee);

      Serial.print(F("Failsafe Apogee: ")); Serial.print(currentAltitude, 2); Serial.println(F(" m"));
      Serial.print(F("Failsafe Max Velocity: ")); Serial.print(maxVelocity, 2); Serial.println(F(" m/s"));
      Serial.print(F("Failsafe Time to Apogee: ")); Serial.print(preciseTimeToApogee, 4); Serial.println(F(" s"));
      Serial.println(F("--------------------------"));

      playShutdownSignal();
      flightComplete = true;
      lastEmergencySignalTime = millis();
      return;
    }
#endif

    if (currentTime - lastPrintTime >= 100) {
      lastPrintTime = currentTime;
      Serial.print((currentTime - launchTime) / 1000.0, 2); Serial.print(F("\t\t"));
      Serial.print(smoothed_a, 2); Serial.print(F("\t\t\t"));
      Serial.println(currentAltitude, 1);
    }
  }
}
