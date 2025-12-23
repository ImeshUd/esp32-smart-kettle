// Smart Kettle IoT Project using ESP32, Firebase, DS18B20, YF-S201, Relays, and Solenoid Valve
// This code assumes you have the necessary libraries installed:
// - FirebaseESP32 (for Firebase connection)
// - OneWire and DallasTemperature (for DS18B20)
// - WiFi (built-in)

// Pin Definitions
#define RELAY_KETTLE 12      // Relay for turning kettle on/off          green
#define RELAY_SOLENOID 13    // Relay for solenoid valve                 yellow
#define FLOW_SENSOR_PIN 14   // YF-S201 flow sensor (interrupt pin)      orange
#define ONE_WIRE_BUS 4       // DS18B20 data pin                         white

// Firebase and WiFi Credentials (Replace with your own)
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define FIREBASE_HOST ""  // Your Firebase Realtime Database URL
#define FIREBASE_AUTH ""       // Your Firebase Auth Key (Note: This should be the Database Secret for Realtime Database)

// Libraries
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Firebase Objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// DS18B20 Setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Flow Sensor Variables
volatile int pulseCount = 0;
float flowRate = 0.0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;
unsigned long oldTime = 0;

// State Variables
bool powerOn = false;
int setWaterLevel = 0;  // In milliliters
float setTemperature = 0.0;   // In Celsius
bool fillingWater = false;
bool heating = false;

// Interrupt Service Routine for Flow Sensor
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  // Pin Modes
  pinMode(RELAY_KETTLE, OUTPUT);
  pinMode(RELAY_SOLENOID, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

  // Initialize Relays (OFF by default)
  digitalWrite(RELAY_KETTLE, LOW);
  digitalWrite(RELAY_SOLENOID, LOW);

  // Attach Interrupt for Flow Sensor
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // DS18B20 Setup
  sensors.begin();

  // WiFi Connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Firebase Configuration
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize Firebase Paths (Updated to match Flutter app paths)
  // Paths: /users/user1/powerOn (bool), /users/user1/waterLevel (int), /users/user1/temperature (float)
}

void loop() {
  // Read Firebase Commands (Updated paths)
  if (Firebase.getBool(firebaseData, "/users/user1/powerOn")) {
    powerOn = firebaseData.boolData();
    Serial.print("PowerOn: ");
    Serial.println(powerOn);
    if (powerOn) {
      // Start the process: First fill water
      fillingWater = true;
      digitalWrite(RELAY_SOLENOID, HIGH);  // Open solenoid valve
      Serial.println("Solenoid valve opened for filling");
    } else {
      // Turn off everything
      fillingWater = false;
      heating = false;
      digitalWrite(RELAY_SOLENOID, LOW);
      digitalWrite(RELAY_KETTLE, LOW);
      Serial.println("All relays turned off");
    }
  }

  if (Firebase.getInt(firebaseData, "/users/user1/waterLevel")) {
    setWaterLevel = firebaseData.intData();
    Serial.print("Set Water Level: ");
    Serial.println(setWaterLevel);
  }

  if (Firebase.getFloat(firebaseData, "/users/user1/temperature")) {
    setTemperature = firebaseData.floatData();
    Serial.print("Set Temperature: ");
    Serial.println(setTemperature);
  }

  // Flow Sensor Calculation (every second)
  if ((millis() - oldTime) > 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / 7.5;  // YF-S201 calibration
    oldTime = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

    Serial.print("Flow Rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min, Total: ");
    Serial.print(totalMilliLitres);
    Serial.println(" mL");

    // Check if water level reached
    if (fillingWater && totalMilliLitres >= setWaterLevel) {
      digitalWrite(RELAY_SOLENOID, LOW);  // Close solenoid valve
      fillingWater = false;
      heating = true;
      totalMilliLitres = 0;  // Reset for next cycle
      Serial.println("Water filling complete, starting heating");
    }
  }

  // Temperature Monitoring
  if (heating) {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    Serial.print("Temperature: ");
    Serial.println(tempC);

    if (tempC >= setTemperature) {
      digitalWrite(RELAY_KETTLE, LOW);  // Turn off heating
      heating = false;
      powerOn = false;  // Reset kettle state
      // Update Firebase to reflect off state
      Firebase.setBool(firebaseData, "/users/user1/powerOn", false);
      Serial.println("Heating complete, kettle turned off");
    } else {
      digitalWrite(RELAY_KETTLE, HIGH);  // Keep heating
      Serial.println("Heating in progress");
    }
  }

  delay(100);  // Small delay to prevent overwhelming the loop
}
