#include <WiFiManager.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include "Orbitron_Medium_10.h"
#include "Orbitron_Medium_12.h"
#include "Orbitron_Medium_15.h"
#include "Orbitron_Medium_20.h"
#include "Orbitron_Medium_25.h"
#include "signal_indicator.h"
#include "battery_charge.h"
#include "battery_full.h"
#include "battery_high.h"
#include "battery_half.h"
#include "battery_low.h"
#include "battery_warn.h"
#include "battery_empty.h"
#include "heart.h"


// Firebase API key and database URL
#define API_KEY "AIzaSyCPPy1vwFNX4kEo9FyCHPUjV50hnRYJyMQ"
#define DATABASE_URL "https://summit-guardian-rtdb-dca45-default-rtdb.asia-southeast1.firebasedatabase.app"

#define BUTTON_PIN_1 0
#define BUTTON_PIN_2 35

// Define the ADC pin
#define ADC_PIN 35

// Define the voltage divider ratio
#define VOLTAGE_DIVIDER_RATIO 2

// Define the reference voltage
#define VREF 3.3

// Define the maximum and minimum battery voltages
#define MAX_BATTERY_VOLTAGE 4.2
#define MIN_BATTERY_VOLTAGE 3.2

#define MEASUREMENT_INTERVAL 10000 // Measure every 10 seconds
#define VOLTAGE_INCREASE_THRESHOLD 0.01 // Adjust as needed

static const int RXPin = 13, TXPin = 12;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

float lastVoltage = 0.0;
unsigned long lastMeasurementMillis = 0;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Firebase objects
FirebaseData fbdo;      // Firebase data object
FirebaseAuth auth;      // Firebase authentication object
FirebaseConfig config;  // Firebase configuration object

// Heart rate sensor object
MAX30105 particleSensor;

// Constants for heart rate calculation
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

long lowStartTime = 0; // Time when heart rate first went below 60
long highStartTime = 0; // Time when heart rate first went above 100

// Timer for sending data to Firebase
unsigned long sendDataPrevMillis = 0;

// Flag for successful signup
bool signupOK = false;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

TFT_eSprite sprSignalIndicator = TFT_eSprite(&tft);
TFT_eSprite sprBatteryIndicator = TFT_eSprite(&tft);
TFT_eSprite sprDistressSignal = TFT_eSprite(&tft);
TFT_eSprite sprTitle = TFT_eSprite(&tft);
TFT_eSprite sprLine = TFT_eSprite(&tft);
TFT_eSprite sprHeartRate = TFT_eSprite(&tft);
TFT_eSprite sprHeart = TFT_eSprite(&tft);
TFT_eSprite sprBodyTemperature = TFT_eSprite(&tft);
TFT_eSprite sprAmbientTemperature = TFT_eSprite(&tft);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup(){
  Serial.begin(115200);

  // Initialize heart rate sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  ss.begin(GPSBaud);

  tft.init();
  spr.createSprite(135, 240);
  sprBatteryIndicator.createSprite(67, 30);
  sprSignalIndicator.createSprite(67, 30);
  sprDistressSignal.createSprite(135, 10);
  sprTitle.createSprite(135, 60);
  sprLine.createSprite(1, 135);
  sprHeartRate.createSprite(66, 70);
  sprHeart.createSprite(66, 70);
  sprBodyTemperature.createSprite(66, 70);
  sprAmbientTemperature.createSprite(66, 70);
  tft.fillScreen(TFT_BLACK);
  sprSignalIndicator.setSwapBytes(true);
  sprBatteryIndicator.setSwapBytes(true);
  sprHeart.setSwapBytes(true);

  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();

  setupWiFi();

  initializeFirebase();

  sendLocationCoordinateToFirebase();

  Firebase.RTDB.setBool(&fbdo, "isClimberDanger", false);

  printTitle();
  drawLine();
  drawHeart();
}

void setupWiFi() {
  WiFiManager wm;

  wm.resetSettings();

  bool res;

  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(&Orbitron_Medium_10);
  spr.setTextColor(TFT_WHITE,TFT_BLACK);
  spr.drawString("Connecting to WiFi...", 0, 0);
  spr.pushSprite(0,0);

  res = wm.autoConnect("SummitGuardianAP","password");

  // Clear the message after WiFi is connected
  if(res) {
      spr.fillSprite(TFT_BLACK);
      spr.pushSprite(0,0);
  }
}

void initializeFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase signUp successful.");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void printTitle() {
  sprTitle.fillSprite(TFT_BLACK);

  sprTitle.setFreeFont(&Orbitron_Medium_20);
  sprTitle.setTextColor(TFT_CYAN,TFT_BLACK);
  sprTitle.drawString("SUMMIT", 20, 0);
  sprTitle.drawString("GUARDIAN", 3, 20);

  sprTitle.pushSprite(0,40);
}

void drawLine() {
  sprLine.fillSprite(TFT_CYAN);
  
  sprLine.pushSprite(68, 100);
}

void drawHeart() {
  sprHeart.fillSprite(TFT_BLACK);

  sprHeart.pushImage(0, 5, 48, 48, heart);

  sprHeart.pushSprite(0,100);
}

void loop(){
  sendDistressSignalAndLocation();

  long irValue = particleSensor.getIR();

  if (irValue > 7000) {
    displayHeartRateData(irValue);

    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }

      // Check if average heart rate is below 60 or above 100 for half of an hour
      if (beatAvg < 60) {
        if (lowStartTime == 0) { // Start the timer
          lowStartTime = millis();
        } else if (millis() - lowStartTime >= 30 * 60 * 1000) { // Check if half of an hour has passed
          Serial.println("Need help: Heart rate has been below 60 for half of an hour");
          sendAlternativeDistressSignalAndLocation();
          lowStartTime = 0; // Reset the timer
        }
      } else if (beatAvg > 100) {
        if (highStartTime == 0) { // Start the timer
          highStartTime = millis();
        } else if (millis() - highStartTime >= 30 * 60 * 1000) { // Check if half of an hour has passed
          Serial.println("Need help: Heart rate has been above 100 for half of an hour");
          sendAlternativeDistressSignalAndLocation();
          highStartTime = 0; // Reset the timer
        }
      } else { // Reset the timers if heart rate is normal
        lowStartTime = 0;
        highStartTime = 0;
      }
    }
    sendHeartRateDataToFirebase();
  }

  if (irValue < 7000) {
    beatAvg = 0;

    sprHeartRate.fillSprite(TFT_BLACK);

    sprHeartRate.setFreeFont(&Orbitron_Medium_15);
    sprHeartRate.setTextColor(TFT_WHITE,TFT_BLACK);
    sprHeartRate.drawString("BPM:", 0, 0);
    sprHeartRate.setFreeFont(&Orbitron_Medium_25);
    sprHeartRate.drawString(String(beatAvg), 0, 20);

    sprHeartRate.pushSprite(0,170);
  }

  sendBodyTemperatureToFirebase();

  sendAmbientTemperatureToFirebase();

  readBattery();

  signalIndicator();

}

void displayHeartRateData(long irValue) {
  sprHeartRate.fillSprite(TFT_BLACK);

  sprHeartRate.setFreeFont(&Orbitron_Medium_15);
  sprHeartRate.setTextColor(TFT_WHITE,TFT_BLACK);
  sprHeartRate.drawString("BPM:", 0, 0);
  sprHeartRate.setFreeFont(&Orbitron_Medium_25);
  sprHeartRate.drawString(String(beatAvg), 0, 20);

  sprHeartRate.pushSprite(0,170);
}

void sendHeartRateDataToFirebase() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    if (Firebase.RTDB.setInt(&fbdo, "sensor/heartRate", beatAvg)) {
      Serial.println(beatAvg);
      Serial.print("- Successfully saved heart rate to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ")");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }
}

void displayBodyTemperatureData(float bodyTempC) {
  sprBodyTemperature.fillSprite(TFT_BLACK);

  sprBodyTemperature.setFreeFont(&Orbitron_Medium_12);
  sprBodyTemperature.setTextColor(TFT_WHITE,TFT_BLACK);
  sprBodyTemperature.drawString("body", 2, 0);
  sprBodyTemperature.drawString("temp:", 2, 15);
  sprBodyTemperature.setFreeFont(&Orbitron_Medium_12);
  sprBodyTemperature.drawString(String(bodyTempC), 2, 32);

  sprBodyTemperature.pushSprite(70,100);
}

void sendBodyTemperatureToFirebase() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    float bodyTempC = mlx.readObjectTempC();

    // Round to two decimal places
    bodyTempC = roundf(bodyTempC * 100) / 100;

    displayBodyTemperatureData(bodyTempC);

    if (Firebase.RTDB.setFloat(&fbdo, "sensor/bodyTemperature", bodyTempC)) {
      Serial.println(bodyTempC);
      Serial.print("- Successfully saved body temperature to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ")");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }
}

void displayAmbientTemperatureData(float ambientTempF) {
  sprAmbientTemperature.fillSprite(TFT_BLACK);

  sprAmbientTemperature.setFreeFont(&Orbitron_Medium_12);
  sprAmbientTemperature.setTextColor(TFT_WHITE,TFT_BLACK);
  sprAmbientTemperature.drawString("ambient", 2, 0);
  sprAmbientTemperature.drawString("temp:", 2, 15);
  sprAmbientTemperature.setFreeFont(&Orbitron_Medium_12);
  sprAmbientTemperature.drawString(String(ambientTempF), 2, 32);

  sprAmbientTemperature.pushSprite(70,170);
}

void sendAmbientTemperatureToFirebase() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    float ambientTempF = mlx.readAmbientTempF();

    // Round to two decimal places
    ambientTempF = roundf(ambientTempF * 100) / 100;

    displayAmbientTemperatureData(ambientTempF);

    if (Firebase.RTDB.setFloat(&fbdo, "sensor/ambientTemperature", ambientTempF)) {
      Serial.println(ambientTempF);
      Serial.print("- Successfully saved ambient temperature to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ")");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }
}

void sendLocationCoordinateToFirebase() {
  if(gps.location.isValid() && signupOK) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    if (Firebase.RTDB.setFloat(&fbdo, "locationCoordinate/latitude", latitude) && Firebase.RTDB.setFloat(&fbdo, "locationCoordinate/longitude", longitude)) {
      Serial.println(latitude, 6);
      Serial.print("- Successfully saved latitude to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ")");
      Serial.println(longitude, 6);
      Serial.print("- Successfully saved longitude to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ")");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS data received: check wiring"));
  }
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void sendDistressSignalAndLocation() {
  if (digitalRead(BUTTON_PIN_1) == LOW && digitalRead(BUTTON_PIN_2) == LOW && signupOK) {
    bool bVal;

    sendLocationCoordinateToFirebase();

    Serial.printf("Send distress signal: %s\n", Firebase.RTDB.setBool(&fbdo, F("isClimberDanger"), true) ? "success" : fbdo.errorReason().c_str());

    sprDistressSignal.fillSprite(TFT_BLACK);

    sprDistressSignal.setFreeFont(&Orbitron_Medium_10);
    sprDistressSignal.setTextColor(TFT_GREEN,TFT_BLACK);
    sprDistressSignal.drawString("Distress signal sent.", 0, 0);

    sprDistressSignal.pushSprite(0,30);
  }
}

void sendAlternativeDistressSignalAndLocation() {
    sendLocationCoordinateToFirebase();

    Serial.printf("Send alternative distress signal: %s\n", Firebase.RTDB.setBool(&fbdo, F("isClimberDanger"), true) ? "success" : fbdo.errorReason().c_str());

    sprDistressSignal.fillSprite(TFT_BLACK);

    sprDistressSignal.setFreeFont(&Orbitron_Medium_10);
    sprDistressSignal.setTextColor(TFT_GREEN,TFT_BLACK);
    sprDistressSignal.drawString("Distress signal sent.", 0, 0);

    sprDistressSignal.pushSprite(0,30);
}

void readBattery() {
  // Read the battery voltage
  float voltage = analogRead(ADC_PIN) / 4095.0 * VREF * VOLTAGE_DIVIDER_RATIO;

  // Map the voltage to a percentage (0-100)
  int percentage = map(voltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 0, 100);

  // Check if the device is charging
  bool isCharging = isBatteryCharging();

  // Draw the battery indicator
  drawBatteryIndicator(percentage, isCharging);
}

bool isBatteryCharging() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMeasurementMillis >= MEASUREMENT_INTERVAL) {
    lastMeasurementMillis = currentMillis;

    float voltage = analogRead(ADC_PIN) / 4095.0 * VREF * VOLTAGE_DIVIDER_RATIO;
    bool isCharging = (voltage - lastVoltage) > VOLTAGE_INCREASE_THRESHOLD;

    lastVoltage = voltage;

    return isCharging;
  }

  // If not enough time has passed since the last measurement, return false
  return false;
}

void drawBatteryIndicator(int percentage, bool isCharging) {
  sprBatteryIndicator.fillSprite(TFT_BLACK);

  if(isCharging) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_charge);
  }

  if (percentage > 99) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_full);
  } else if (percentage > 59 && percentage < 100) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_high);
  } else if (percentage > 39 && percentage < 60) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_half);
  } else if (percentage > 19 && percentage < 40) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_low);
  } else if (percentage > 1 && percentage < 20) {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_warn);
  } else {
    sprBatteryIndicator.pushImage(40, 0, 24, 24, battery_empty);
  }

  sprBatteryIndicator.pushSprite(69, 0);
}

void signalIndicator() {
  int rssi = WiFi.RSSI();

  sprSignalIndicator.fillSprite(TFT_BLACK);
  sprSignalIndicator.pushImage(0, 4, 16, 16, signal_indicator);
  sprSignalIndicator.setTextColor(TFT_WHITE,TFT_BLACK);
  sprSignalIndicator.drawString(String(rssi) + "dBm", 18, 8);

  sprSignalIndicator.pushSprite(0,0);
}