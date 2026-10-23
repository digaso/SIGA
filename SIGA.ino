#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <WiFiS3.h>

// --- Wi-Fi credentials ---
const char WIFI_SSID[] = "";
const char WIFI_PASSWORD[] = "";

WiFiServer server(80);
bool serverRunning = false;

Adafruit_MPU6050 mpu;
ArduinoLEDMatrix matrix;

// --- Button pins ---
const int PIN_PAUSE_BUTTON = 2;
const int PIN_RESET_BUTTON = 4;

// --- Timing constants ---
const unsigned long SAMPLE_INTERVAL = 20;          // ms between sensor reads (~50 Hz)
const unsigned long MIN_STEP_INTERVAL = 300;       // ms minimum between two steps
const unsigned long BUTTON_DEBOUNCE = 200;         // ms
const unsigned long WIFI_RETRY_INTERVAL = 10000;   // ms between reconnection attempts
const unsigned long DISPLAY_SWITCH_INTERVAL = 2000; // ms between heart and step count after the goal

// --- Step detection constants ---
const float STEP_THRESHOLD = 1.2;    // m/s^2 above baseline to register a step
const float REARM_THRESHOLD = 0.5;   // must drop below this before the next step
const float SMOOTHING = 0.5;         // low-pass factor for the magnitude (0-1)
const float BASELINE_RATE = 0.005;   // how fast the baseline follows the sensor

// --- Step state ---
int stepCount = 0;
int stepGoal = 5000;                 // daily goal, can be changed from the dashboard
float filteredMagnitude = 9.81;
float baseline = 9.81;
bool stepArmed = true;
unsigned long lastSampleTime = 0;
unsigned long lastStepTime = 0;
unsigned long lastWifiRetry = 0;

// --- Display state ---
int displayedSteps = -1;             // last number drawn on the LED matrix
bool heartShown = false;

// Heart shown on the 8x12 LED matrix when the goal is reached
uint8_t heart[8][12] = {
  { 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0 },
  { 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0 },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 }
};

// --- Button state ---
bool paused = false;
bool lastPauseReading = HIGH;
bool lastResetReading = HIGH;
unsigned long lastPausePress = 0;
unsigned long lastResetPress = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PAUSE_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);

  matrix.begin();

  if (!mpu.begin()) {
    Serial.println("Error: MPU6050 not detected. Check the wiring.");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWifi();
}

void loop() {
  unsigned long now = millis();

  handleButtons(now);
  handleWifi(now);

  if (!paused && now - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = now;
    updateSteps(now);
  }

  updateDisplay(now);

  if (serverRunning) {
    handleClient();
  }
}

// Reads both buttons and applies pause/resume and reset on each press.
void handleButtons(unsigned long now) {
  bool pauseReading = digitalRead(PIN_PAUSE_BUTTON);
  if (pauseReading == LOW && lastPauseReading == HIGH && now - lastPausePress > BUTTON_DEBOUNCE) {
    paused = !paused;
    lastPausePress = now;
  }
  lastPauseReading = pauseReading;

  bool resetReading = digitalRead(PIN_RESET_BUTTON);
  if (resetReading == LOW && lastResetReading == HIGH && now - lastResetPress > BUTTON_DEBOUNCE) {
    resetSteps();
    lastResetPress = now;
  }
  lastResetReading = resetReading;
}

void resetSteps() {
  stepCount = 0;
  stepArmed = true;
}

// Reads the accelerometer, smooths it, and counts a step on each clear peak.
void updateSteps(unsigned long now) {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float magnitude = sqrt(accel.acceleration.x * accel.acceleration.x +
                         accel.acceleration.y * accel.acceleration.y +
                         accel.acceleration.z * accel.acceleration.z);

  // Low-pass filter to remove short noise spikes
  filteredMagnitude = SMOOTHING * magnitude + (1.0 - SMOOTHING) * filteredMagnitude;

  float force = filteredMagnitude - baseline;

  // Only let the baseline follow the sensor while at rest,
  // so steps do not pull the baseline upward.
  if (force < REARM_THRESHOLD) {
    baseline = (1.0 - BASELINE_RATE) * baseline + BASELINE_RATE * filteredMagnitude;
  }

  // Hysteresis: a step needs a rise above STEP_THRESHOLD and a full drop below REARM_THRESHOLD
  if (stepArmed && force > STEP_THRESHOLD) {
    stepArmed = false;
    if (now - lastStepTime > MIN_STEP_INTERVAL) {
      stepCount++;
      lastStepTime = now;
      Serial.print("Steps: ");
      Serial.println(stepCount);
    }
  } else if (!stepArmed && force < REARM_THRESHOLD) {
    stepArmed = true;
  }
}

// Shows the step count, or alternates heart and step count once the goal is reached.
void updateDisplay(unsigned long now) {
  bool goalReached = stepGoal > 0 && stepCount >= stepGoal;
  bool heartTurn = (now / DISPLAY_SWITCH_INTERVAL) % 2 == 0;

  if (goalReached && heartTurn) {
    showHeart();
  } else {
    showNumber(stepCount);
  }
}

void showHeart() {
  if (heartShown) {
    return;
  }
  matrix.renderBitmap(heart, 8, 12);
  heartShown = true;
  displayedSteps = -1;
}

// Draws the step count. Values of 1000 or more are shown in thousands ("4k")
// so the text always fits without scrolling, which would block the loop.
void showNumber(int total) {
  if (!heartShown && total == displayedSteps) {
    return;
  }
  heartShown = false;
  displayedSteps = total;

  String text;
  if (total < 1000) {
    text = String(total);
  } else {
    text = String(total / 1000) + "k";
  }

  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(0xFFFFFFFF);
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.print(text);
  matrix.endText(NO_SCROLL);
  matrix.endDraw();
}

// Connects to Wi-Fi. Returns true on success.
bool connectWifi() {
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

  for (int attempt = 0; attempt < 5 && WiFi.status() != WL_CONNECTED; attempt++) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(3000);
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.println("Wi-Fi not available. Counting continues; will retry.");
  return false;
}

// Retries the connection periodically and starts the server once connected.
void handleWifi(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (!serverRunning) {
      server.begin();
      serverRunning = true;
      Serial.print("Dashboard IP address: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  serverRunning = false;
  if (now - lastWifiRetry > WIFI_RETRY_INTERVAL) {
    lastWifiRetry = now;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// Reads one HTTP request, applies it, and replies with the current state as JSON.
void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  unsigned long clientStart = millis();
  String requestLine = "";
  String line = "";
  bool requestComplete = false;

  while (client.connected() && millis() - clientStart < 1000) {
    if (!client.available()) {
      continue;
    }
    char c = client.read();
    if (c == '\n') {
      if (line.length() == 0) {
        requestComplete = true;   // blank line marks the end of the headers
        break;
      }
      if (requestLine.length() == 0) {
        requestLine = line;       // first line, e.g. "GET /goal?value=8000 HTTP/1.1"
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }

  if (requestComplete) {
    handleRequest(requestLine);
    sendStatus(client);
  }

  client.stop();
}

// Routes:
//   GET /                    -> current state
//   GET /goal?value=<steps>  -> sets the daily goal
//   GET /reset               -> sets the step count to 0
void handleRequest(String requestLine) {
  if (requestLine.startsWith("GET /reset")) {
    resetSteps();
  } else if (requestLine.startsWith("GET /goal?value=")) {
    int start = requestLine.indexOf('=') + 1;
    int end = requestLine.indexOf(' ', start);
    int value = requestLine.substring(start, end).toInt();
    if (value > 0) {
      stepGoal = value;
    }
  }
}

void sendStatus(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.print("{\"steps\": ");
  client.print(stepCount);
  client.print(", \"goal\": ");
  client.print(stepGoal);
  client.print(", \"goalReached\": ");
  client.print(stepCount >= stepGoal ? "true" : "false");
  client.print(", \"status\": \"");
  client.print(paused ? "paused" : "running");
  client.println("\"}");
}
