# SIGA – Step Counter

A step counter built with an **Arduino UNO R4 WiFi** and an **MPU6050** accelerometer.
It counts your steps, shows them on the board's LED matrix, and sends them over Wi-Fi to a web dashboard.

## What you need

- Arduino UNO R4 WiFi
- MPU6050 accelerometer module
- 2 push buttons
- Jumper wires (and a breadboard)

## Wiring

### MPU6050

| MPU6050 | Arduino |
|---------|---------|
| VCC     | 5V (or 3.3V) |
| GND     | GND     |
| SDA     | SDA     |
| SCL     | SCL     |

### Buttons

Each button has one leg on a pin and the other leg on **GND**. No resistors needed (the internal pull-up is used).

| Button        | Arduino pin | Other leg |
|---------------|-------------|-----------|
| Pause / resume | D2         | GND       |
| Reset steps    | D4         | GND       |

## How it works

1. The board reads the accelerometer about 50 times per second.
2. It combines X, Y and Z into one value, so it works no matter how the board is held.
3. The signal is smoothed to remove noise.
4. When the signal jumps clearly above its usual level, that counts as a **step**.
   After a step, it waits 300 ms before counting another one (one footstep makes several small bumps).
5. The step count shows on the LED matrix (e.g. `850`, or `4k` above 1000).
6. When you reach your daily goal, a **heart** appears on the matrix for 3 seconds.

**Buttons:** D2 pauses/resumes counting, D4 resets the count to 0.

## Setup

1. Install these libraries in the Arduino IDE: **Adafruit MPU6050** and **ArduinoGraphics**
   (`Arduino_LED_Matrix` and `WiFiS3` come with the UNO R4 board package).
2. Open `SIGA.ino` and fill in your Wi-Fi name and password:
   ```cpp
   const char WIFI_SSID[] = "";
   const char WIFI_PASSWORD[] = "";
   ```
   Don't commit your real password to GitHub.
3. Upload the sketch and open the Serial Monitor (115200 baud).
   It prints the board's IP address, e.g. `Dashboard IP address: 192.168.1.42`.

Wi-Fi is optional. If there's no connection, the board keeps counting and tries to connect again every 10 seconds.

## Dashboard

Open `dashboard.html` in a browser on the **same Wi-Fi network**, type the Arduino's IP address and connect.

It shows:
- Today's steps and a progress ring toward your goal
- Distance (km) and calories burned, based on your height and weight
- A chart of the last 7 days

You can change the daily goal from the dashboard and it's sent to the Arduino.
Your settings and step history are saved in the browser.

### Arduino API

The dashboard talks to the board with simple HTTP requests. Every request returns the current state as JSON:

| Request | What it does |
|---------|--------------|
| `GET /` | Returns the current state |
| `GET /goal?value=8000` | Sets the daily goal |
| `GET /reset` | Resets the step count |

```json
{"steps": 1234, "goal": 5000, "goalReached": false, "status": "running"}
```
