#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>

// =====================
//   PIN DEFINITIONS
// =====================
#define DHTPIN        25
#define DHTTYPE       DHT11
#define RAIN_PIN      33
#define MOISTURE_PIN  32
#define RELAY_PIN     26
#define SIM_RX        16
#define SIM_TX        17

// =====================
//   OLED CONFIG
// =====================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================
//   HARDWARE OBJECTS
// =====================
RTC_DS3231 rtc;
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial sim900(2); // UART2: GPIO16=RX2, GPIO17=TX2

// =====================
//   SYSTEM FLAGS
// =====================
bool displayAvailable  = false;
int  oledFailCount     = 0;     // consecutive OLED write failures
unsigned long lastOledReinit = 0; // millis of last reinit attempt
bool rtcAvailable     = false;
bool simAvailable     = false;

// =====================
//   NON-BLOCKING SMS QUEUE
// =====================
#define SMS_QUEUE_SIZE 4
String   smsQueue[SMS_QUEUE_SIZE];
bool     smsQueueForce[SMS_QUEUE_SIZE];
int      smsQueueHead = 0;
int      smsQueueTail = 0;
int      smsQueueCount = 0;

enum SmsState { SMS_IDLE, SMS_SET_MODE, SMS_SEND_CMD, SMS_WAIT_PROMPT, SMS_SEND_BODY, SMS_WAIT_CONFIRM };
SmsState smsState       = SMS_IDLE;
unsigned long smsStateStart    = 0;
unsigned long smsCooldownUntil = 0; // dont start next send until this millis
String   smsPending     = "";

// =====================
//   TUNING PARAMETERS
// =====================
#define PHONE_NUMBER          "+918088765085"
#define MOISTURE_DRY          4095
#define MOISTURE_WET          1500
#define MOISTURE_LOW          25
#define MOISTURE_HIGH         50
#define MOISTURE_HYSTERESIS   5    // deadband around thresholds to prevent chatter
#define PUMP_COOLDOWN         30000 // 30s minimum between pump cycles
#define RAIN_PER_MINUTE       0.5
#define PUMP_ON_DURATION      20000
#define SENSOR_READ_DELAY     2000
#define SIGNAL_CHECK_INTERVAL 10000
#define PAGE_INTERVAL         4000
#define DHT_READ_ATTEMPTS     3
#define SMS_THROTTLE_TIME     15000  // 15s between auto/alert SMSes; commands bypass this

// =====================
//   STRUCTS (before forward declarations)
// =====================
struct SensorData {
  float temperature;
  float humidity;
  float moisture;
  bool  isRaining;
  bool  isValid;
};

struct SimTime {
  int hour, day, week, month, year;
};

// =====================
//   FORWARD DECLARATIONS
// =====================
void     logError(String component, String message);
bool     canSendSMS(bool force = false);
void     sendSMS(String message, bool force = false);
void     processSMS();
void     setPump(bool state);
void     updateOLED(SensorData data);
void     checkSignal();
void     sendStatusReport();
void     parseIncomingSMS(String text);
void     handleIncomingCall();
void     updateRainStats(SensorData data);
void     handleWateringCycle(SensorData data);
String   sendATCommand(String cmd, int timeout);
bool     initSIM900();

// =====================
//   RAIN STATISTICS
// =====================
float rainDaily   = 0;
float rainWeekly  = 0;
float rainMonthly = 0;
float rainYearly  = 0;

// =====================
//   TIME TRACKING
// =====================
unsigned long simStartMillis = 0;
int lastSimDay    = 0;
int lastSimWeek   = 0;
int lastSimMonth  = 0;
int lastSimYear   = 0;
int lastSimHour   = -1;

// =====================
//   WATERING CYCLE
// =====================
bool wateringCycleActive  = false;
unsigned long pumpStartMillis = 0;
int wateringCountToday    = 0;
int lastWateringSimDay    = -1;
bool rainedToday          = false; // if true, no watering for rest of sim day
int wateringHours[4]      = {0, 6, 12, 18};

// =====================
//   PUMP CONTROL
// =====================
bool pumpRunning    = false;
bool manualOverride = false;
unsigned long pumpOnTime      = 0;
unsigned long pumpLastOffTime = 0; // for cooldown between cycles

// =====================
//   DISPLAY
// =====================
int oledPage = 0;
unsigned long lastPageSwitch = 0;

// =====================
//   SMS
// =====================
unsigned long lastSMSTime = 0;
int failedSMSCount = 0;

// =====================
//   SIGNAL MONITORING
// =====================
int signalBars = 0;
int signalDBm  = 0;
unsigned long lastSignalCheck = 0;
unsigned long lastRingTime    = 0;
unsigned long lastSensorRead  = 0;

// =====================
//   SENSOR SNAPSHOT
// =====================
SensorData lastKnownData;

// =====================
//   HELPER: Moisture %
// =====================
float getMoisturePercent(int raw) {
  return constrain(map(raw, MOISTURE_DRY, MOISTURE_WET, 0, 100), 0, 100);
}

// =====================
//   HELPER: logError
// =====================
void logError(String component, String message) {
  Serial.printf("[ERROR] %s: %s\n", component.c_str(), message.c_str());
}

// =====================
//   HELPER: Set Pump
// =====================
void setPump(bool state) {
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  pumpRunning = state;
  if (state) {
    pumpOnTime = millis();
  } else {
    pumpLastOffTime = millis(); // record when pump turned off for cooldown
  }
  Serial.println(state ? "[PUMP] ON" : "[PUMP] OFF");
}

// =====================
//   HELPER: SMS Throttle
//   force=true bypasses throttle for command replies & critical alerts
// =====================
bool canSendSMS(bool force) {
  if (force || millis() - lastSMSTime >= SMS_THROTTLE_TIME) {
    lastSMSTime = millis();
    return true;
  }
  return false;
}

// =====================
//   HELPER: Read Sensors
// =====================
SensorData readSensors() {
  SensorData data;
  data.isValid = true;

  float tempSum = 0, humSum = 0;
  int validReadings = 0;
  for (int i = 0; i < DHT_READ_ATTEMPTS; i++) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h) && t >= -40 && t <= 125 && h >= 0 && h <= 100) {
      tempSum += t;
      humSum  += h;
      validReadings++;
    }
    delay(100);
  }
  if (validReadings > 0) {
    data.temperature = tempSum / validReadings;
    data.humidity    = humSum  / validReadings;
  } else {
    data.temperature = 0;
    data.humidity    = 0;
    data.isValid     = false;
    logError("DHT11", "All read attempts failed");
  }

  int moistSum = 0;
  for (int i = 0; i < 3; i++) moistSum += analogRead(MOISTURE_PIN);
  data.moisture = getMoisturePercent(moistSum / 3);

  bool rain1 = (digitalRead(RAIN_PIN) == LOW);
  delay(50);
  bool rain2 = (digitalRead(RAIN_PIN) == LOW);
  data.isRaining = (rain1 && rain2);

  return data;
}

// =====================
//   HELPER: Sim Time
// =====================
SimTime getSimTime() {
  // Time scale: 1 real second = 8 sim minutes → 1 sim day = 3 real minutes
  unsigned long elapsed    = (millis() - simStartMillis) / 1000;
  unsigned long simMinutes = elapsed * 8;
  SimTime st;
  st.hour  = (simMinutes % 1440) / 60;
  st.day   = (simMinutes / 1440) % 7;
  st.week  = (simMinutes / 10080) % 4;
  st.month = (simMinutes / 43200) % 12;
  st.year  = (simMinutes / 525600);
  return st;
}

// =====================
//   OLED: Draw Signal Bars (top-right corner)
//   x,y = top-left of bar cluster
//   bars = 0..4
// =====================
void drawSignalBars(int x, int y, int bars) {
  // 4 bars, each taller than last: w=3px, gap=1px
  // heights: 4, 6, 8, 10 px; baseline at y+10
  int barW    = 3;
  int gap     = 1;
  int heights[4] = {4, 6, 8, 10};
  int baseline = y + 10;
  for (int i = 0; i < 4; i++) {
    int bx = x + i * (barW + gap);
    int bh = heights[i];
    int by = baseline - bh;
    if (i < bars) {
      display.fillRect(bx, by, barW, bh, SSD1306_WHITE); // filled = active
    } else {
      display.drawRect(bx, by, barW, bh, SSD1306_WHITE); // outline = inactive
    }
  }
}

// =====================
//   OLED: Draw Moisture Bar
//   full-width progress bar
// =====================
void drawMoistureBar(int y, float pct) {
  int barX = 0, barW = 128, barH = 6;
  display.drawRect(barX, y, barW, barH, SSD1306_WHITE);
  int fill = (int)((pct / 100.0f) * (barW - 2));
  if (fill > 0) display.fillRect(barX + 1, y + 1, fill, barH - 2, SSD1306_WHITE);
}

// =====================
//   OLED: Status Bar (top row, every page)
//   Shows: pump indicator | page dots | signal bars
// =====================
void drawStatusBar(int page) {
  // Pump dot
  if (pumpRunning) {
    display.fillCircle(3, 3, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(3, 3, 3, SSD1306_WHITE);
  }
  // "P" label
  display.setTextSize(1);
  display.setCursor(9, 0);
  display.print(manualOverride ? "M" : "A"); // M=manual A=auto

  // Page dots (center)
  int dotX = 56;
  for (int i = 0; i < 4; i++) {
    if (i == page) display.fillCircle(dotX + i * 6, 3, 2, SSD1306_WHITE);
    else           display.drawCircle(dotX + i * 6, 3, 2, SSD1306_WHITE);
  }

  // Signal bars (top-right)
  drawSignalBars(112, 0, signalBars);

  // Divider line
  display.drawFastHLine(0, 8, 128, SSD1306_WHITE);
}

// =====================
//   OLED: Update Display (4 rotating pages)
// =====================
void updateOLED(SensorData data) {
  if (!displayAvailable) return;

  if (millis() - lastPageSwitch >= PAGE_INTERVAL) {
    oledPage = (oledPage + 1) % 4;
    lastPageSwitch = millis();
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawStatusBar(oledPage);

  SimTime st = getSimTime();

  switch (oledPage) {

    // ---- PAGE 0: Temperature & Humidity ----
    case 0:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("ENVIRONMENT");

      display.setTextSize(2);
      display.setCursor(0, 22);
      if (data.isValid) {
        display.printf("%.1f", data.temperature);
        display.setTextSize(1);
        display.print(" C");
      } else {
        display.print("--.-");
        display.setTextSize(1);
        display.print(" ERR");
      }

      display.setTextSize(2);
      display.setCursor(0, 44);
      if (data.isValid) {
        display.printf("%.1f", data.humidity);
        display.setTextSize(1);
        display.print(" %RH");
      } else {
        display.print("--.-");
      }
      break;

    // ---- PAGE 1: Soil Moisture & Rain ----
    case 1:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("SOIL & RAIN");

      // Moisture big number
      display.setTextSize(2);
      display.setCursor(0, 22);
      display.printf("%d%%", (int)data.moisture);

      // Status tag — 3-tier moisture zones
      display.setTextSize(1);
      display.setCursor(52, 24);
      if      (data.moisture < MOISTURE_LOW)   display.print("[DRY!]");  // 0-25%: pump now
      else if (data.moisture <= MOISTURE_HIGH) display.print("[SCHED]"); // 25-50%: interval
      else                                     display.print("[WET] ");  // >50%: no water

      // Moisture bar
      drawMoistureBar(38, data.moisture);

      // Rain status - ALL IN ONE LINE
      display.setCursor(0, 49);
      display.printf("R:%s D:%.1f W:%.1f M:%.1fmm", 
        data.isRaining ? "YES" : "No ", rainDaily, rainWeekly, rainMonthly);
      break;

    // ---- PAGE 2: Pump & Watering ----
    case 2:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("PUMP STATUS");

      display.setTextSize(2);
      display.setCursor(0, 22);
      display.println(pumpRunning ? "RUNNING" : "IDLE   ");

      display.setTextSize(1);
      display.setCursor(0, 42);
      display.printf("Mode: %s", manualOverride ? "MANUAL" : "AUTO  ");
      display.setCursor(0, 54);
      display.printf("Cycles today: %d", wateringCountToday);
      break;

    // ---- PAGE 3: Signal & Time ----
    case 3:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("NETWORK & TIME");

      // Big signal bars (decorative, larger)
      {
        int bx = 0, by = 22;
        int bW = 8, gap = 3;
        int heights2[4] = {8, 14, 20, 26};
        int baseline2 = by + 26;
        for (int i = 0; i < 4; i++) {
          int px = bx + i * (bW + gap);
          int bh = heights2[i];
          int py = baseline2 - bh;
          if (i < signalBars)
            display.fillRect(px, py, bW, bh, SSD1306_WHITE);
          else
            display.drawRect(px, py, bW, bh, SSD1306_WHITE);
        }
        // dBm label
        display.setCursor(50, 30);
        display.setTextSize(1);
        if (signalBars == 0) display.print("No signal");
        else {
          display.printf("%d dBm", signalDBm);
          display.setCursor(50, 42);
          display.printf("%d/4 bars", signalBars);
        }
      }

      display.setCursor(0, 54);
      display.printf("SimHr:%02d  Day:%d Wk:%d", st.hour, st.day, st.week);
      break;
  }

  // Write to display — if it fails (inrush current glitch), reinit
  // Adafruit_SSD1306 doesn't return errors, so we detect by re-checking I2C
  display.display();

  // Periodic OLED health check — reinit if it went blank due to voltage sag
  if (millis() - lastOledReinit > 5000) {
    lastOledReinit = millis();
    Wire.beginTransmission(OLED_ADDRESS);
    byte err = Wire.endTransmission();
    if (err != 0) {
      // I2C device not responding — reinit
      oledFailCount++;
      Serial.printf("[OLED] I2C err %d, reinit attempt #%d\n", err, oledFailCount);
      displayAvailable = false;
      delay(100);
      if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        displayAvailable = true;
        oledFailCount = 0;
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        Serial.println("[OLED] Reinit OK");
      }
    } else {
      oledFailCount = 0; // I2C healthy
    }
  }
}

// =====================
//   SERIAL PORT OWNERSHIP
// =====================
String simBuffer = ""; // persistent RX buffer for handleIncomingCall
bool   simBusy   = false; // true while sendATCommand owns serial

// =====================
//   SIM900A: Send AT Command (FIXED: Longer timeout)
// =====================
String sendATCommand(String cmd, int timeout) {
  simBusy   = true;
  simBuffer = "";
  delay(50);                            // brief wait so any in-flight bytes arrive
  while (sim900.available()) sim900.read(); // flush hardware buffer
  sim900.println(cmd);

  String response = "";
  unsigned long start = millis();
  while (millis() - start < (unsigned long)timeout) {
    while (sim900.available()) {
      response += (char)sim900.read();
    }
    if (response.indexOf("OK")      != -1 ||
        response.indexOf("ERROR")   != -1 ||
        response.indexOf(">")       != -1 ||
        response.indexOf("CONNECT") != -1) break;
    // NOTE: +CMGS removed from early-exit — it appears in AT+CMGS="..." echo
    // and causes premature bailout before the > prompt arrives
    delay(5);
  }
  response.trim();
  simBusy = false;                      // release serial port back to handleIncomingCall
  if (response.length() > 0)
    Serial.printf("[SIM] CMD:%s | RSP:%s\n", cmd.c_str(), response.c_str());
  return response;
}

// =====================
//   SIM900A: Init (FIXED: Add SMSC setup)
// =====================
bool initSIM900() {
  Serial.println("[SIM] Initializing SIM900A...");
  for (int i = 0; i < 5; i++) {
    String rsp = sendATCommand("AT", 1000);
    if (rsp.indexOf("OK") != -1) {
      sendATCommand("ATE0",             500);  // echo off
      sendATCommand("AT+CMGF=1",        500);  // text mode
      sendATCommand("AT+CNMI=2,2,0,0,0",500);  // SMS direct to serial
      
      // FIXED: Set SMSC for India carriers (Jio/Airtel)
      // Change this number based on your carrier
      sendATCommand("AT+CSCA=\"+919999999999\"", 1000);
      
      Serial.println("[SIM] SIM900A ready");
      return true;
    }
    delay(1000);
  }
  logError("SIM900A", "No response to AT");
  return false;
}

// =====================
//   SIM900A: Send SMS (COMPLETELY FIXED)
// =====================
// Enqueue an SMS — returns immediately, no blocking
void sendSMS(String message, bool force) {
  if (!simAvailable) { logError("SMS", "SIM not available"); return; }
  if (!force && (millis() - lastSMSTime < SMS_THROTTLE_TIME)) {
    Serial.println("[SMS] Throttled");
    return;
  }
  if (message.length() > 160) { logError("SMS", "Too long"); return; }
  if (smsQueueCount >= SMS_QUEUE_SIZE) { logError("SMS", "Queue full"); return; }
  if (force) lastSMSTime = millis();

  smsQueue[smsQueueTail]      = message;
  smsQueueForce[smsQueueTail] = force;
  smsQueueTail = (smsQueueTail + 1) % SMS_QUEUE_SIZE;
  smsQueueCount++;
  Serial.println("[SMS] Queued: " + message.substring(0, 40));
}

// =====================
//   NON-BLOCKING SMS STATE MACHINE — call every loop()
// =====================
void processSMS() {
  if (!simAvailable) return;

  unsigned long now = millis();

  // Nothing to do
  if (smsState == SMS_IDLE && smsQueueCount == 0) return;

  // Cooldown between sends — SIM900A needs recovery time
  if (smsState == SMS_IDLE && now < smsCooldownUntil) return;

  // Start next queued message
  if (smsState == SMS_IDLE && smsQueueCount > 0) {
    if (signalBars < 1) return;
    smsPending   = smsQueue[smsQueueHead];
    smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
    smsQueueCount--;
    smsState      = SMS_SET_MODE;
    smsStateStart = now;
    simBusy       = true;
    while (sim900.available()) sim900.read(); // flush stale bytes
    sim900.println("AT+CMGF=1");
    Serial.println("[SMS] State: SET_MODE -> " + smsPending.substring(0, 30));
    return;
  }

  switch (smsState) {

    case SMS_SET_MODE: {
      // Drain all available bytes into a response string
      String r = "";
      while (sim900.available()) r += (char)sim900.read();
      if (r.indexOf("OK") != -1) {
        smsState      = SMS_SEND_CMD;
        smsStateStart = now;
        sim900.println("AT+CMGS="" + String(PHONE_NUMBER) + """);
        Serial.println("[SMS] State: SEND_CMD");
      } else if (r.indexOf("ERROR") != -1) {
        Serial.println("[SMS] SET_MODE ERROR — retrying in 3s");
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 3000;
        // Put message back at front of queue
        smsQueueHead = (smsQueueHead - 1 + SMS_QUEUE_SIZE) % SMS_QUEUE_SIZE;
        smsQueue[smsQueueHead] = smsPending;
        smsQueueCount++;
      } else if (now - smsStateStart > 4000) {
        Serial.println("[SMS] SET_MODE timeout — retrying in 2s");
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 2000;
        smsQueueHead = (smsQueueHead - 1 + SMS_QUEUE_SIZE) % SMS_QUEUE_SIZE;
        smsQueue[smsQueueHead] = smsPending;
        smsQueueCount++;
      }
      break;
    }

    case SMS_SEND_CMD: {
      String r = "";
      while (sim900.available()) r += (char)sim900.read();
      if (r.indexOf(">") != -1) {
        smsState      = SMS_WAIT_CONFIRM;
        smsStateStart = now;
        sim900.print(smsPending);
        delay(100);
        sim900.write(0x1A); // Ctrl+Z
        Serial.println("[SMS] State: WAIT_CONFIRM");
      } else if (r.indexOf("ERROR") != -1 || now - smsStateStart > 6000) {
        Serial.println("[SMS] SEND_CMD failed");
        failedSMSCount++;
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 3000;
      }
      break;
    }

    case SMS_WAIT_CONFIRM: {
      String r = "";
      while (sim900.available()) r += (char)sim900.read();
      if (r.indexOf("+CMGS:") != -1 || r.indexOf("OK") != -1) {
        Serial.println("[SMS] ✓ Sent: " + smsPending.substring(0, 40));
        failedSMSCount   = 0;
        lastSMSTime      = now;
        smsState         = SMS_IDLE;
        simBusy          = false;
        smsCooldownUntil = now + 4000; // 4s before next send
      } else if (r.indexOf("ERROR") != -1) {
        Serial.println("[SMS] ✗ Error on confirm");
        failedSMSCount++;
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 3000;
      } else if (now - smsStateStart > 8000) {
        // No echo — SIM900A quirk, treat as sent
        Serial.println("[SMS] ✓ Assumed sent (no echo)");
        failedSMSCount   = 0;
        lastSMSTime      = now;
        smsState         = SMS_IDLE;
        simBusy          = false;
        smsCooldownUntil = now + 5000; // longer cooldown after no-echo
      }
      break;
    }

    default:
      smsState = SMS_IDLE; simBusy = false;
      break;
  }

  if (failedSMSCount >= 5) {
    Serial.println("[SMS] 5 failures — reinit SIM");
    initSIM900();
    failedSMSCount   = 0;
    smsState         = SMS_IDLE;
    simBusy          = false;
    smsCooldownUntil = now + 5000;
  }
}




// =====================
//   SIM900A: Check Signal
// =====================
void checkSignal() {
  if (!simAvailable) return;
  if (simBusy) return; // don't interrupt an ongoing SMS send
  String rsp = sendATCommand("AT+CSQ", 1000);
  int idx = rsp.indexOf("+CSQ:");
  if (idx != -1) {
    int rssi = rsp.substring(idx + 6).toInt();
    if (rssi == 99) {
      signalBars = 0; signalDBm = 0;
    } else {
      signalDBm  = -113 + (rssi * 2);
      signalBars = (rssi >= 20) ? 4 :
                   (rssi >= 15) ? 3 :
                   (rssi >= 10) ? 2 :
                   (rssi >=  5) ? 1 : 0;
    }
    Serial.printf("[SIM] Signal: %ddBm (%d/4 bars)\n", signalDBm, signalBars);
  }
}

// =====================
//   ON-DEMAND STATUS REPORT (FIXED: Rain data in one line)
// =====================
void sendStatusReport() {
  SimTime st = getSimTime();
  
  // FIXED: Make message shorter and put all rain data on one line
  String msg = "SOIL:" + String((int)lastKnownData.moisture) + "%";
  if      (lastKnownData.moisture < MOISTURE_LOW)   msg += "(DRY-PUMP) ";  // 0-25%
  else if (lastKnownData.moisture <= MOISTURE_HIGH) msg += "(SCHED) ";     // 25-50%
  else                                              msg += "(WET-SKIP) ";  // >50%
  msg += "RAIN:" + String(lastKnownData.isRaining ? "YES" : "NO") + " ";
  msg += "T:" + String((int)lastKnownData.temperature) + "C H:" + String((int)lastKnownData.humidity) + "% ";
  msg += "PUMP:" + String(pumpRunning ? "ON" : "OFF") + " ";
  msg += "MODE:" + String(manualOverride ? "MANUAL" : "AUTO") + " ";
  msg += "CYCLES:" + String(wateringCountToday) + " ";
  msg += "RAIN(D/W/M):" + String(rainDaily, 1) + "/" + String(rainWeekly, 1) + "/" + String(rainMonthly, 1) + "mm";
  
  // Ensure message doesn't exceed 160 chars
  if (msg.length() > 160) {
    msg = msg.substring(0, 160);
  }
  
  Serial.println("[SMS] Status report length: " + String(msg.length()) + " chars");
  sendSMS(msg, true);  // force: report sent to both numbers
}

// =====================
//   PARSE INCOMING SMS COMMAND
//   REPORT   → full status
//   PUMP ON  → manual pump on
//   PUMP OFF → manual pump off
//   AUTO     → restore auto mode
// =====================
void parseIncomingSMS(String text) {
  text.trim();
  text.toUpperCase();
  Serial.println("[SMS] Cmd: " + text);

  if      (text.indexOf("REPORT")   != -1) { sendStatusReport(); }
  else if (text.indexOf("PUMP ON")  != -1) { manualOverride = true;  setPump(true);  sendSMS("OK: Pump ON. Manual mode active.", true); }
  else if (text.indexOf("PUMP OFF") != -1) { manualOverride = true;  setPump(false); sendSMS("OK: Pump OFF. Manual mode active.", true); }
  else if (text.indexOf("AUTO")     != -1) { manualOverride = false; setPump(false); sendSMS("OK: Auto mode restored. Pump OFF.", true); }
  else                                     { sendSMS("Cmds: REPORT / PUMP ON / PUMP OFF / AUTO", true); }
}

// =====================
//   PERSISTENT SERIAL BUFFER
//   Accumulates SIM900A output across loop() calls
//   Processes only when a full line (\n) arrives
// =====================

void processSimLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.println("[SIM RX] " + line);

  // --- Incoming CALL ---
  if (line.indexOf("RING") != -1) {
    unsigned long now = millis();
    if (now - lastRingTime > 3000) {
      lastRingTime = now;
      Serial.println("[CALL] Ring detected → ATH + report");
      delay(600);
      sendATCommand("ATH", 1000);
      delay(500);
      sendStatusReport();
    }
    return;
  }

  // --- Incoming SMS header: +CMT: "+91...","","date" ---
  // Next line in buffer will be the message body
  // We flag it and read the NEXT line as the SMS text
  static bool nextLineIsSMS = false;
  if (line.indexOf("+CMT:") != -1) {
    nextLineIsSMS = true;
    return;
  }
  if (nextLineIsSMS) {
    nextLineIsSMS = false;
    parseIncomingSMS(line);
    return;
  }
}

void handleIncomingCall() {
  if (simBusy) return;                  // AT command in progress — don't steal bytes
  // Drain all available bytes into persistent buffer
  while (sim900.available()) {
    char c = (char)sim900.read();
    simBuffer += c;
  }

  // Process every complete line in the buffer
  int nlIdx;
  while ((nlIdx = simBuffer.indexOf('\n')) != -1) {
    String line = simBuffer.substring(0, nlIdx);
    simBuffer   = simBuffer.substring(nlIdx + 1);
    processSimLine(line);
  }

  // Safety: discard if buffer grows too large (garbage data)
  if (simBuffer.length() > 512) {
    Serial.println("[SIM] Buffer overflow — clearing");
    simBuffer = "";
  }
}

// =====================
//   RAIN TRACKING
// =====================
void updateRainStats(SensorData data) {
  SimTime st = getSimTime();
  if (data.isRaining) {
    rainDaily   += RAIN_PER_MINUTE;
    rainWeekly  += RAIN_PER_MINUTE;
    rainMonthly += RAIN_PER_MINUTE;
    rainYearly  += RAIN_PER_MINUTE;
  }
  if (st.day   != lastSimDay)   { rainDaily   = 0; lastSimDay   = st.day;   }
  if (st.week  != lastSimWeek)  { rainWeekly  = 0; lastSimWeek  = st.week;  }
  if (st.month != lastSimMonth) { rainMonthly = 0; lastSimMonth = st.month; }
  if (st.year  != lastSimYear)  { rainYearly  = 0; lastSimYear  = st.year;  }
}

// =====================
//   WATERING CYCLE LOGIC
//
//   Moisture Tiers (AUTO mode only):
//   0–25%   → CRITICAL DRY: Pump ON immediately, no schedule needed
//   25–50%  → MODERATE: Water only at 6-hour scheduled intervals (0,6,12,18)
//   >50%    → WET: No watering for the day
// =====================
void handleWateringCycle(SensorData data) {
  if (manualOverride) return;

  SimTime st = getSimTime();

  // Reset daily counters on new sim day
  if (st.day != lastWateringSimDay) {
    wateringCountToday = 0;
    lastWateringSimDay = st.day;
    rainedToday        = false; // new day — rain flag clears
  }

  // Auto-stop pump after duration
  if (pumpRunning && (millis() - pumpOnTime >= PUMP_ON_DURATION)) {
    setPump(false);
    wateringCycleActive = false;
    Serial.println("[PUMP] Auto-stop after duration");
  }

  // Rain check — if raining NOW, set flag and block for rest of sim day
  if (data.isRaining) {
    if (!rainedToday) {
      rainedToday = true;
      if (pumpRunning) { setPump(false); }
      Serial.println("[PUMP] Rain detected — watering blocked for rest of day");
      sendSMS("Rain detected. No watering today.");
    } else {
      if (pumpRunning) { setPump(false); }
    }
    return;
  }
  // Even if not raining now, if it rained earlier today — still block
  if (rainedToday) {
    if (pumpRunning) { setPump(false); }
    Serial.println("[PUMP] Blocked — rained earlier today");
    return;
  }

  // Cooldown guard — don't restart pump too soon after it last turned off
  bool inCooldown = (millis() - pumpLastOffTime < PUMP_COOLDOWN);
  if (inCooldown && !pumpRunning) {
    Serial.printf("[PUMP] Cooldown active (%.0fs left)\n",
                  (PUMP_COOLDOWN - (millis() - pumpLastOffTime)) / 1000.0);
    return;
  }

  // ---- TIER 1: CRITICAL DRY (0 to MOISTURE_LOW - HYSTERESIS) ----
  // Use hysteresis: turn ON below LOW, don't turn OFF until above LOW+HYSTERESIS
  // This prevents chatter when moisture bounces near the 25% boundary
  float dryThresholdOn  = MOISTURE_LOW;                      // turn ON  below 25%
  float dryThresholdOff = MOISTURE_LOW + MOISTURE_HYSTERESIS; // turn OFF above 30%

  if (pumpRunning && wateringCycleActive) {
    // Pump is already running in tier-1 mode — only stop it via duration timer (handled above)
    // or if soil has recovered past the hysteresis OFF threshold
    if (data.moisture >= dryThresholdOff) {
      setPump(false);
      wateringCycleActive = false;
      Serial.printf("[PUMP] DRY tier: soil recovered to %.1f%% (>%d%%) — stopping\n",
                    data.moisture, (int)dryThresholdOff);
    }
    return;
  }

  if (data.moisture < dryThresholdOn) {
    if (!pumpRunning) {
      setPump(true);
      wateringCycleActive = true;
      wateringCountToday++;
      lastSimHour = st.hour;
      Serial.printf("[PUMP] CRITICAL DRY: Immediate ON (moist=%.1f%%) cycle#%d\n",
                    data.moisture, wateringCountToday);
      sendSMS("ALERT: Soil critically dry (" + String((int)data.moisture) + "%). Pump ON now.");
    }
    return;
  }

  // ---- TIER 2: MODERATE (MOISTURE_LOW to MOISTURE_HIGH) ----
  // Hysteresis: schedule fires between LOW+HYSTERESIS and HIGH-HYSTERESIS
  // so a reading of 25.1% doesn't trigger a scheduled run then immediately bail
  float schedLow  = MOISTURE_LOW  + MOISTURE_HYSTERESIS; // 30%
  float schedHigh = MOISTURE_HIGH - MOISTURE_HYSTERESIS; // 45%

  if (data.moisture >= schedLow && data.moisture <= schedHigh) {
    bool inSchedule = false;
    for (int i = 0; i < 4; i++) {
      if (st.hour == wateringHours[i] && lastSimHour != st.hour) {
        inSchedule = true; break;
      }
    }
    if (inSchedule && !pumpRunning) {
      setPump(true);
      wateringCycleActive = true;
      wateringCountToday++;
      lastSimHour = st.hour;
      Serial.printf("[PUMP] SCHEDULED (moist=%.1f%%) cycle#%d at hour %d\n",
                    data.moisture, wateringCountToday, st.hour);
    } else if (inSchedule) {
      lastSimHour = st.hour; // consume hour even if pump already running
    }
    return;
  }

  // ---- TIER 3: WET (above MOISTURE_HIGH - HYSTERESIS) ----
  if (data.moisture > (MOISTURE_HIGH - MOISTURE_HYSTERESIS)) {
    if (pumpRunning) {
      setPump(false);
      wateringCycleActive = false;
      Serial.printf("[PUMP] WET SOIL: Pump OFF — moisture %.1f%%\n", data.moisture);
    } else {
      bool inSchedule = false;
      for (int i = 0; i < 4; i++) {
        if (st.hour == wateringHours[i] && lastSimHour != st.hour) {
          inSchedule = true; break;
        }
      }
      if (inSchedule) {
        lastSimHour = st.hour;
        Serial.printf("[PUMP] WET SOIL: Skipping schedule (moist=%.1f%%)\n", data.moisture);
      }
    }
    return;
  }
}

// =====================
//   SETUP
// =====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Starting Smart Irrigation...");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  setPump(false);

  dht.begin();
  Wire.begin();

  // OLED
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    displayAvailable = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 10); display.println("Smart Irrigation");
    display.setCursor(35, 28); display.println("Booting...");
    // Draw empty signal bars placeholder
    drawSignalBars(112, 0, 0);
    display.display();
    Serial.println("[BOOT] OLED OK");
  } else {
    logError("OLED", "Not found at 0x3C");
  }

  // RTC
  if (rtc.begin()) {
    rtcAvailable = true;
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("[BOOT] RTC OK");
  } else {
    logError("RTC", "DS3231 not found");
  }

  // SIM900A
  sim900.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(2000);
  simAvailable = initSIM900();
  if (simAvailable) {
    checkSignal();
    delay(500);
    sendSMS("Smart Irrigation Online. Send REPORT for status.", true);
    Serial.println("[BOOT] SIM900A OK");
  } else {
    logError("SIM900A", "Init failed");
  }

  simStartMillis  = millis();
  lastPageSwitch  = millis();
  lastSignalCheck = millis();
  lastSensorRead  = millis();

  // Seed lastKnownData so report is never blank
  lastKnownData = readSensors();

  Serial.println("[BOOT] Done. Running.");
}

// =====================
//   LOOP
// =====================
void loop() {
  unsigned long now = millis();

  // ---- Sensor Read ----
  if (now - lastSensorRead >= SENSOR_READ_DELAY) {
    lastSensorRead = now;
    SensorData data = readSensors();
    lastKnownData = data;

    updateRainStats(data);
    handleWateringCycle(data);
    updateOLED(data);

    Serial.printf("[DATA] T=%.1fC H=%.1f%% M=%.1f%% Rain=%s Pump=%s\n",
      data.temperature, data.humidity, data.moisture,
      data.isRaining ? "YES" : "No",
      pumpRunning    ? "ON"  : "OFF");
  }

  // ---- Signal Check ----
  if (now - lastSignalCheck >= SIGNAL_CHECK_INTERVAL) {
    lastSignalCheck = now;
    checkSignal();
  }

  // ---- Non-blocking SMS state machine ----
  processSMS();

  // ---- Incoming Call / SMS Commands ----
  handleIncomingCall();

  // ---- Safety: 60s hard pump cutoff ----
  if (pumpRunning && !manualOverride && (now - pumpOnTime > 60000)) {
    setPump(false);
    logError("PUMP", "Safety cutoff >60s");
    sendSMS("WARNING: Pump safety cutoff! >60s limit hit.", true);
  }

  delay(20); // Short delay — keeps serial buffer responsive
}
