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
int  oledFailCount     = 0;
unsigned long lastOledReinit = 0;
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
unsigned long smsCooldownUntil = 0;
String   smsPending     = "";

// =====================
//   TUNING PARAMETERS
// =====================
#define PHONE_NUMBER          "+918088765085"
#define MOISTURE_DRY          4095
#define MOISTURE_WET          1500
#define MOISTURE_LOW          25
#define MOISTURE_HIGH         50
#define MOISTURE_HYSTERESIS   5
#define PUMP_COOLDOWN         30000
#define RAIN_PER_MINUTE       0.5
#define PUMP_ON_DURATION      20000
#define SENSOR_READ_DELAY     2000
#define SIGNAL_CHECK_INTERVAL 10000
#define PAGE_INTERVAL         4000
#define DHT_READ_ATTEMPTS     3
#define SMS_THROTTLE_TIME     15000

// =====================
//   STRUCTS
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
unsigned long rainStopTime = 0;    // millis when rain stopped
bool rainActive            = false; // currently raining
#define RAIN_RESUME_DELAY  25000   // 25s after rain stops before pump can resume
int wateringHours[4]      = {0, 6, 12, 18};

// =====================
//   PUMP CONTROL
// =====================
bool pumpRunning    = false;
bool manualOverride = false;       // stays true until AUTO command received
unsigned long pumpOnTime      = 0;
unsigned long pumpLastOffTime = 0;

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
int dryReadCount = 0; // consecutive dry reads before pump triggers

// =====================
//   SERIAL PORT OWNERSHIP
// =====================
String simBuffer = "";
bool   simBusy   = false;

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
    pumpLastOffTime = millis();
  }
  Serial.printf("[PUMP] %s moisture=%.1f pumpOnTime=%lu now=%lu duration=%lu\n",
    state ? "ON" : "OFF", lastKnownData.moisture, pumpOnTime, millis(), millis() - pumpOnTime);
}

// =====================
//   HELPER: SMS Throttle
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
//   OLED: Draw Signal Bars
// =====================
void drawSignalBars(int x, int y, int bars) {
  int barW       = 3;
  int gap        = 1;
  int heights[4] = {4, 6, 8, 10};
  int baseline   = y + 10;
  for (int i = 0; i < 4; i++) {
    int bx = x + i * (barW + gap);
    int bh = heights[i];
    int by = baseline - bh;
    if (i < bars) {
      display.fillRect(bx, by, barW, bh, SSD1306_WHITE);
    } else {
      display.drawRect(bx, by, barW, bh, SSD1306_WHITE);
    }
  }
}

// =====================
//   OLED: Draw Moisture Bar
// =====================
void drawMoistureBar(int y, float pct) {
  int barX = 0, barW = 128, barH = 6;
  display.drawRect(barX, y, barW, barH, SSD1306_WHITE);
  int fill = (int)((pct / 100.0f) * (barW - 2));
  if (fill > 0) display.fillRect(barX + 1, y + 1, fill, barH - 2, SSD1306_WHITE);
}

// =====================
//   OLED: Status Bar
// =====================
void drawStatusBar(int page) {
  if (pumpRunning) {
    display.fillCircle(3, 3, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(3, 3, 3, SSD1306_WHITE);
  }
  display.setTextSize(1);
  display.setCursor(9, 0);
  display.print(manualOverride ? "M" : "A");

  int dotX = 56;
  for (int i = 0; i < 4; i++) {
    if (i == page) display.fillCircle(dotX + i * 6, 3, 2, SSD1306_WHITE);
    else           display.drawCircle(dotX + i * 6, 3, 2, SSD1306_WHITE);
  }

  drawSignalBars(112, 0, signalBars);
  display.drawFastHLine(0, 8, 128, SSD1306_WHITE);
}

// =====================
//   OLED: Update Display
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

    case 1:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("SOIL & RAIN");
      display.setTextSize(2);
      display.setCursor(0, 22);
      display.printf("%d%%", (int)data.moisture);
      display.setTextSize(1);
      display.setCursor(52, 24);
      if      (data.moisture < MOISTURE_LOW)   display.print("[DRY!]");
      else if (data.moisture <= MOISTURE_HIGH) display.print("[SCHED]");
      else                                     display.print("[WET] ");
      drawMoistureBar(38, data.moisture);
      display.setCursor(0, 49);
      display.printf("R:%s D:%.1f W:%.1f M:%.1fmm",
        data.isRaining ? "YES" : "No ", rainDaily, rainWeekly, rainMonthly);
      break;

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

    case 3:
      display.setTextSize(1);
      display.setCursor(0, 11);
      display.println("NETWORK & TIME");
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

  display.display();

  if (millis() - lastOledReinit > 5000) {
    lastOledReinit = millis();
    Wire.beginTransmission(OLED_ADDRESS);
    byte err = Wire.endTransmission();
    if (err != 0) {
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
      oledFailCount = 0;
    }
  }
}

// =====================
//   SIM900A: Send AT Command
// =====================
String sendATCommand(String cmd, int timeout) {
  simBusy   = true;
  simBuffer = "";
  delay(50);
  while (sim900.available()) sim900.read();
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
    delay(5);
  }
  response.trim();
  simBusy = false;
  if (response.length() > 0)
    Serial.printf("[SIM] CMD:%s | RSP:%s\n", cmd.c_str(), response.c_str());
  return response;
}

// =====================
//   SIM900A: Init
// =====================
bool initSIM900() {
  Serial.println("[SIM] Initializing SIM900A...");
  for (int i = 0; i < 5; i++) {
    String rsp = sendATCommand("AT", 1000);
    if (rsp.indexOf("OK") != -1) {
      sendATCommand("ATE0",              500);
      sendATCommand("AT+CMGF=1",         500);
      sendATCommand("AT+CNMI=2,2,0,0,0", 500);
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
//   SIM900A: Send SMS (enqueue)
// =====================
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
//   NON-BLOCKING SMS STATE MACHINE
// =====================
void processSMS() {
  if (!simAvailable) return;

  unsigned long now = millis();

  if (smsState == SMS_IDLE && smsQueueCount == 0) return;
  if (smsState == SMS_IDLE && now < smsCooldownUntil) return;

  if (smsState == SMS_IDLE && smsQueueCount > 0) {
    if (signalBars < 1) return;
    smsPending   = smsQueue[smsQueueHead];
    smsQueueHead = (smsQueueHead + 1) % SMS_QUEUE_SIZE;
    smsQueueCount--;
    smsState      = SMS_SET_MODE;
    smsStateStart = now;
    simBusy       = true;
    while (sim900.available()) sim900.read();
    sim900.println("AT+CMGF=1");
    Serial.println("[SMS] State: SET_MODE -> " + smsPending.substring(0, 30));
    return;
  }

  switch (smsState) {

    case SMS_SET_MODE: {
      String r = "";
      while (sim900.available()) r += (char)sim900.read();
      if (r.indexOf("OK") != -1) {
        smsState      = SMS_SEND_CMD;
        smsStateStart = now;
        sim900.println("AT+CMGS=\"" + String(PHONE_NUMBER) + "\"");
        Serial.println("[SMS] State: SEND_CMD");
      } else if (r.indexOf("ERROR") != -1) {
        Serial.println("[SMS] SET_MODE ERROR — retrying in 3s");
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 3000;
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
        sim900.write(0x1A);
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
        Serial.println("[SMS] Sent: " + smsPending.substring(0, 40));
        failedSMSCount   = 0;
        lastSMSTime      = now;
        smsState         = SMS_IDLE;
        simBusy          = false;
        smsCooldownUntil = now + 4000;
      } else if (r.indexOf("ERROR") != -1) {
        Serial.println("[SMS] Error on confirm");
        failedSMSCount++;
        smsState = SMS_IDLE; simBusy = false;
        smsCooldownUntil = now + 3000;
      } else if (now - smsStateStart > 8000) {
        Serial.println("[SMS] Assumed sent (no echo)");
        failedSMSCount   = 0;
        lastSMSTime      = now;
        smsState         = SMS_IDLE;
        simBusy          = false;
        smsCooldownUntil = now + 5000;
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
  if (simBusy) return;
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
//   STATUS REPORT
// =====================
void sendStatusReport() {
  SimTime st = getSimTime();
  String msg = "SOIL:" + String((int)lastKnownData.moisture) + "%";
  if      (lastKnownData.moisture < MOISTURE_LOW)   msg += "(DRY) ";
  else if (lastKnownData.moisture <= MOISTURE_HIGH) msg += "(SCHED) ";
  else                                              msg += "(WET) ";
  msg += "RAIN:" + String(lastKnownData.isRaining ? "YES" : "NO") + " ";
  msg += "T:" + String((int)lastKnownData.temperature) + "C H:" + String((int)lastKnownData.humidity) + "% ";
  msg += "PUMP:" + String(pumpRunning ? "ON" : "OFF") + " ";
  msg += "MODE:" + String(manualOverride ? "MANUAL" : "AUTO") + " ";
  msg += "CYCLES:" + String(wateringCountToday) + " ";
  msg += "RAIN(D/W/M):" + String(rainDaily, 1) + "/" + String(rainWeekly, 1) + "/" + String(rainMonthly, 1) + "mm";
  if (msg.length() > 160) msg = msg.substring(0, 160);
  Serial.println("[SMS] Status report length: " + String(msg.length()) + " chars");
  sendSMS(msg, true);
}

// =====================
//   PARSE INCOMING SMS
// =====================
void parseIncomingSMS(String text) {
  text.trim();
  text.toUpperCase();
  Serial.println("[SMS] Cmd: " + text);

  if (text.indexOf("RELAY ON") != -1) {
    manualOverride = true;
    setPump(true);
    sendSMS("OK: Pump ON (Manual mode)", true);
  }
  else if (text.indexOf("RELAY OFF") != -1) {
    manualOverride = true;
    setPump(false);
    sendSMS("OK: Pump OFF (Manual mode)", true);
  }
  else if (text.indexOf("AUTO") != -1) {
    manualOverride = false;
    setPump(false);
    sendSMS("OK: AUTO mode. Pump follows soil moisture.", true);
  }
  else if (text.indexOf("REPORT") != -1) {
    sendStatusReport();
  }
  else {
    sendSMS("RELAY ON / RELAY OFF / AUTO / REPORT", true);
  }
}

// =====================
//   PROCESS SIM LINE
// =====================
void processSimLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.println("[SIM RX] " + line);

  if (line.indexOf("RING") != -1) {
    unsigned long now = millis();
    if (now - lastRingTime > 3000) {
      lastRingTime = now;
      Serial.println("[CALL] Ring detected -> ATH + report");
      delay(600);
      sendATCommand("ATH", 1000);
      delay(500);
      sendStatusReport();
    }
    return;
  }

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

// =====================
//   HANDLE INCOMING CALL/SMS
// =====================
void handleIncomingCall() {
  if (simBusy) return;
  while (sim900.available()) {
    char c = (char)sim900.read();
    simBuffer += c;
  }

  int nlIdx;
  while ((nlIdx = simBuffer.indexOf('\n')) != -1) {
    String line = simBuffer.substring(0, nlIdx);
    simBuffer   = simBuffer.substring(nlIdx + 1);
    processSimLine(line);
  }

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
// =====================
void handleWateringCycle(SensorData data) {
  Serial.printf("[WATER] moisture=%.1f manualOverride=%d pumpRunning=%d\n",
    data.moisture, manualOverride, pumpRunning);

  // Manual mode — do nothing, wait for AUTO command via SMS
  if (manualOverride) return;

  SimTime st = getSimTime();

  // Reset daily watering count on new sim day
  if (st.day != lastWateringSimDay) {
    lastWateringSimDay = st.day;
    wateringCountToday = 0;
  }

  // Rain handling — turn pump off when raining, wait 25s after rain stops
  if (data.isRaining) {
    rainActive = true;
    rainStopTime = millis(); // keep resetting while still raining
    if (pumpRunning) {
      setPump(false);
      Serial.println("[PUMP] Rain detected — pump OFF");
      sendSMS("RAIN: Pump OFF. Resumes 25s after rain stops.", false);
    }
    return;
  } else if (rainActive) {
    // Rain just stopped — start 25s countdown
    if (millis() - rainStopTime < RAIN_RESUME_DELAY) {
      Serial.printf("[RAIN] Waiting %lus before resuming...\n", (RAIN_RESUME_DELAY - (millis() - rainStopTime)) / 1000);
      return;
    } else {
      rainActive = false;
      Serial.println("[RAIN] Resume delay done — auto watering restored");
      sendSMS("RAIN stopped. Auto watering resumed.", false);
    }
  }

  // Turn ON only below 25% (dry) — 3 consecutive dry reads to prevent flicker
  if (data.moisture < MOISTURE_LOW && !pumpRunning) {
    dryReadCount++;
    if (dryReadCount >= 3) {
      setPump(true);
      wateringCountToday++;
      dryReadCount = 0;
      Serial.printf("[AUTO] Soil dry (%.1f%%) -> PUMP ON\n", data.moisture);
      sendSMS("AUTO: Soil dry (" + String((int)data.moisture) + "%). Pump ON.", false);
    } else {
      Serial.printf("[AUTO] Dry read %d/3 — waiting\n", dryReadCount);
    }
  }
  // Turn OFF only above 55% (50% + 5% hysteresis)
  else if (data.moisture > (MOISTURE_HIGH + MOISTURE_HYSTERESIS) && pumpRunning) {
    dryReadCount = 0;
    setPump(false);
    Serial.printf("[AUTO] Soil wet (%.1f%%) -> PUMP OFF\n", data.moisture);
    sendSMS("AUTO: Soil wet (" + String((int)data.moisture) + "%). Pump OFF.", false);
  }
  else {
    dryReadCount = 0;
  }
}

// =====================
//   SETUP
// =====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Starting Smart Irrigation...");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // force relay OFF immediately at boot
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

  lastKnownData = readSensors();
  pumpLastOffTime = millis() - PUMP_COOLDOWN - 1; // allow immediate start after boot
  pumpOnTime = millis(); // prevent false safety cutoff on first loop

  Serial.println("[BOOT] Done. Running.");
}

// =====================
//   LOOP
// =====================
void loop() {
  unsigned long now = millis();

  // Sensor Read
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

  // Signal Check
  if (now - lastSignalCheck >= SIGNAL_CHECK_INTERVAL) {
    lastSignalCheck = now;
    checkSignal();
  }

  // Non-blocking SMS state machine
  processSMS();

  // Incoming Call / SMS
  handleIncomingCall();

  // Safety: 60s hard pump cutoff — DISABLED FOR TESTING
  // if (pumpRunning && !manualOverride && (now - pumpOnTime > 60000)) {
  //   setPump(false);
  //   logError("PUMP", "Safety cutoff >60s");
  //   sendSMS("WARNING: Pump safety cutoff! >60s limit hit.", true);
  // }

  delay(20);
}
