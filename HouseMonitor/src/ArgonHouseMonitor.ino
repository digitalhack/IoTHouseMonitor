/*
 * Project PhotonBME280Pub
 * Description:
 * Author:
 * Date:
 */
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_Sensor.h"
#include "Particle.h"
#include "clickButton.h"

// SYSTEM_MODE(SEMI_AUTOMATIC);
// SYSTEM_MODE(AUTOMATIC);

#define LOG_TO_SERIAL
#define PUBLISH
//#define NO_BME280

#ifdef LOG_TO_SERIAL
SerialLogHandler logHandler(LOG_LEVEL_INFO);
#endif

typedef enum {
  START_STATE,
  CONNECT_TO_CLOUD,
  CONNECTING_TO_CLOUD,
  CONNECTED_TO_CLOUD
} state_t;

bool bme_not_found = 0;

void connectToCloud();
void connectingToCloud();
void connectedToCloud();

state_t currentState = START_STATE;

int setPubInt(String new_interval);

void getData();
void publishData();

void statusToDisplay(int x, int y, const char* msg);
void dataToDisplay();

// Sensor Data

float signal_strength;
float signal_quality;
bool power_on;
float battery_charge;
float temp;
float humid;
float press;

char json_data[100];

// timing

unsigned long nextDisplay;
unsigned long nextPublish;
int dispInt = 2000;
int pubInt = 1000 * 60 * 5;  // Default pubInt 5 minutes

// bool connectToCloud = true;

#ifndef NO_BME280
Adafruit_BME280 bme;  // I2C
#endif

Adafruit_SSD1306 display(128, 32, 4);

#define BUTTONS_SINGLE 1
#define BUTTONS_DOUBLE 2
#define BUTTONS_TRIPLE 3

#define BUTTONS_SINGLE_LONG -1
#define BUTTONS_DOUBLE_LONG -2
#define BUTTONS_TRIPLE_LONG -3

typedef enum { SETUP, PROCESS, PRINT_STATUS } buttons_t;

ClickButton buttonA(5, LOW, CLICKBTN_PULLUP);
ClickButton buttonB(4, LOW, CLICKBTN_PULLUP);
ClickButton buttonC(3, LOW, CLICKBTN_PULLUP);
ClickButton buttonD(2, LOW, CLICKBTN_PULLUP);

int* buttonControl(buttons_t cmd);

void setup() {
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  statusToDisplay(0, 10, "Starting...");

  buttonControl(SETUP);

  Particle.function("setPubInt", setPubInt);
  Particle.variable("pubInt", &pubInt, INT);

  pinMode(PWR, INPUT);

#ifndef NO_BME280
  if (!bme.begin(0x76)) {
    Log.error("Could not find a valid BME280 sensor, check wiring!");
    statusToDisplay(0, 10, "BME not found");
    bme_not_found = 1;
  }

#endif
  currentState = CONNECT_TO_CLOUD;
}

void loop() {
  buttonControl(PROCESS);
  buttonControl(PRINT_STATUS);

  switch (currentState) {
    case CONNECT_TO_CLOUD:
      connectToCloud();
      break;
    case CONNECTING_TO_CLOUD:
      connectingToCloud();
      break;
    case CONNECTED_TO_CLOUD:
      connectedToCloud();
      break;
      // default:
  }
}

void connectToCloud() {
#if PLATFORM_ID == PLATFORM_ARGON
  WiFi.on();
  WiFi.connect();
#elif PLATFORM_ID == PLATFORM_BORON
#error "need cellular code defined"
#endif
  statusToDisplay(0, 10, "Connecting...");
  Particle.connect();
  currentState = CONNECTING_TO_CLOUD;
}

void connectingToCloud() {
  if (Particle.connected()) {
    statusToDisplay(0, 10, "Connected!");
    currentState = CONNECTED_TO_CLOUD;
  }

  nextDisplay = millis() + 5000;
  nextPublish = nextDisplay;
}

// Log.infof("%s: %5.2f%% %5.2f*F %5.2f hPa", Time.timeStr().c_str(),
// humid, temp, press);

void connectedToCloud() {
  if (bme_not_found)
    return;

  if (millis() > nextDisplay) {
    getData();
    dataToDisplay();
  }

  if (millis() > nextPublish) {
    getData();
    publishData();
  }

  if (millis() + 60000 < nextPublish) {
  }
}

int setPubInt(String new_interval) {
  int temp = strtol(new_interval.c_str(), nullptr, 10);
  if (temp >= 2000) {
    pubInt = temp;
    nextPublish = pubInt + millis();
#ifdef LOG_TO_SERIAL
    Log.info("pubInt set to %d", pubInt);
#endif
    return 1;
  }
#ifdef LOG_TO_SERIAL
  Log.info("pubInt set failed");
#endif
  return -1;
}

void getData() {
  // Get Signal Data

#if PLATFORM_ID == PLATFORM_ARGON
  WiFiSignal sig = WiFi.RSSI();
#endif

  signal_strength = sig.getStrength();
  signal_quality = sig.getQuality();

  // Check external power

  power_on = digitalRead(PWR);

  // Get Battery Info

#if PLATFORM_ID == PLATFORM_BORON
  FuelGauge fuel;
  battery_charge = fuel.getSoC();
#else
  battery_charge = 100.0;
#endif

  // Get environmental data

#ifdef NO_BME280
  temp = 60.00;
  humid = 35.00;
  press = 1000.00;
#else
  temp = (bme.readTemperature() * 1.8) + 32;
  humid = bme.readHumidity();
  press = bme.readPressure() / 100.0F;
#endif
}

void statusToDisplay(int x, int y, const char* msg) {
  display.clearDisplay();
  display.setCursor(x, y);
  display.print(msg);
  display.display();
  delay(1000);
#ifdef LOG_TO_SERIAL
  Log.info(msg);
#endif
}

void dataToDisplay() {
  char buf[32];
  nextDisplay = millis() + dispInt;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.print("Online");
  display.setCursor(0, 10);
  snprintf(buf, sizeof(buf), "S%3.0f%% Q%3.0f%% B%3.0f%%", signal_strength,
           signal_quality, battery_charge);
  display.print(buf);
  display.setCursor(0, 20);
  snprintf(buf, sizeof(buf), "T%3.0fF H%3.0f%% P%6.1f", temp, humid, press);
  display.print(buf);
  display.display();
}

void publishData() {
  nextPublish = millis() + pubInt;

  snprintf(json_data, sizeof(json_data),
           "{\"t\":\"%4.2f\",\"h\":\"%4.2f\","
           "\"p\":\"%4.2f\",\"u\":\"%1d\",\"s\":\"%4.1f\","
           "\"q\":\"%4.1f\",\"b\":\"%4.1f\"}",
           temp, humid, press, power_on, signal_strength, signal_quality,
           battery_charge);

#ifdef LOG_TO_SERIAL
  Log.info(json_data);
#endif

#ifdef PUBLISH
  Particle.publish("e", json_data, PRIVATE);
#endif
}

int* buttonControl(buttons_t cmd) {
  static int buttonsOut[4];

  if (cmd == SETUP) {
    buttonsOut[0] = 0;
    buttonsOut[1] = 0;
    buttonsOut[2] = 0;
    buttonsOut[3] = 0;

    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);
    pinMode(D4, INPUT_PULLUP);
    pinMode(D5, INPUT_PULLUP);

    buttonA.debounceTime = 20;     // Debounce timer in ms
    buttonA.multiclickTime = 250;  // Time limit for multi clicks
    buttonA.longClickTime = 1000;  // time until "held-down clicks" register

    buttonB.debounceTime = 20;     // Debounce timer in ms
    buttonB.multiclickTime = 250;  // Time limit for multi clicks
    buttonB.longClickTime = 1000;  // time until "held-down clicks" register

    buttonC.debounceTime = 20;     // Debounce timer in ms
    buttonC.multiclickTime = 250;  // Time limit for multi clicks
    buttonC.longClickTime = 1000;  // time until "held-down clicks" register

    buttonD.debounceTime = 20;     // Debounce timer in ms
    buttonD.multiclickTime = 250;  // Time limit for multi clicks
    buttonD.longClickTime = 1000;  // time until "held-down clicks" register

  } else if (cmd == PROCESS) {
    buttonA.Update();
    if (buttonA.clicks != 0)
      buttonsOut[0] = buttonA.clicks;

    buttonB.Update();
    if (buttonB.clicks != 0)
      buttonsOut[1] = buttonB.clicks;

    buttonC.Update();
    if (buttonC.clicks != 0)
      buttonsOut[2] = buttonC.clicks;

    buttonD.Update();
    if (buttonD.clicks != 0)
      buttonsOut[3] = buttonD.clicks;

  } else if (cmd == PRINT_STATUS) {
    for (int i = 0; i < 4; i++) {
      if (buttonsOut[i] == 1)
        Log.info("button: %d: SINGLE click", i);
      if (buttonsOut[i] == 2)
        Log.info("button: %d: DOUBLE click", i);
      if (buttonsOut[i] == 3)
        Log.info("button: %d: TRIPLE click", i);
      if (buttonsOut[i] == -1)
        Log.info("button: %d: SINGLE LONG click", i);
      if (buttonsOut[i] == -2)
        Log.info("button: %d: DOUBLE LONG click", i);
      if (buttonsOut[i] == -3)
        Log.info("button: %d: TRIPLE LONG click", i);
      buttonsOut[i] = 0;
    }
  }
  return buttonsOut;
}
