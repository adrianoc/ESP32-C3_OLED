#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include "Secrets.h"


#define OLED_RESET U8X8_PIN_NONE  // Reset pin
#define OLED_SDA 5
#define OLED_SCL 6

#define DEBUG_BUTTON 9


U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA);

int width = 72;
int height = 40;
int xOffset = 30;  // = (132-w)/2
int yOffset = 12;  // = (64-h)/2


void display_informartion(float temp, float humidity) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_crox1c_tr);
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "T: %2.1f C", temp);
  u8g2.drawStr(xOffset + 0, yOffset + 25, buffer);

  snprintf(buffer, sizeof(buffer), "H: %2.1f %", humidity);
  u8g2.drawStr(xOffset + 0, yOffset + 45, buffer);

  u8g2.sendBuffer();
}

SensirionI2cSht3x sensor;

static char errorMessage[64];
static int16_t error;

void setupOTAUpdates() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setPassword("xpto");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  
}

void setup() {
    Serial.begin(921600);
    while (!Serial) {
        delay(100);
    }

    pinMode(DEBUG_BUTTON, INPUT);

    u8g2.begin();
    u8g2.setContrast(255);     // set contrast to maximum
    u8g2.setBusClock(400000);  //400kHz I2C

    setupOTAUpdates();

    Wire.begin();
    sensor.begin(Wire, SHT30_I2C_ADDR_44);

    sensor.stopMeasurement();
    delay(1);
    sensor.softReset();
    delay(100);
    uint16_t aStatusRegister = 0u;
    error = sensor.readStatusRegister(aStatusRegister);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute readStatusRegister(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    Serial.print("aStatusRegister: ");
    Serial.print(aStatusRegister);
    Serial.println();
    error = sensor.startPeriodicMeasurement(REPEATABILITY_MEDIUM,
                                            MPS_ONE_PER_SECOND);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
}

long last = 0;
void loop() {
    ArduinoOTA.handle();

    auto current = millis();
    if (current - last > 1000) {
        last = current;
        float aTemperature = 0.0;
        float aHumidity = 0.0;
        error = sensor.blockingReadMeasurement(aTemperature, aHumidity);
        if (error != NO_ERROR) {
            Serial.print("Error trying to execute blockingReadMeasurement(): ");
            errorToString(error, errorMessage, sizeof errorMessage);
            Serial.println(errorMessage);
            return;
        }

        display_informartion(aTemperature, aHumidity);

        if (digitalRead(DEBUG_BUTTON) == 0) {
            char buffer[48];
            snprintf(buffer, sizeof(buffer), "Temp: %2.1f C\tHum: %2.1f %", aTemperature, aHumidity);
            Serial.println(buffer);
        }
    }
}
