#include <Arduino.h>
#include "defines.h"
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include <TinyGPS++.h>
#include <axp20x.h>
#include <ArduinoJson.h>
#include <xxtea-lib.h>
#include <Adafruit_AHTX0.h>

// ESP32 deep-sleep defines
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define mS_TO_S_FACTOR 1000    /* Conversion factor for mili seconds to seconds */
#define TIME_TO_SLEEP 600       /* Time ESP32 will go to sleep (in seconds) */

char *uid;
char *message;
SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet;

RTC_DATA_ATTR double locationLat;
RTC_DATA_ATTR double locationLng;
RTC_DATA_ATTR double locationAltitude;
RTC_DATA_ATTR int locationSatellites;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR TinyGPSPlus gps;

HardwareSerial GPS(1);
AXP20X_Class axp;
Adafruit_AHTX0 aht;
sensors_event_t humidityAir, tempAir;
int soilMoisture;
int leafMoisture;

String key = F("smartAgriculture");

char *getUID();
char *getMACPwd();
char *setSensorsMessage();
char *setMessage(char *cryptMessage);
void startAXP();
void startGPS();
void stopGPS();
void startAirSensor();
void measure();
int sendLoRaPacket(char *message);
static void smartDelay(unsigned long ms);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.printf("Hello, counter = %i\n", bootCount);
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  uid = getUID();

  xxtea.setKey(key);
  startAXP();
  if (bootCount % 4 == 0)
  {
    startGPS();
    smartDelay(60 * mS_TO_S_FACTOR);
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
    stopGPS();
  }
  startAirSensor();

  measure();
  char *cryptMessage = setSensorsMessage();
  Serial.println(cryptMessage);
  char *message = setMessage(cryptMessage);
  for (int i = 0; i < 2; i++){
    while (!sendLoRaPacket(message))
      ;
    delay(3000);
  }
  bootCount++;
  Serial.println("Going to sleep...");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop()
{
}

char *getUID()
{
  unsigned char mac[6] = {0};
  esp_efuse_mac_get_default(mac);
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char uid[3];
  sprintf(uid, "%02X:%02X:%02X", mac[3], mac[4], mac[5]);
  return strdup(uid);
}

char *setSensorsMessage()
{
  char message[255];
  bool batteryConnect = axp.isBatteryConnect();
  float batteryVoltage = axp.getBattVoltage() / 1000;
  float airTemperature = tempAir.temperature;
  float airHumidity = humidityAir.relative_humidity;

  sprintf(message, "%i;%.2f;%.5f;%.5f;%.2f;%i;%.2f;%.2f;%i;%i", batteryConnect, batteryVoltage,
          locationLat, locationLng, locationAltitude, locationSatellites, airTemperature, airHumidity,
          soilMoisture, leafMoisture);

  Serial.println(message);
  String crypt = xxtea.encrypt(message);
  int strLen = crypt.length() + 1;
  char cryptMessage[strLen];
  crypt.toCharArray(cryptMessage, strLen);
  return strdup(cryptMessage);
}

char *setMessage(char *cryptMessage)
{
  char message[strlen(uid) + strlen(cryptMessage) + strlen("#") + 1];
  strcpy(message, uid);
  strcat(message, "#");
  strcat(message, cryptMessage);
  Serial.println(message);
  return strdup(message);
}

void startAXP()
{
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
  {
    Serial.println("AXP192 Begin PASS");
  }
  else
  {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LoRa
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON); // Unknown
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON); // Unknown
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED
}

void startGPS()
{
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS
  GPS.begin(9600, SERIAL_8N1, 34, 12);
  Serial.println("GPS started");
}
void stopGPS()
{
  axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
  Serial.println("GPS stopped");
}

void startAirSensor()
{
  if (!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
    while (1)
      delay(100);
  }
  Serial.println("AHT10 or AHT20 found");
}

void measure()
{
  aht.getEvent(&humidityAir, &tempAir);
  soilMoisture = analogRead(AOUT_PIN);
  leafMoisture = analogRead(AOUT_PIN);
  // TODO: actualizar al recibir el sensor
}

int sendLoRaPacket(char *message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  return LoRa.endPacket();
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
  locationLat = gps.location.lat();
  locationLng = gps.location.lng();
  locationAltitude = gps.altitude.meters();
  locationSatellites = gps.satellites.value();
}