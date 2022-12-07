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

char *uid;
char *message;
SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet;

TinyGPSPlus gps;
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
void startGPS();
void startAirSensor();
void measure();
int sendLoRaPacket(char *message);
static void smartDelay(unsigned long ms);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

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
  startGPS();
  startAirSensor();
}

void loop()
{
  smartDelay(5000);
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  measure();
  char *cryptMessage = setSensorsMessage();
  Serial.println(cryptMessage);
  char *message = setMessage(cryptMessage);
  while (!sendLoRaPacket(message))
    ;
  delay(10000);
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
  double locationLat = gps.location.lat();
  double locationLng = gps.location.lng();
  double locationAltitude = gps.altitude.meters();
  int locationSatellites = gps.satellites.value();
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

void startGPS()
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
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  GPS.begin(9600, SERIAL_8N1, 34, 12);
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
}