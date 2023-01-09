#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <xxtea-lib.h>
#include <ArduinoJson.h>

#define SCK 5   // GPIO5  -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define SS 18   // GPIO18 -- SX1278's CS
#define RST 14  // GPIO14 -- SX1278's RESET
#define DI0 26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6
#define VALVE_PIN 12

#define mS_TO_S_FACTOR 1000
#define TIME_TO_WATER 3600

String key = F("smartAgriculture");
StaticJsonDocument<512> doc;

char *uid;
String message;
String packSize;
int watering;
unsigned long temp = 0;

char *getUID();
void cbk(int packetSize);
bool checkUID(String message);
void getMessage();
void water();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  pinMode(VALVE_PIN, OUTPUT);

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

  LoRa.receive();
  Serial.println("init ok");
  doc.clear();
}

void loop()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    cbk(packetSize);
  }
  if (!checkUID(message))
  {
    delay(10);
    return;
  }

  getMessage();
  water();

  delay(1000);
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

void cbk(int packetSize)
{
  message = "";
  char flag = '!';
  bool condition = false;
  packSize = String(packetSize, DEC);
  for (int i = 0; i < packetSize; i++)
  {
    if (condition || ((char)LoRa.read() == flag))
    {
      message += (char)LoRa.read();
      condition = true;
    }
  }
}

bool checkUID(String message)
{
  int index = message.indexOf("#");
  if (index != -1)
  {
    String receivedUID = message.substring(0, index);
    int strLen = receivedUID.length() + 1;
    char rUID[strLen];
    receivedUID.toCharArray(rUID, strLen);
    if (strcmp(rUID, uid) == 0)
    {
      Serial.println("uid coincide");
      return true;
    }
  }
  return false;
}

void getMessage()
{
  int index = message.indexOf("#");
  if (index != -1)
  {
    String receivedMessage = message.substring(index + 1);
    Serial.print("receivedMessage: ");
    Serial.println(receivedMessage);
    int lastIndex = receivedMessage.length() - 1;
    receivedMessage.remove(lastIndex);
    Serial.print("receivedMessage removed: ");
    Serial.println(receivedMessage);
    String decrypt = xxtea.decrypt(receivedMessage);
    int index = decrypt.indexOf("}");
    Serial.print("decrypt: ");
    Serial.println(decrypt);
    String substring = "";
    if (index != -1)
    {
      substring = decrypt.substring(0, index + 1);
      Serial.print("sin ␄: ");
      Serial.println(substring);
    }
    deserializeJson(doc, substring);
    watering = doc["water"].as<String>().toInt();
    Serial.print("bool water: ");
    Serial.println(watering);
  }
}

void water()
{
  unsigned long tempLocal = millis();
  if (watering && (temp > (tempLocal + TIME_TO_WATER * mS_TO_S_FACTOR)))
  {
    watering = 0;
  }

  if (watering)
  {
    Serial.println("Start Watering");
    digitalWrite(VALVE_PIN, HIGH);
    temp = millis();
  }
  else
  {
    Serial.println("Stop Watering");
    digitalWrite(VALVE_PIN, LOW);
  }
}