/*
    Get unique identifier (chip MAC address) of a TTGO development board
    Get values from Soil Moisture Sensor (3.3V, GND, VP)
    Get values from Rain Drop Sensor (3.3V, GND, 16)
*/

#include <Arduino.h>
#include "SSD1306.h"
#define AOUT_PIN 36 // ESP4 pin GPIO36 (ADC0-VP) that connects to AOUT pin of moisture sensor
#define BOUT_PIN 25 // ESP4 pin GPIO25 (ADC13)
#define ITERATIONS 10

SSD1306 display(0x3c, 21, 22);
int counter = 0;
int soilTotal, leafTotal = 0;

void print_mac(const unsigned char *mac);
void macTest();

void setup()
{
  pinMode(16, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(AOUT_PIN, INPUT);
  pinMode(BOUT_PIN, INPUT);
  pinMode(2, OUTPUT);

  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

  Serial.begin(115200);
  while (!Serial)
    ;
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Start measuring");
  display.display();
  delay(1500);
  display.clear();
  display.display();
  delay(1000);
  Serial.println("Soil Moisture & Leaf Moisture calibrator");
  Serial.println("1 value each 10s");
  Serial.println("--------------");
  macTest();
}

void loop()
{
  digitalWrite(2, HIGH);
  delay(500);
  int soilMoisture = analogRead(AOUT_PIN);
  int leafMoisture = !digitalRead(BOUT_PIN);
  digitalWrite(2, LOW);
  Serial.println(counter);
  String isWet = leafMoisture ? "true" : "false";
  Serial.printf("Soil Moisture measure = %i\n", soilMoisture);
  Serial.println("Leaf Moisture measure = " + isWet);
  Serial.println("--------------");
  soilTotal += soilMoisture;


  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  display.drawString(0, 0, "Soil: ");
  display.drawString(90, 0, String(soilMoisture));
  display.drawString(0, 20, "Leaf: ");
  display.drawString(90, 20, String(isWet));
  display.drawString(0, 40, "Counter: ");
  display.drawString(90, 40, String(counter));
  display.display();
  counter++;

  if (counter >= ITERATIONS)
  {
    counter = 0;
    Serial.printf("s:%i;l:%i\n", soilTotal, leafTotal);
    int soilAverage = soilTotal / ITERATIONS;
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, "LOOP FINISHED");

    display.display();
    delay(2000);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);

    display.drawString(0, 0, "Soil av: ");
    display.drawString(90, 0, String(soilAverage));
    display.display();
    Serial.printf("Soil average: %i\n", soilAverage);
    Serial.println("--------------");
    soilTotal = 0;
    leafTotal = 0;
  }

  delay(10000);
}

void print_mac(const unsigned char *mac)
{
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void macTest()
{
  unsigned char mac_base[6] = {0};
  esp_efuse_mac_get_default(mac_base);
  esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
  print_mac(mac_base);
}
