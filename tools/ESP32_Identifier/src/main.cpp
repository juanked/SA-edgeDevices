/*
    Get unique identifier (chip MAC address) of a TTGO development board
*/

#include <Arduino.h>

void print_mac(const unsigned char *mac);
void macTest();

void setup()
{
  Serial.begin(115200);
  delay(500);
  macTest();
}

void loop()
{

}

void print_mac(const unsigned char *mac)
{
  printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void macTest()
{
  unsigned char mac_base[6] = {0};
  esp_efuse_mac_get_default(mac_base);
  esp_read_mac(mac_base, ESP_MAC_WIFI_STA);
  printf("MAC Address: ");
  print_mac(mac_base);
}
