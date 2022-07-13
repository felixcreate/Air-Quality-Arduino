#include "secrets.h"

#include <WiFiNINA.h>
#include <WiFiUdp.h>

//NTP globals
WiFiUDP NTP;
unsigned int localPort = 2390;
IPAddress timeServer(162, 159, 200, 123);
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
//NTP globals end


void setup() {
  Serial.begin(9600);
  Serial.println("Starting");
  if(connectWifi(5000)) {
    Serial.println("Connected");
  }
  else {
    Serial.println("Failed");
  }
  NTP.begin(localPort);
}

void loop() {
  long epoch = getNTPepoch(5000);
  if(epoch == 0) {
    Serial.println("Request failed");
  }
  else {
    Serial.println(epoch);
  }
}

//Connect to wifi or return false if timeout expires
bool connectWifi(int timeout) {
  long start = millis();
  long lastrun = millis();
  int Status = WiFi.begin(ssid, pass);
  while(Status != WL_CONNECTED) {
    if((millis() - lastrun) > 500) {
      Status = WiFi.begin(ssid, pass);
      lastrun = millis();
    }
    if((millis() - start) > timeout){
      return false;
    }
  }
  return true;
}

//Helper function
unsigned long sendNTPpacket(IPAddress& address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  NTP.beginPacket(address, 123);
  NTP.write(packetBuffer, NTP_PACKET_SIZE);
  NTP.endPacket();
}

//Get epoch from ntp server or return 0 if request times out
long getNTPepoch(int timeout) {
  sendNTPpacket(timeServer);
  long start = millis();
  long lastrun = millis();
  int packetsize = NTP.parsePacket();
  while(!packetsize) {
    if((millis() - lastrun) > 500) {
      packetsize = NTP.parsePacket();
      lastrun = millis();
    }
    if((millis() - start) > timeout){
      return 0;
    }
  }
  NTP.read(packetBuffer, NTP_PACKET_SIZE);
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  return secsSince1900 - 2208988800;
}
