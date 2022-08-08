#include <ArduinoJson.h>

#include "secrets.h"

#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_JSON.h>
#include <ArduinoMqttClient.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08JWS.h>
#include <DS3231.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <I2S.h>


//NTP globals
WiFiUDP NTP;
unsigned int localPort = 2390;
IPAddress timeServer(162, 159, 200, 123);
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
//NTP globals end


//RTC globals
DS3231 RTCWrite;
RTClib RTCRead;
//RTC globals end


//MQTT globals
const char broker[] = "mqtt.googleapis.com";
WiFiSSLClient wifiSslClient;
MqttClient    mqttClient(wifiSslClient);
//MQTT globals end


//I2S globals
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 256;
const double samplingFrequency = 8000;
double vReal[samples];
double vImag[samples];
//I2S globals end

void setup() {
  Serial.begin(9600);

  Wire.begin();
  I2S.begin(I2S_PHILIPS_MODE, 8000, 32);

  mqttClient.setId(calculateClientId());
  
  Serial.println("Starting");
  if(connectWifi(5000)) {
    Serial.println("Connected");
  }
  else {
    Serial.println("Failed");
  }
  NTP.begin(localPort);
  unsigned long epoch = getNTPepoch(5000);
  if(epoch == 0) {
    Serial.println("Request failed");
  }
  else {
    Serial.println(epoch);
    RTCWrite.setEpoch(epoch);
  }
  double test[] = {45.6, 34.5, 23, 12.6};
  Serial.println(generateJSON(6545645, 3434534, test, test, test, 5, 1, 6, 34.3, 89));
}

void loop() {
  //int offset = calculateI2SOffset();
  //Serial.println(calculateFreq(offset));
}

//Connect to wifi or return false if timeout expires
bool connectWifi(int timeout) {
  unsigned long start = millis();
  unsigned long lastrun = millis();
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
unsigned long getNTPepoch(int timeout) {
  sendNTPpacket(timeServer);
  unsigned long start = millis();
  unsigned long lastrun = millis();
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

String calculateClientId() {
  String clientId;

  // Format:
  //
  //   projects/{project-id}/locations/{cloud-region}/registries/{registry-id}/devices/{device-id}
  //

  clientId += "projects/";
  clientId += PROJECT_ID;
  clientId += "/locations/";
  clientId += CLOUD_REGION;
  clientId += "/registries/";
  clientId += REGISTRY_ID;
  clientId += "/devices/";
  clientId += DEVICE_ID;

  return clientId;
}

unsigned long getRTCEpoch() {
  return RTCRead.now().unixtime();
}

String calculateJWT() {
  unsigned long now = getRTCEpoch();
  
  // calculate the JWT, based on:
  //   https://cloud.google.com/iot/docs/how-tos/credentials/jwts
  JSONVar jwtHeader;
  JSONVar jwtClaim;

  jwtHeader["alg"] = "ES256";
  jwtHeader["typ"] = "JWT";

  jwtClaim["aud"] = PROJECT_ID;
  jwtClaim["iat"] = now;
  jwtClaim["exp"] = now + (24L * 60L * 60L); // expires in 24 hours 

  return ECCX08JWS.sign(0, JSON.stringify(jwtHeader), JSON.stringify(jwtClaim));
}

bool connectMQTT(int timeout) {
  unsigned long start = millis();
  unsigned long lastrun = millis();
  while(!mqttClient.connected()) {
    if((millis() - lastrun) > 1000) {
      String jwt = calculateJWT();
      mqttClient.setUsernamePassword("unused", jwt);
      mqttClient.connect(broker, 8883);
    }
    if((millis() - start) > timeout) {
      return false;
    }
  }
  return true;
}

void publishMessage() {
  Serial.println("Publishing message");

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("/devices/" + DEVICE_ID + "/state");
  mqttClient.print("hello ");
  mqttClient.print(millis());
  mqttClient.endMessage();
}

int calculateI2SOffset() {
  int offset;
  for(int y = 0;y < 20;y++) {
    unsigned long start = millis();
    int readings = 0;
    int64_t runningsum = 0;
    while((millis() - start) < 100) {
      int sample = I2S.read();
      if(sample) {
        runningsum += sample;
        readings++;
      }
    }
    offset = runningsum/readings;
  }
  return offset;
}

double calculateFreq(int offset) {
  for(int i = 0; i < samples;i) {
    int sample = I2S.read();
    if(sample) {
      vReal[i] = sample - offset;
      vImag[i] = 0.0;
      i++;
    }
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  return FFT.MajorPeak(vReal, samples, samplingFrequency);
}

String generateJSON(unsigned long timeStamp, unsigned long start, double temperature[4], double humidity[4], double pressure[4], int pm1, int pm25, int pm10, double spl, int frequency) {
  StaticJsonDocument<500> doc;
  doc["timestamp"] = timeStamp;
  doc["start"] = start;

  JsonObject Data = doc.createNestedObject("data");
  
  JsonArray Temp = Data.createNestedArray("temperature");
  Temp.add(temperature[0]);
  Temp.add(temperature[1]);
  Temp.add(temperature[2]);
  Temp.add(temperature[3]);

  JsonArray Humidity = Data.createNestedArray("humidity");
  Humidity.add(humidity[0]);
  Humidity.add(humidity[1]);
  Humidity.add(humidity[2]);
  Humidity.add(humidity[3]);

  JsonArray Pressure = Data.createNestedArray("pressure");
  Pressure.add(pressure[0]);
  Pressure.add(pressure[1]);
  Pressure.add(pressure[2]);
  Pressure.add(pressure[3]);

  Data["spl"] = spl;
  Data["frequency"] = frequency;

  JsonObject Particulate = Data.createNestedObject("particulate");
  Particulate["pm1.0"] = pm1;
  Particulate["pm2.5"] = pm25;
  Particulate["pm10.0"] = pm10;

  String out = "";
  serializeJson(doc, out);
  return out;
}
