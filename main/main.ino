#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USING_TIMER_TC3         false
#define USING_TIMER_TC4         false
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        true
#define USING_TIMER_TCC2        false    

#define status_pin 9

//#include <ArduinoECCX08.h>
//#include <utility/ECCX08JWS.h>
#include <CustomJWT.h>
#include <ArduinoMqttClient.h>
//#include <Arduino_JSON.h>
#include <WiFiNINA.h>
#include <DS3231.h>
#include <SPI.h>
#include <SD.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PMS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <SAMDTimerInterrupt.h>
#include <StreamUtils.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSen5x.h>
#include "secrets.h"

//Network constants
const char ssid[]        = NETWORK_SSID;
const char pass[]        = NETWORK_PASS;
//const char broker[] = "felixfb.org";
const IPAddress broker = MQTT_BROKER;

//JWT constants
//char key[] = {0xcf, 0x45, 0xd8, 0xc1, 0xd5, 0x4e, 0xa4, 0x3c, 0x6e, 0x89, 0x25, 0xc4, 0x45, 0x40, 0x62, 0xd3, 0x20, 0x08, 0x75, 0xf7, 0xff, 0xb9, 0xf4, 0xb1, 0xe6, 0x39, 0x8c, 0x40, 0xcd, 0x5d, 0x7d, 0x60, 0x27, 0x32, 0x47, 0x43, 0xb9, 0x33, 0xaf, 0x70, 0x0f, 0x71, 0xec, 0xcf, 0x34, 0x83, 0x18, 0xdf, 0xe5, 0x8c, 0xa2, 0xef, 0x2a, 0xa8, 0x07, 0x09, 0x59, 0x6b, 0xcd, 0x57, 0x6f, 0x86, 0xbe, 0xe2};
char key[] = {JWT_SECRET};
char internalJWTheader[50];
char internalJWTpayload[150];
char internalJWTsignature[50];
char internalJWTout[300];
CustomJWT jwt(key, internalJWTheader, sizeof(internalJWTheader), internalJWTpayload, sizeof(internalJWTpayload), internalJWTsignature, sizeof(internalJWTsignature), internalJWTout, sizeof(internalJWTout));

//PMS globals
PMS pms(Serial1);
PMS::DATA data;

//Sensirion globals
SensirionI2CScd4x scd40;
SensirionI2CSen5x sen55;
uint32_t lastSen55Cleaning;

//BME globals
Adafruit_BME280 bme;

//NTP globals
WiFiUDP NTPUdp;
NTPClient timeClient(NTPUdp, "pool.ntp.org", 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

//RTC globals
DS3231 myRTC;

//MQTT globals
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//Timer globals
SAMDTimer ITimer(TIMER_TCC1);
//volatile uint8_t minuteOverFlow = 0;
volatile uint8_t readingTriggers = 0;

//Other globals
uint32_t startupEpoch;
//uint16_t unsentCounter = 0;
uint8_t minuteCounter = 0;
//File openFile; 


//Reading and data variables
uint32_t readingStartEdge;

double temps[] = {-1.0, -1.0, -1.0, -1.0};
double humidities[] = {-1.0, -1.0, -1.0, -1.0};
double pressures[] = {-1.0, -1.0, -1.0, -1.0};
double pm1s[] = {-1.0, -1.0, -1.0, -1.0};
double pm25s[] = {-1.0, -1.0, -1.0, -1.0};
double pm10s[] = {-1.0, -1.0, -1.0, -1.0};
double scdTemps[] = {-1.0, -1.0, -1.0, -1.0};
double scdHumidities[] = {-1.0, -1.0, -1.0, -1.0};
double co2s[] = {-1.0, -1.0, -1.0, -1.0};
double senpm1s[] = {-1.0, -1.0, -1.0, -1.0};
double senpm25s[] = {-1.0, -1.0, -1.0, -1.0};
double senpm4s[] = {-1.0, -1.0, -1.0, -1.0};
double senpm10s[] = {-1.0, -1.0, -1.0, -1.0};
double senTemps[] = {-1.0, -1.0, -1.0, -1.0};
double senHumidities[] = {-1.0, -1.0, -1.0, -1.0};
double vocs[] = {-1.0, -1.0, -1.0, -1.0};
double noxs[] = {-1.0, -1.0, -1.0, -1.0};

uint8_t tempReadingCounter = 0;
uint8_t humidityReadingCounter = 0;
uint8_t pressureReadingCounter = 0;
uint8_t pm1ReadingCounter = 0;
uint8_t pm25ReadingCounter = 0;
uint8_t pm10ReadingCounter = 0;
uint8_t scdTempReadingCounter = 0;
uint8_t scdHumidityReadingCounter = 0;
uint8_t co2ReadingCounter = 0;
uint8_t senpm1ReadingCounter = 0;
uint8_t senpm25ReadingCounter = 0;
uint8_t senpm4ReadingCounter = 0;
uint8_t senpm10ReadingCounter = 0;
uint8_t senTempReadingCounter = 0;
uint8_t senHumidityReadingCounter = 0;
uint8_t vocReadingCounter = 0;
uint8_t noxReadingCounter = 0;

double tempAccumulator = 0;
double humidityAccumulator = 0;
double pressureAccumulator = 0;
uint32_t pm1Accumulator = 0;
uint32_t pm25Accumulator = 0;
uint32_t pm10Accumulator = 0;
double scdTempAccumulator = 0;
double scdHumidityAccumulator = 0;
uint32_t co2Accumulator = 0;
double senpm1Accumulator = 0;
double senpm25Accumulator = 0;
double senpm4Accumulator = 0;
double senpm10Accumulator = 0;
double senTempAccumulator = 0;
double senHumidityAccumulator = 0;
double vocAccumulator = 0;
double noxAccumulator = 0;

//debug
unsigned long start;

// led state
int led_timer = 0;
bool led_state = false;

void Timer_Handler() {
  //TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  //minuteOverFlow++;
  //if(minuteOverFlow == 60) {
    readingTriggers++;
    //minuteOverFlow = 0;
  //}
}

void setup() {
  randomSeed(analogRead(A2)); //seed the pseudo random number generator
  pinMode(status_pin, OUTPUT);

  Serial.begin(9600); //Debug
  Serial1.begin(9600); // Start communication with PMS sensor
  pms.passiveMode();
  while (!Serial); //
  Serial.println("start");

  for(int i=0;i<600;i++) { // 6 second to let all sensors finish initializing and to let status led show that the program has started
    int part = i % 200; // 0 - 199
    if(part < 100) {
      analogWrite(status_pin, (part*255)/100);
    }
    else {
      analogWrite(status_pin, 255 - (((part - 100)*255)/100));
    }
    delay(10);
  }
  analogWrite(status_pin, 255);
  led_state = true;
  //if (!ECCX08.begin()) {  //Should come first since RTC is can't handle the I2C clock speed this sets (shouldn't be an problem with all the redundancy for this issue)
  //  while(1);
  //}
  pms.requestRead();
  delay(2000);
  if(!pms.readUntil(data, 10000)) {
    Serial.println("pms no");
    while(1) {
      analogWrite(status_pin, 0);
      delay(100);
      analogWrite(status_pin, 255);
      delay(100);
    }
  }

  Wire.begin();
  Wire.setClock(100000u);

  Wire.beginTransmission(0x68);
  if(Wire.endTransmission() != 0) {
    while(1) {
      analogWrite(status_pin, 0);
      delay(100);
      analogWrite(status_pin, 255);
      delay(100);
    }
  }
  myRTC.setClockMode(false); // 24 hour clock mode, not that it really matters for this
  //myRTC.enableOscillator(true, false, 0);

  if(!bme.begin(0x77, &Wire)) { // Check for and setup BME280 
    while(1) {
      analogWrite(status_pin, 0);
      delay(100);
      analogWrite(status_pin, 255);
      delay(100);
    }
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED, //configure BME280 for weather measuring
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  
  scd40.begin(Wire);
  if(scd40.stopPeriodicMeasurement()) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }}
  scd40.setSensorAltitude(195);
  if(scd40.startPeriodicMeasurement()) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }}

  sen55.begin(Wire);
  if(sen55.deviceReset()) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }}
  delay(1000);
  if(sen55.setFanAutoCleaningInterval(0)) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }}
  if(sen55.startMeasurement()) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }}
  if(!SD.begin(4)) {while(1) {
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }} // Start SD
  //setRTCEpoch(1674536397UL);   //debug
  //bme.takeForcedMeasurement();
  //Serial.println(data.PM_SP_UG_2_5);
  //Serial.println(bme.readTemperature());
  //Serial.println(getRTCEpoch());
  //Serial.println(calculateJWT(getRTCEpoch()));
  //Serial.println(getRTCEpoch());
  WiFi.setDNS(primaryDNS, secondaryDNS);
  mqttClient.setId("hamhigh1");
  mqttClient.setConnectionTimeout(5000); // 10000
  WiFi.setTimeout(5000);
  timeClient.setUpdateInterval(0);
  timeClient.begin();
  Serial.println("Connecting to wifi");
  while(!connectWiFi(3, 500)) {
    Serial.println("No wifi");
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }
  Serial.println("Getting ntp");
  while(!getNTPTime(5, 100)) {
    Serial.println("No ntp");
    analogWrite(status_pin, 0);
    delay(100);
    analogWrite(status_pin, 255);
    delay(100);
  }
  Serial.println("Network ok");
  setRTCEpoch(timeClient.getEpochTime());
  startupEpoch = timeClient.getEpochTime();
  lastSen55Cleaning = timeClient.getEpochTime();
  
  Serial.println("Sending startup message");
  if(connectMQTT(timeClient.getEpochTime())) {
    if(mqttClient.beginMessage("airquality/hamhigh1/startup", 1, false, 1)) {
      mqttClient.write('_');
      if(mqttClient.endMessage()) {
        Serial.println("Message sent");
      }
    }
    mqttClient.stop();
  }

  //File unsentFile = SD.open("unsent");
  //if(unsentFile) {
  //  while(unsentFile.available()) {
  //    if(unsentFile.read() == ';') {
  //      unsentCounter++;
  //    }
  //  }
  //  unsentFile.close();
  //}
  //else {
  //  while(1);
  //}
  for(int i=0;i<6000;i++) { // 60 seconds for sensor to start getting good readings
    int part = i % 200; // 0 - 199
    if(part < 100) {
      analogWrite(status_pin, (part*255)/100);
    }
    else {
      analogWrite(status_pin, 255 - (((part - 100)*255)/100));
    }
    delay(10);
  }
  analogWrite(status_pin, 0);
  led_state = false;
  //Timer configure and start
  //attachInterrupt(digitalPinToInterrupt(3), Timer_Handler, FALLING);
  ITimer.attachInterruptInterval_MS(30000, Timer_Handler); //60 seconds per trigger
  //start = millis(); //debug
  //https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
  /*GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC5->COUNT16.CTRLA.bit.SWRST);

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;

  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / 1024);
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);*/

  readingStartEdge = timeClient.getEpochTime();
  //digitalWrite(LED_BUILTIN, HIGH);


  Serial.println(timeClient.getFormattedTime()); // debug
  Serial.println(getRTCEpoch());
  Serial.println(readingStartEdge);
  //Serial.println(unsentCounter);
  bme.takeForcedMeasurement();
  //Serial.println(data.PM_SP_UG_2_5);
  Serial.println(bme.readTemperature());
  bool isScdReady;
    if(!scd40.getDataReadyFlag(isScdReady)) {
      if(isScdReady) {
        float scdtemptmp;
        float scdhumiditytmp;
        uint16_t co2tmp;
        if(!scd40.readMeasurement(co2tmp, scdtemptmp, scdhumiditytmp)) {
          if(co2tmp != 0) {
            Serial.println(co2tmp); //debug
          }
        }
      }
    }
  bool isSenReady;
    if(!sen55.readDataReady(isSenReady)) {
      if(isSenReady) {
        float pm1tmp;
        float pm25tmp;
        float pm4tmp;
        float pm10tmp;
        float senhumiditytmp;
        float sentemptmp;
        float voctmp;
        float noxtmp;
        if(!sen55.readMeasuredValues(pm1tmp, pm25tmp, pm4tmp, pm10tmp, senhumiditytmp, sentemptmp, voctmp, noxtmp)) {
          Serial.println(pm25tmp);
        }
      }
    }
  //generateJWT(getRTCEpoch());
  //Serial.println(jwt.out);
  Serial.println(getRTCEpoch());
  double test[] = {45.6, 34.5, 23, 12.6};
  double testi[] = {4, 3, 23, 126};
  //generateJSON(500, 1000, test, test, test, testi, testi, testi);
  //serializeJson(jsonDocument, Serial);
  //jsonDocument.clear();
  double test1[] = {78.6, 12.5, 56.2, 56.3};
  double testi1[] = {41, 23, 83, 5};
  //generateJSON(890, 2376, test1, test1, test1, testi1, testi1, testi1);
  //serializeJson(jsonDocument, Serial);
  //jsonDocument.clear();
  printMemory();

  // set mqtt client id hamhigh1   DONE
  // set temp sensor config    DONE
  // connect to wifi    DONE
  // get ntp time    DONE
  // don't continue until ntp time has been retrieved   DONE
  // set startup time with ntp    DONE
  // set rtc clock with ntp time (utc)    DONE
  // read unsent messages file and set unsent readings counter   DONE
  // wait 60 seconds for sensors to settle   led slow blink  DONE
  // set starting edge time of reading with ntp time   DONE
  // led set high
}

void loop() {
  if((readingTriggers == 1) || (readingTriggers == 2)) { //Trigger 1 for normal, 2 for delayed
    analogWrite(status_pin, 255);
    delay(250);
    analogWrite(status_pin, 0);
    minuteCounter += readingTriggers;
    bool run15minute = false;
    if((minuteCounter % 15) == 0) {
      run15minute = true;
    }
    else if((minuteCounter % 15) == 1) {
      if(readingTriggers == 2) {
        run15minute = true;
      }
    }
    readingTriggers = 0;
    Serial.println("1"); //debug
    //Serial.println(millis() - start);
    collectData();

    if(run15minute) {
      Serial.println("15"); //debug
      uint8_t index = ((uint8_t) minuteCounter/15) - 1;
      if(tempReadingCounter != 0) temps[index] = tempAccumulator / tempReadingCounter;
      if(humidityReadingCounter != 0) humidities[index] = humidityAccumulator / humidityReadingCounter;
      if(pressureReadingCounter != 0) pressures[index] = pressureAccumulator / pressureReadingCounter;
      if(pm1ReadingCounter != 0) pm1s[index] = ((double) pm1Accumulator) / pm1ReadingCounter;
      if(pm25ReadingCounter != 0) pm25s[index] = ((double) pm25Accumulator) / pm25ReadingCounter;
      if(pm10ReadingCounter != 0) pm10s[index] = ((double) pm10Accumulator) / pm10ReadingCounter;
      if(scdTempReadingCounter != 0) scdTemps[index] = scdTempAccumulator / scdTempReadingCounter;
      if(scdHumidityReadingCounter != 0) scdHumidities[index] = scdHumidityAccumulator / scdHumidityReadingCounter;
      if(co2ReadingCounter != 0) co2s[index] = ((double) co2Accumulator) / co2ReadingCounter;
      if(senpm1ReadingCounter != 0) senpm1s[index] = senpm1Accumulator / senpm1ReadingCounter;
      if(senpm25ReadingCounter != 0) senpm25s[index] = senpm25Accumulator / senpm25ReadingCounter;
      if(senpm4ReadingCounter != 0) senpm4s[index] = senpm4Accumulator / senpm4ReadingCounter;
      if(senpm10ReadingCounter != 0) senpm10s[index] = senpm10Accumulator / senpm10ReadingCounter;
      if(senTempReadingCounter != 0) senTemps[index] = senTempAccumulator / senTempReadingCounter;
      if(senHumidityReadingCounter != 0) senHumidities[index] = senHumidityAccumulator / senHumidityReadingCounter;
      if(vocReadingCounter != 0) vocs[index] = vocAccumulator / vocReadingCounter;
      if(noxReadingCounter != 0) noxs[index] = noxAccumulator / noxReadingCounter;

      tempReadingCounter = humidityReadingCounter = pressureReadingCounter = pm1ReadingCounter = pm25ReadingCounter = pm10ReadingCounter = scdTempReadingCounter = scdHumidityReadingCounter = co2ReadingCounter = senpm1ReadingCounter = senpm25ReadingCounter = senpm4ReadingCounter = senpm10ReadingCounter = senTempReadingCounter = senHumidityReadingCounter = vocReadingCounter = noxReadingCounter = tempAccumulator = humidityAccumulator = pressureAccumulator = pm1Accumulator = pm25Accumulator = pm10Accumulator = scdTempAccumulator = scdHumidityAccumulator = co2Accumulator = senpm1Accumulator = senpm25Accumulator = senpm4Accumulator = senpm10Accumulator = senTempAccumulator = senHumidityAccumulator = vocAccumulator = noxAccumulator = 0;
    }
    if(minuteCounter > 59) {
      minuteCounter -= 60;
      Serial.println("60"); //debug
      printMemory(); // debug
      uint32_t endEdgeTime = getRTCEpoch();
      if((endEdgeTime - lastSen55Cleaning) > 604800) {
        sen55.startFanCleaning();
        lastSen55Cleaning = endEdgeTime;
      }
      printMemory(); // debug

      StaticJsonDocument<1536> jsonDocument;
      jsonDocument["s"] = readingStartEdge;
      jsonDocument["e"] = endEdgeTime;
  
      JsonArray Temp = jsonDocument.createNestedArray("t");
      Temp.add(temps[0]);
      Temp.add(temps[1]);
      Temp.add(temps[2]);
      Temp.add(temps[3]);

      JsonArray Humidity = jsonDocument.createNestedArray("h");
      Humidity.add(humidities[0]);
      Humidity.add(humidities[1]);
      Humidity.add(humidities[2]);
      Humidity.add(humidities[3]);

      JsonArray Pressure = jsonDocument.createNestedArray("p");
      Pressure.add(pressures[0]);
      Pressure.add(pressures[1]);
      Pressure.add(pressures[2]);
      Pressure.add(pressures[3]);

      JsonArray Part1 = jsonDocument.createNestedArray("p1");
      Part1.add(pm1s[0]);
      Part1.add(pm1s[1]);
      Part1.add(pm1s[2]);
      Part1.add(pm1s[3]);

      JsonArray Part25 = jsonDocument.createNestedArray("p25");
      Part25.add(pm25s[0]);
      Part25.add(pm25s[1]);
      Part25.add(pm25s[2]);
      Part25.add(pm25s[3]);

      JsonArray Part10 = jsonDocument.createNestedArray("p10");
      Part10.add(pm10s[0]);
      Part10.add(pm10s[1]);
      Part10.add(pm10s[2]);
      Part10.add(pm10s[3]);

      JsonArray ScdTemp = jsonDocument.createNestedArray("scdt");
      ScdTemp.add(scdTemps[0]);
      ScdTemp.add(scdTemps[1]);
      ScdTemp.add(scdTemps[2]);
      ScdTemp.add(scdTemps[3]);

      JsonArray ScdHumidity = jsonDocument.createNestedArray("scdh");
      ScdHumidity.add(scdHumidities[0]);
      ScdHumidity.add(scdHumidities[1]);
      ScdHumidity.add(scdHumidities[2]);
      ScdHumidity.add(scdHumidities[3]);

      JsonArray Co2 = jsonDocument.createNestedArray("co2");
      Co2.add(co2s[0]);
      Co2.add(co2s[1]);
      Co2.add(co2s[2]);
      Co2.add(co2s[3]);

      JsonArray SenPm1 = jsonDocument.createNestedArray("senpm1");
      SenPm1.add(senpm1s[0]);
      SenPm1.add(senpm1s[1]);
      SenPm1.add(senpm1s[2]);
      SenPm1.add(senpm1s[3]);

      JsonArray SenPm25 = jsonDocument.createNestedArray("senpm25");
      SenPm25.add(senpm25s[0]);
      SenPm25.add(senpm25s[1]);
      SenPm25.add(senpm25s[2]);
      SenPm25.add(senpm25s[3]);

      JsonArray SenPm4 = jsonDocument.createNestedArray("senpm4");
      SenPm4.add(senpm4s[0]);
      SenPm4.add(senpm4s[1]);
      SenPm4.add(senpm4s[2]);
      SenPm4.add(senpm4s[3]);

      JsonArray SenPm10 = jsonDocument.createNestedArray("senpm10");
      SenPm10.add(senpm10s[0]);
      SenPm10.add(senpm10s[1]);
      SenPm10.add(senpm10s[2]);
      SenPm10.add(senpm10s[3]);

      JsonArray SenT = jsonDocument.createNestedArray("sent");
      SenT.add(senTemps[0]);
      SenT.add(senTemps[1]);
      SenT.add(senTemps[2]);
      SenT.add(senTemps[3]);

      JsonArray SenH = jsonDocument.createNestedArray("senh");
      SenH.add(senHumidities[0]);
      SenH.add(senHumidities[1]);
      SenH.add(senHumidities[2]);
      SenH.add(senHumidities[3]);

      JsonArray Voc = jsonDocument.createNestedArray("voc");
      Voc.add(vocs[0]);
      Voc.add(vocs[1]);
      Voc.add(vocs[2]);
      Voc.add(vocs[3]);

      JsonArray Nox = jsonDocument.createNestedArray("nox");
      Nox.add(noxs[0]);
      Nox.add(noxs[1]);
      Nox.add(noxs[2]);
      Nox.add(noxs[3]);

      printMemory(); // debug
      String readingFilename = String(endEdgeTime, 16);
      Serial.println(readingFilename); // debug
      bool stored = false;
      File readingLog = SD.open(readingFilename, O_WRITE | O_CREAT);
      if(readingLog) {
        serializeJson(jsonDocument, readingLog);
        readingLog.close();
        stored = true;
      }
      printMemory(); // debug
      bool sent = false;
      if(connectWiFi(3, 500)) { // max 16 seconds
        printMemory(); // debug
        if(getNTPTime(5, 100)) { // max 5.5 seconds
          printMemory(); // debug
          setRTCEpoch(timeClient.getEpochTime());
          if(connectMQTT(timeClient.getEpochTime())) {
            printMemory(); // debug
            if(mqttClient.beginMessage("airquality/hamhigh1/data", measureJson(jsonDocument), false, 2)) {
              printMemory(); // debug
              WriteBufferingStream bufferedMqtt{mqttClient, 128};
              printMemory(); // debug
              serializeJson(jsonDocument, bufferedMqtt);
              printMemory(); // debug
              serializeJson(jsonDocument, Serial); // debug
              printMemory(); // debug
              bufferedMqtt.flush();
              if(mqttClient.endMessage()) {
                sent = true;
                printMemory(); // debug
              }
            }
            mqttClient.stop();
          }
        }
      }
      if(!sent) {
        if(stored) {
          File unsent = SD.open("unsent", O_READ | O_WRITE | O_APPEND);
          if(unsent) {
            Serial.println("Saving as unsent"); // debug
            unsent.print(readingFilename);
            unsent.print(';');
            unsent.close();
          }
        }
      }
      Serial.println("done"); // debug
      readingStartEdge = endEdgeTime;
      for(int i=0;i<4;i++) {
        //Serial.println("Period " + (i + 1));
        //Serial.println(temps[i]);
        //Serial.println(humidities[i]);
        //Serial.println(pressures[i]);
        //Serial.println(pm1s[i]);
        //Serial.println(pm25s[i]);
        //Serial.println(pm10s[i]);
        temps[i] = -1.0;
        humidities[i] = -1.0;
        pressures[i] = -1.0;
        pm1s[i] = -1.0;
        pm25s[i] = -1.0;
        pm10s[i] = -1.0;
        scdTemps[i] = -1.0;
        scdHumidities[i] = -1.0;
        co2s[i] = -1.0;
        senpm1s[i] = -1.0;
        senpm25s[i] = -1.0;
        senpm4s[i] = -1.0;
        senpm10s[i] = -1.0;
        senTemps[i] = -1.0;
        senHumidities[i] = -1.0;
        vocs[i] = -1.0;
        noxs[i] = -1.0;
      }
    }
    if((minuteCounter > 5) && (minuteCounter < 55)) {
      Serial.println("tried");
      File unsent = SD.open("unsent");
      Serial.println("try open");
      if(unsent) {
        Serial.println("opened");
        if(unsent.size() != 0) {
          Serial.println("has stuff");
          if(WiFi.status() == WL_CONNECTED) {
            if(getNTPTime(5, 100)) {
              if(connectMQTT(timeClient.getEpochTime())) {
                char readingName[9] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
                unsent.read(readingName, 8);
                Serial.println(readingName); // debug
                File unsentReading = SD.open(readingName);
                if(unsentReading) {
                  if(mqttClient.beginMessage("airquality/hamhigh1/unsent", unsentReading.size(), false, 2)) {
                    WriteBufferingStream bufferedMqtt{mqttClient, 64};
                    while(unsentReading.available()) {
                      bufferedMqtt.write(unsentReading.read());
                    }
                    unsentReading.close();
                    bufferedMqtt.flush();
                    if(mqttClient.endMessage()) {
                      Serial.println("next");
                      File unsentTMP = SD.open("tmp", O_READ | O_WRITE | O_CREAT | O_TRUNC);
                      if(unsentTMP) {
                        Serial.println("next open");
                        unsent.seek(unsent.position() + 1);
                        while(unsent.available()) {
                          char fileBuffer[64];
                          int size = unsent.read(fileBuffer, 64);
                          unsentTMP.write(fileBuffer, size);
                          printMemory(); // debug
                        }
                        unsentTMP.close();
                        unsent.close();
                        unsentTMP = SD.open("tmp");
                        unsent = SD.open("unsent", O_TRUNC | O_WRITE);
                        while(unsentTMP.available()) {
                          char fileBuffer[64];
                          int size = unsentTMP.read(fileBuffer, 64);
                          unsent.write(fileBuffer, size);
                          printMemory(); // debug
                        }
                        unsent.close();
                        unsentTMP.close();
                        printMemory(); // debug
                      }
                    }
                  }
                  unsentReading.close();
                }
                mqttClient.stop();
              }
            }
          }
        }
        unsent.close();
      }
    }
    printMemory(); // debug
  }
  else if(readingTriggers > 2) {
    Serial.println("overtime");
    readingTriggers = 0;
    analogWrite(status_pin, 255);
    delay(250);
    analogWrite(status_pin, 0);
    Serial.println("reset");
    minuteCounter = 1;
    readingStartEdge = getRTCEpoch() - 60;
    tempReadingCounter = humidityReadingCounter = pressureReadingCounter = pm1ReadingCounter = pm25ReadingCounter = pm10ReadingCounter = scdTempReadingCounter = scdHumidityReadingCounter = co2ReadingCounter = senpm1ReadingCounter = senpm25ReadingCounter = senpm4ReadingCounter = senpm10ReadingCounter = senTempReadingCounter = senHumidityReadingCounter = vocReadingCounter = noxReadingCounter = tempAccumulator = humidityAccumulator = pressureAccumulator = pm1Accumulator = pm25Accumulator = pm10Accumulator = scdTempAccumulator = scdHumidityAccumulator = co2Accumulator = senpm1Accumulator = senpm25Accumulator = senpm4Accumulator = senpm10Accumulator = senTempAccumulator = senHumidityAccumulator = vocAccumulator = noxAccumulator = 0;
    for(int i=0;i<4;i++) {
      temps[i] = -1.0;
      humidities[i] = -1.0;
      pressures[i] = -1.0;
      pm1s[i] = -1.0;
      pm25s[i] = -1.0;
      pm10s[i] = -1.0;
      scdTemps[i] = -1.0;
      scdHumidities[i] = -1.0;
      co2s[i] = -1.0;
      senpm1s[i] = -1.0;
      senpm25s[i] = -1.0;
      senpm4s[i] = -1.0;
      senpm10s[i] = -1.0;
      senTemps[i] = -1.0;
      senHumidities[i] = -1.0;
      vocs[i] = -1.0;
      noxs[i] = -1.0;
    }
    collectData();
    Serial.println("back to normal");
  }
  //if((millis() - lastMillis) >= 5000) {
    //lastMillis = millis();
    //publishMessage();  
  //}

  // what time frame to use???? DONE 1 minute readings 15 minute averages
  // 1 minute timer for all readings (timer doesn't get paused even when sending data)
  //  every 15 times (15 minutes) collect all readings
  //  every 60 times set end edge time with rtc time, serialize, generate reference string of end edge time, save to sd card using reference string as filename, 
  //   if connected to wifi 
  //      get ntp time (try max 5 times, wait at least 100 ms between tries) (max 5.5 sec)
  //   if not connected 
  //      try connecting and then get ntp (max 21.5 sec)
  //   if failed getting ntp (no internet connection) or failed connecting to wifi 
  //      append reference string to file containing references to unsent readings and add one to unsent readings counter
  //   else if both work
  //      set rtc using the ntp time that was retrieved for the mqtt connection
  //          mqtt timing and usage:
  //              connect(): max 10 secs plus mqtt client timeout, 1 for successful 0 for failed         !!! For some reason this seems to be 28 seconds regardless of timeout value
  //              beginmessage(): max n/a, 1 for success 0 for fail
  //              print, write ...
  //              endmessage(): max 2*mqtt client timeout for qos 2, 1 for success, 0 for fail (not connected or PUBREC or PUBACK or PUBCOMP timed out)
  //              stop(): runs mqttclient disconnect and wificlient stop, max 5 secs, void
  //      send readings to airquality/hamhigh1/data   qos 2  max 10 + timeout + 2*timeout + 5    
  //      send status message with uptime current ntp - startup to airquality/hamhigh1/status    qos 0   retained     NOT DOING THIS
  //   if mqtt fails (server down) 
  //      append reference string to file containing references to unsent readings and add one to unsent readings counter
  //   set starting edge time of reading using same time as previous end edge time
  //  for all timer triggers between 5 and 55 minute times    <----
  //    if there are unsent readings
  //      if connected to wifi
  //        get ntp time (try max 5 times, wait at least 100 ms between tries)
  //        if successful
  //          try sending all unsent readings by opening one file at a time and sending it to airquality/hamhigh1/unsent   qos 2      ONLY ONE FILE PER MINUTE
  //          if successfully sent
  //            remove reference strings from not sent file
  //  clear jsondocument

}
void setRTCEpoch(uint32_t sec) {
  //Wire.setClock(400000u); // Redundancy
  myRTC.setEpoch(sec);
}
uint32_t getRTCEpoch() {
  //Wire.setClock(400000u); // Redundancy
  return RTClib::now().unixtime();
}

int connectWiFi(int8_t tries, uint32_t wait) { // with 3 tries and a wifi timeout of 5000ms this function should not take longer than 16 seconds
  if(WiFi.status() == WL_CONNECTED) {
    return 1;
  }
  int8_t timesTried = 0;
  while(WiFi.begin(ssid, pass) != WL_CONNECTED) {
    if(timesTried == (tries - 1)) break;
    delay(wait);
    timesTried++;
  }
  if(WiFi.status() == WL_CONNECTED) {
    return 1;
  }
  else {
    return 0;
  }
}

bool getNTPTime(int8_t tries, uint32_t wait) {
  int8_t timesTried = 0;
  bool state;
  do {
    state = timeClient.forceUpdate();
    timesTried++;
  }
  while(!state && (timesTried < tries));
  return state;
}

int connectMQTT(uint32_t now) {
  printMemory(); // debug
  Serial.print("Attempting to connect to MQTT broker: "); // debug
  Serial.println(" "); // debug
  printMemory(); // debug
  generateJWT(now);
  mqttClient.setUsernamePassword("unused", jwt.out);
  printMemory(); // debug

  return mqttClient.connect(broker, 1883);

  Serial.println("You're connected to the MQTT broker"); // debug
  Serial.println(); // debug
}
void generateJWT(uint32_t now) {
  StaticJsonDocument<256> jwtpayload;
  jwtpayload["aud"] = "connection-hub";
  jwtpayload["iss"] = "hamhigh1";
  jwtpayload["iat"] = now;
  jwtpayload["exp"] = now + (60L * 60L);
  jwtpayload["uid"] = random(0, 2147483647);
  char jsonpayload[100];
  serializeJson(jwtpayload, jsonpayload, 100);
  jwt.encodeJWT(jsonpayload);
}
void publishMessage() {
  Serial.println("Publishing message");

  mqttClient.beginMessage("test", false, 2, false);
  mqttClient.print("hello ");
  mqttClient.print(millis());
  mqttClient.endMessage();
}
void collectData() {
  if(bme.takeForcedMeasurement()) {
    tempAccumulator += bme.readTemperature() + 273.15;
    tempReadingCounter++;
    humidityAccumulator += bme.readHumidity();
    humidityReadingCounter++;
    pressureAccumulator += bme.readPressure();
    pressureReadingCounter++;
  }
  pms.requestRead();
  delay(1000);
  if(pms.readUntil(data, 5000)) {
    pm1Accumulator += data.PM_SP_UG_1_0;
    pm1ReadingCounter++;
    pm25Accumulator += data.PM_SP_UG_2_5;
    pm25ReadingCounter++;
    pm10Accumulator += data.PM_SP_UG_10_0;
    pm10ReadingCounter++;
  }
  bool isScdReady;
  if(!scd40.getDataReadyFlag(isScdReady)) {
    if(isScdReady) {
      float scdtemptmp;
      float scdhumiditytmp;
      uint16_t co2tmp;
      if(!scd40.readMeasurement(co2tmp, scdtemptmp, scdhumiditytmp)) {
        if(co2tmp != 0) {
          scdTempAccumulator += scdtemptmp + 273.15;
          scdTempReadingCounter++;
          scdHumidityAccumulator += scdhumiditytmp;
          scdHumidityReadingCounter++;
          co2Accumulator += co2tmp;
          co2ReadingCounter++;
          Serial.println(co2tmp); //debug
        }
      }
    }
  }
  bool isSenReady;
  if(!sen55.readDataReady(isSenReady)) {
    if(isSenReady) {
      float pm1tmp;
      float pm25tmp;
      float pm4tmp;
      float pm10tmp;
      float senhumiditytmp;
      float sentemptmp;
      float voctmp;
      float noxtmp;
      if(!sen55.readMeasuredValues(pm1tmp, pm25tmp, pm4tmp, pm10tmp, senhumiditytmp, sentemptmp, voctmp, noxtmp)) {
        senpm1Accumulator += pm1tmp;
        senpm1ReadingCounter++;
        senpm25Accumulator += pm25tmp;
        senpm25ReadingCounter++;
        senpm4Accumulator += pm4tmp;
        senpm4ReadingCounter++;
        senpm10Accumulator += pm10tmp;
        senpm10ReadingCounter++;
        senTempAccumulator += sentemptmp + 273.15;
        senTempReadingCounter++;
        senHumidityAccumulator += senhumiditytmp;
        senHumidityReadingCounter++;
        if(!isnan(voctmp)) {
          vocAccumulator += voctmp;
          vocReadingCounter++;
        }
        if(!isnan(noxtmp)) {
          noxAccumulator += noxtmp;
          noxReadingCounter++;
        }
        Serial.println(pm25tmp);
        Serial.println(noxtmp);
      }
    }
  }
}
/*void generateJSON(uint32_t startEdge, uint32_t endEdge) {
  jsonDocument["s"] = startEdge;
  jsonDocument["e"] = endEdge;
  
  JsonArray Temp = jsonDocument.createNestedArray("t");
  Temp.add(temps[0]);
  Temp.add(temps[1]);
  Temp.add(temps[2]);
  Temp.add(temps[3]);

  JsonArray Humidity = jsonDocument.createNestedArray("h");
  Humidity.add(humidities[0]);
  Humidity.add(humidities[1]);
  Humidity.add(humidities[2]);
  Humidity.add(humidities[3]);

  JsonArray Pressure = jsonDocument.createNestedArray("p");
  Pressure.add(pressures[0]);
  Pressure.add(pressures[1]);
  Pressure.add(pressures[2]);
  Pressure.add(pressures[3]);

  JsonArray Part1 = jsonDocument.createNestedArray("p1");
  Part1.add(pm1s[0]);
  Part1.add(pm1s[1]);
  Part1.add(pm1s[2]);
  Part1.add(pm1s[3]);

  JsonArray Part25 = jsonDocument.createNestedArray("p25");
  Part25.add(pm25s[0]);
  Part25.add(pm25s[1]);
  Part25.add(pm25s[2]);
  Part25.add(pm25s[3]);

  JsonArray Part10 = jsonDocument.createNestedArray("p10");
  Part10.add(pm10s[0]);
  Part10.add(pm10s[1]);
  Part10.add(pm10s[2]);
  Part10.add(pm10s[3]);

  JsonArray ScdTemp = jsonDocument.createNestedArray("scdt");
  ScdTemp.add(scdTemps[0]);
  ScdTemp.add(scdTemps[1]);
  ScdTemp.add(scdTemps[2]);
  ScdTemp.add(scdTemps[3]);

  JsonArray ScdHumidity = jsonDocument.createNestedArray("scdh");
  ScdHumidity.add(scdHumidities[0]);
  ScdHumidity.add(scdHumidities[1]);
  ScdHumidity.add(scdHumidities[2]);
  ScdHumidity.add(scdHumidities[3]);

  JsonArray Co2 = jsonDocument.createNestedArray("co2");
  Co2.add(co2s[0]);
  Co2.add(co2s[1]);
  Co2.add(co2s[2]);
  Co2.add(co2s[3]);
}*/

// debug
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void printMemory() {
  Serial.print(F("Free Memory: "));
  Serial.print(freeMemory());
  Serial.println(F("B"));
}
// debug