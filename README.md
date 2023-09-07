Arduino firmware for [this project](https://github.com/felixcreate/Air-Quality-Monitor)

#### Libraries used:

- [MQTT](https://www.arduino.cc/reference/en/libraries/arduinomqttclient/)
- [ArduinoJson](https://arduinojson.org/)
- [WiFiNINA](https://www.arduino.cc/reference/en/libraries/wifinina/)
- [CustomJWT](https://github.com/Ant2000/CustomJWT)
- [SAMD_TimerInterrupt](https://github.com/khoih-prog/SAMD_TimerInterrupt)
- [ArduinoStreamUtils](https://github.com/bblanchon/ArduinoStreamUtils)
- [Sensirion sen5x](https://github.com/Sensirion/arduino-i2c-sen5x)
- [Sensirion scd4x](https://github.com/Sensirion/arduino-i2c-scd4x)
- [DS3231](https://www.arduino.cc/reference/en/libraries/ds3231/)
- [PMS](https://www.arduino.cc/reference/en/libraries/pms-library/)
- [Adafruit BME280](https://www.arduino.cc/reference/en/libraries/adafruit-bme280-library/)
- [SD](https://www.arduino.cc/reference/en/libraries/sd/)
- [NTP](https://www.arduino.cc/reference/en/libraries/ntpclient/)



To use this you must create a "secrets.h" in main/ for your network's SSID and password, and also your MQTT broker address and JWT secret:

```
#define NETWORK_SSID "SSID"
#define NETWORK_PASS "PASS"
#define MQTT_BROKER IPAddress(0, 0, 0, 0)
#define JWT_SECRET ""
```
<br>

This was only tested on an Arduino Nano 33 IoT, although it should work on any Arduino with a SAMD processor and an wifi/ethernet module or shield that's compatible with the WiFiNINA library.
