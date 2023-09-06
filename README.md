This is the arduino code for the project.

Libraries used:
- [MQTT](https://www.arduino.cc/reference/en/libraries/arduinomqttclient/)
- [JSON](https://www.arduino.cc/reference/en/libraries/arduino_json/)
- [ECCX08](https://www.arduino.cc/reference/en/libraries/arduinoeccx08/)
- [Wifi](https://www.arduino.cc/reference/en/libraries/wifinina/)
- [DS3231](https://www.arduino.cc/reference/en/libraries/ds3231/)
- [PMS5003](https://www.arduino.cc/reference/en/libraries/pms-library/)
- [Adafruit BME280](https://www.arduino.cc/reference/en/libraries/adafruit-bme280-library/)
- [SD card](https://www.arduino.cc/reference/en/libraries/sd/)
- [I2S/Microphone](https://docs.arduino.cc/learn/built-in-libraries/i2s)
- [~~Old NTP~~](https://www.arduino.cc/reference/en/libraries/ntpclient/)
- [NTP](https://github.com/arduino-libraries/WiFiNINA/blob/master/examples/WiFiUdpNtpClient/WiFiUdpNtpClient.ino)

Create a "secrets.h" in main/ for SSID name, password, MQTT broker address, and JWT secret:<br/>
```
#define NETWORK_SSID "YOUR_SSID"
#define NETWORK_PASS "PASS"
#define MQTT_BROKER IPAddress(0, 0, 0, 0)
#define JWT_SECRET ""
