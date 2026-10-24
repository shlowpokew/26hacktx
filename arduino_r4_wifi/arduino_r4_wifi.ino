#include <WiFiS3.h>

const char* WIFI_SSID = "your-wifi-name";
const char* WIFI_PASSWORD = "your-wifi-password";
const char* ROS_HOST_IP = "192.168.1.100";
const unsigned int ROS_PORT = 4211;
const unsigned int ARDUINO_PORT = 4210;

WiFiUDP udp;
String line;

void sendMessage(const String& message) {
  udp.beginPacket(ROS_HOST_IP, ROS_PORT);
  udp.print(message);
  udp.print('\n');
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  while (WiFi.begin(WIFI_SSID, WIFI_PASSWORD) != WL_CONNECTED) {
    delay(5000);
  }
  udp.begin(ARDUINO_PORT);
  sendMessage("{\"type\":\"status\",\"state\":\"online\"}");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize == 0) {
    delay(5);
    return;
  }

  line = "";
  while (udp.available()) {
    char character = static_cast<char>(udp.read());
    if (character != '\n' && character != '\r') {
      line += character;
    }
  }

  if (line.indexOf("\"type\":\"ping\"") >= 0) {
    sendMessage("{\"type\":\"pong\"}");
  } else if (line.indexOf("\"type\":\"command\"") >= 0) {
    // Replace this acknowledgement with motor/sensor handling as hardware is added.
    sendMessage("{\"type\":\"ack\",\"message\":\"command received\"}");
  } else {
    sendMessage("{\"type\":\"error\",\"message\":\"unknown message type\"}");
  }
}
