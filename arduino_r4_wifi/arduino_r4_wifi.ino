#include <WiFiS3.h>

const char* WIFI_SSID = "your-wifi-name";
const char* WIFI_PASSWORD = "your-wifi-password";
const char* ROS_HOST_IP = "192.168.1.100";
const unsigned int ROS_PORT = 4211;
const unsigned int ARDUINO_PORT = 4210;

WiFiUDP udp;
String line;

class Drv8833Motor {
 public:
  Drv8833Motor(uint8_t input1Pin, uint8_t input2Pin, uint8_t sleepPin)
      : input1Pin_(input1Pin),
        input2Pin_(input2Pin),
        sleepPin_(sleepPin) {}

  void begin() {
    pinMode(input1Pin_, OUTPUT);
    pinMode(input2Pin_, OUTPUT);
    pinMode(sleepPin_, OUTPUT);
    digitalWrite(sleepPin_, HIGH);
    stop();
  }

  // Speed ranges from -255 (reverse) to 255 (forward).
  void setSpeed(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
      analogWrite(input1Pin_, speed);
      digitalWrite(input2Pin_, LOW);
    } else if (speed < 0) {
      digitalWrite(input1Pin_, LOW);
      analogWrite(input2Pin_, -speed);
    } else {
      stop();
    }
  }

  void stop() {
    digitalWrite(input1Pin_, LOW);
    digitalWrite(input2Pin_, LOW);
  }

  void brake() {
    digitalWrite(input1Pin_, HIGH);
    digitalWrite(input2Pin_, HIGH);
  }

 private:
  uint8_t input1Pin_;
  uint8_t input2Pin_;
  uint8_t sleepPin_;
};

// NEED TO FIGURE OUT ports on the motor driver
Drv8833Motor testMotor(5, 6, 8);

void sendMessage(const String& message) {
  udp.beginPacket(ROS_HOST_IP, ROS_PORT);
  udp.print(message);
  udp.print('\n');
  udp.endPacket();
}

void sendStatus(const String& state) {
  String message = "{\"type\":\"motor_status\",\"state\":\"";
  message += state;
  message += "\"}";
  sendMessage(message);
  Serial.println(message);
}

void handleCommand(const String& command) {
  if (command == "motor stop") {
    testMotor.stop();
    sendStatus("stopped");
  } else if (command == "motor brake") {
    testMotor.brake();
    sendStatus("braking");
  } else if (command.startsWith("motor forward ")) {
    int speed = command.substring(14).toInt();
    testMotor.setSpeed(speed);
    sendStatus("forward");
  } else if (command.startsWith("motor reverse ")) {
    int speed = command.substring(14).toInt();
    testMotor.setSpeed(-speed);
    sendStatus("reverse");
  } else {
    sendStatus("unknown_command");
  }
}

void setup() {
  Serial.begin(115200);
  testMotor.begin();
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
    int dataStart = line.indexOf("\"data\":\"");
    if (dataStart >= 0) {
      dataStart += 8;
      int dataEnd = line.indexOf('"', dataStart);
      if (dataEnd > dataStart) {
        handleCommand(line.substring(dataStart, dataEnd));
      }
    }
  } else {
    sendMessage("{\"type\":\"error\",\"message\":\"unknown message type\"}");
  }
}
