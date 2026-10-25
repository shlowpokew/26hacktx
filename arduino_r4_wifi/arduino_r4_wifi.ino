#include <WiFiS3.h>

const char* WIFI_SSID = "your-wifi-name";
const char* WIFI_PASSWORD = "your-wifi-password";
const char* ROS_HOST_IP = "192.168.1.100";
const unsigned int ROS_PORT = 4211;
const unsigned int ARDUINO_PORT = 4210;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1500;

WiFiUDP udp;
String line;
unsigned long lastHeartbeatMs = 0;
bool heartbeatSeen = false;
bool heartbeatExpired = false;

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

// DRV8833 inputs: left IN1/IN2 on D5/D6, right IN1/IN2 on D9/D10.
// Both channels share nSLEEP on D8. Confirm the wiring against your board.
Drv8833Motor leftMotor(5, 6, 8);
Drv8833Motor rightMotor(9, 10, 8);

bool parseSpeed(const String& text, int& speed) {
  if (text.length() == 0) return false;
  int start = (text[0] == '-') ? 1 : 0;
  if (start == text.length()) return false;
  for (int i = start; i < text.length(); ++i) {
    if (!isDigit(text[i])) return false;
  }
  long parsed = text.toInt();
  if (parsed < -255 || parsed > 255) return false;
  speed = static_cast<int>(parsed);
  return true;
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}

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
  if (!heartbeatSeen || millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    stopMotors();
    sendStatus("no_ros_heartbeat");
    return;
  }
  if (command == "motor stop") {
    stopMotors();
    sendStatus("stopped");
  } else if (command == "motor brake") {
    leftMotor.brake();
    rightMotor.brake();
    sendStatus("braking");
  } else if (command.startsWith("motor forward ") || command.startsWith("motor reverse ")) {
    const bool reverse = command.startsWith("motor reverse ");
    int speed;
    if (!parseSpeed(command.substring(14), speed) || speed < 0) {
      sendStatus("invalid_motor_speed");
      return;
    }
    leftMotor.setSpeed(reverse ? -speed : speed);
    rightMotor.setSpeed(reverse ? -speed : speed);
    sendStatus(reverse ? "reverse" : "forward");
  } else if (command.startsWith("drive ")) {
    int separator = command.indexOf(' ', 6);
    int leftSpeed, rightSpeed;
    if (separator < 0 || !parseSpeed(command.substring(6, separator), leftSpeed) ||
        !parseSpeed(command.substring(separator + 1), rightSpeed)) {
      sendStatus("invalid_drive_command");
      return;
    }
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
    sendStatus("driving");
  } else {
    sendStatus("unknown_command");
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(8);
  leftMotor.begin();
  rightMotor.begin();
  while (WiFi.begin(WIFI_SSID, WIFI_PASSWORD) != WL_CONNECTED) {
    delay(5000);
  }
  udp.begin(ARDUINO_PORT);
  sendMessage("{\"type\":\"status\",\"state\":\"online\"}");
}

void loop() {
  if (heartbeatSeen && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    stopMotors();
    if (!heartbeatExpired) {
      sendStatus("heartbeat_lost");
      heartbeatExpired = true;
    }
  }

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
    lastHeartbeatMs = millis();
    heartbeatSeen = true;
    heartbeatExpired = false;
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
