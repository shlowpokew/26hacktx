// UNO R4 WiFi + DRV8833: one motor, one two-second run per reset.
// D5 -> IN1, D6 -> IN2, D8 -> EEP (nSLEEP).
// Motor -> OUT1/OUT2. Arduino GND -> driver GND.
// Supply the driver separately at a voltage suitable for the motor/module.

constexpr int IN1_PIN = 5;
constexpr int IN2_PIN = 6;
constexpr int SLEEP_PIN = 8;
constexpr int MOTOR_PWM = 64;  // 64/255 = about 25% duty; not regulated RPM.
constexpr unsigned long RUN_MS = 2000;

void setup() {
  // Establish the coast/stop state before enabling the driver.
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);
  pinMode(SLEEP_PIN, OUTPUT);
  analogWriteResolution(8);

  Serial.begin(115200);
  // Do not wait for a serial monitor: this also works with USB power alone.
  delay(3000);
  Serial.println("Motor running for 2 seconds at 25% PWM.");
  analogWrite(IN1_PIN, MOTOR_PWM);
  delay(RUN_MS);

  analogWrite(IN1_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  // Both inputs LOW disables drive (coast); the shaft may coast briefly.
  Serial.println("Motor drive stopped. Press RESET to repeat.");
}

void loop() {
  // Intentionally empty: do not repeat the test automatically.
}
