// Full-power forward motor calibration.
// USB Serial at 115200 baud: f = full power forward, d/x = brake.

constexpr int LEFT_IN1 = 25;
constexpr int LEFT_IN2 = 26;
constexpr int RIGHT_IN1 = 27;
constexpr int RIGHT_IN2 = 14;
constexpr int MOTOR_EN = 23;
constexpr int FULL_PWM = 255;

void brakeMotor(int in1, int in2) {
  analogWrite(in1, 255);
  analogWrite(in2, 255);
}

void driveMotorForward(int in1, int in2, bool reversePolarity) {
  analogWrite(in1, reversePolarity ? 0 : FULL_PWM);
  analogWrite(in2, reversePolarity ? FULL_PWM : 0);
}

void brakeBoth() {
  brakeMotor(LEFT_IN1, LEFT_IN2);
  brakeMotor(RIGHT_IN1, RIGHT_IN2);
}

void fullPowerForward() {
  // The right motor has reversed electrical polarity because of its mounting.
  driveMotorForward(LEFT_IN1, LEFT_IN2, false);
  driveMotorForward(RIGHT_IN1, RIGHT_IN2, true);
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  brakeBoth();
  digitalWrite(MOTOR_EN, HIGH);

  Serial.println("Full-power forward motor calibration ready.");
  Serial.println("f: both motors forward at raw PWM 255");
  Serial.println("d or x: brake both motors");
  Serial.println("The robot remains stopped after upload/reset until f is received.");
}

void loop() {
  if (!Serial.available()) return;

  const char command = Serial.read();
  if (command == 'f' || command == 'F') {
    fullPowerForward();
    Serial.println("FULL POWER FORWARD: L/R raw PWM = 255/255");
  } else if (command == 'd' || command == 'D' ||
             command == 'x' || command == 'X') {
    brakeBoth();
    Serial.println("BRAKE");
  }
}
