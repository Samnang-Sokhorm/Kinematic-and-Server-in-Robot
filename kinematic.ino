#define M1_pwm 6   // Front Left
#define M1_dir 38
#define M2_pwm 4   // Front Right
#define M2_dir 34
#define M3_pwm 10  // Rear Left
#define M3_dir 26
#define M4_pwm 11  // Rear Right
#define M4_dir 28

#define WHEEL_RADIUS 0.06
#define ROBOT_RADIUS 0.35

String inString = "";  // Buffer for incoming serial data
float Vx = 0.0;
float Vy = 0.0;
float Omega = 0.0;
bool dataReceived = false; // ✅ Flag to indicate when valid data has arrived
unsigned long lastDataTime = 0;
const unsigned long timeout = 200;
void setup() {
  Serial.begin(115200);

  pinMode(M1_pwm, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M3_dir, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  pinMode(M4_dir, OUTPUT);

  // Make sure all motors are stopped at startup
  stopAllMotors();

  Serial.println("Waiting for server data...");
}

void stopAllMotors() {
  analogWrite(M1_pwm, 0);
  analogWrite(M2_pwm, 0);
  analogWrite(M3_pwm, 0);
  analogWrite(M4_pwm, 0);
}

void kinematics(float Vx, float Vy, float Omega) {
  float w1 = (1 / WHEEL_RADIUS) * (Vx - Vy - ROBOT_RADIUS * Omega);
  float w2 = (1 / WHEEL_RADIUS) * (Vx + Vy - ROBOT_RADIUS * Omega);
  float w3 = (1 / WHEEL_RADIUS) * (Vx - Vy + ROBOT_RADIUS * Omega);
  float w4 = (1 / WHEEL_RADIUS) * (Vx + Vy + ROBOT_RADIUS * Omega);

  setMotorSpeed(M1_pwm, M1_dir, w1 * 3);
  setMotorSpeed(M2_pwm, M2_dir, w2 * 3);
  setMotorSpeed(M3_pwm, M3_dir, w3 * 3);
  setMotorSpeed(M4_pwm, M4_dir, w4 * 3);
}

void setMotorSpeed(int pwm_pin, int dir_pin, float speed) {
  int pwm_value = constrain(abs(speed), 0, 255);
  pwm_value = 255 - pwm_value;
  bool direction = (speed >= 0);

  if (pwm_pin == M3_pwm || pwm_pin == M4_pwm) {
    direction = !direction;
  }

  digitalWrite(dir_pin, direction ? HIGH : LOW);
  analogWrite(pwm_pin, pwm_value);
}

void parseSerialData(String data) {
    int xIndex = data.indexOf("\"x\":");
    int yIndex = data.indexOf("\"y\":");
    int zIndex = data.indexOf("\"z\":");

    if (xIndex != -1 && yIndex != -1 && zIndex != -1) {
        Vx = data.substring(xIndex + 4, data.indexOf(",", xIndex)).toFloat();
        Vy = data.substring(yIndex + 4, data.indexOf(",", yIndex)).toFloat();
        Omega = data.substring(zIndex + 4, data.indexOf("}", zIndex)).toFloat();

        dataReceived = true;
        lastDataTime = millis(); // reset timeout

        kinematics(Vx, Vy, Omega);

        Serial.print("Vx: "); Serial.print(Vx);
        Serial.print(" | Vy: "); Serial.print(Vy);
        Serial.print(" | Omega: "); Serial.println(Omega);
    } else {
        Serial.println("Error: Invalid format");
    }
}

void loop() {
    if (Serial.available()) {
        char inChar = Serial.read();
        inString += inChar;

        if (inChar == '\n') {
            parseSerialData(inString);
            inString = "";
        }
    }

    // stop motors if no new data for timeout period
    if (millis() - lastDataTime > timeout) {
        dataReceived = false;
    }

    if (dataReceived) {
        kinematics(Vx, Vy, Omega);
    } else {
        stopAllMotors();
    }
}
