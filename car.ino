#include <Servo.h>
#include <WiFiS3.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

// ===== Motor Pins (L298N) - Arduino R4 GPIO =====
const int ENA = 5, IN1 = 6, IN2 = 7;
const int ENB = 8, IN3 = 9, IN4 = 10;
const int SPEED_A = 160;  // Motor A speed
const int SPEED_B = 160;  // Motor B speed (adjust if one motor is stronger)

// ===== MH Line Sensor Pins =====
// MH sensor: LOW = Black (line), HIGH = White (no line)
const int SENSOR_L = 2;    // Left sensor D0 pin
const int SENSOR_R = A5;   // Right sensor analog

// ===== Servo Pins & Objects - Arduino R4 Pins =====
const int PIN_BASE = 3, PIN_FORW = 11, PIN_VERT = 12, PIN_CLAW = 13;
Servo baseServo, forwardServo, verticalServo, clawServo;
int basePos = 90, forPos = 90, vertPos = 90, clawPos = 90;
int baseNow = 90;  // current base servo angle
const int CLAW_OPEN = 180;
const int CLAW_CLOSED = 0;
const int BASE_SERVO_STEP_DEG = 1;                  // smaller = slower
const unsigned long BASE_SERVO_UPDATE_MS = 30;      // larger = slower
unsigned long lastBaseServoUpdate = 0;

// ===== WiFi & Command Socket =====
const char* ssid = "Dialog 4G 764";
const char* password = "3BA93d15";
const uint16_t COMMAND_PORT = 80;
WiFiServer commandServer(COMMAND_PORT);
WiFiClient commandClient;

// ===== State Machine =====
enum RobotState {
  STATE_IDLE,           // Waiting for item command
  STATE_PICKING,        // Executing pick sequence
  STATE_DONE            // Pick complete
};

RobotState currentState = STATE_IDLE;

unsigned long stateStartTime = 0;
String targetItem = "";
int pickStep = 0;

// ===== Function Declarations =====
void handleIncomingCommands();
bool processItemCommand(const String& itemName);
void executePickingSequence(unsigned long elapsed);
void setupMotors();
void setupServos();
void writeAllServos();
int stepToward(int current, int target, int step);
void setMotor(bool a1, bool a2, bool b1, bool b2);
void turnForward();
void turnBack();
void moveLeft();
void moveRight();
void stopMotors();

// ===== WiFi & Command Socket Setup =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // WiFi setup for Arduino R4
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    commandServer.begin();
    Serial.print("Command server listening on port ");
    Serial.print(COMMAND_PORT);
    Serial.println(" (plain TCP)");
  } else {
    Serial.println("\nFailed to connect WiFi - continuing in offline mode");
  }
  
  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_R, INPUT);

  setupMotors();
  setupServos();

  Serial.println("=== Robot Picker Ready ===");
  Serial.println("Waiting for item commands...");
}

bool processItemCommand(const String& itemName) {
  Serial.println("Item received: " + itemName);
  if (currentState == STATE_IDLE) {
    targetItem = itemName;
    currentState = STATE_PICKING;
    pickStep = 0;
    stateStartTime = millis();
    Serial.println("Starting pick sequence for: " + targetItem);
    return true;
  }

  Serial.println("Robot busy, command ignored.");
  return false;
}

void handleIncomingCommands() {
  if (!commandClient || !commandClient.connected()) {
    WiFiClient incoming = commandServer.available();
    if (incoming) {
      commandClient = incoming;
      Serial.println("Command client connected");
    }
  }

  if (commandClient && commandClient.connected() && commandClient.available()) {
    String cmd = commandClient.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      if (processItemCommand(cmd)) {
        commandClient.println("OK:" + cmd);
      } else {
        commandClient.println("BUSY:" + cmd);
      }
    }
  }
}

// ===== Main Loop =====
void loop() {
  // Handle incoming item commands from TCP client.
  handleIncomingCommands();
  
  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {
    case STATE_IDLE:
      // Keep current servo targets (no auto recenter).
      break;

    case STATE_PICKING:
      executePickingSequence(elapsed);
      break;
      
    case STATE_DONE:
      // Finish pick without changing arm position.
      currentState = STATE_IDLE;
      break;
  }
  
  // Update servos
  writeAllServos();
  
  delay(20);  // ~50 Hz update rate
}

// ===== PICKING SEQUENCE =====
void executePickingSequence(unsigned long elapsed) {
  // Pick item in the current arm position (no base/forward/vertical movement).
  if (elapsed < 350) {
    clawPos = CLAW_OPEN;      // Ensure open before grab
  }
  else if (elapsed < 1350) {
    clawPos = CLAW_CLOSED;    // Grab and hold item
  }
  else {
    currentState = STATE_DONE;
  }
}

// ===== Motor Helpers =====
void setupMotors() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
}

void setMotor(bool a1, bool a2, bool b1, bool b2) {
  analogWrite(ENA, SPEED_A);
  analogWrite(ENB, SPEED_B);
  digitalWrite(IN1, a1); digitalWrite(IN2, a2);
  digitalWrite(IN3, b1); digitalWrite(IN4, b2);
  delayMicroseconds(100);  // Small delay for stable motor control
}

void turnForward() { setMotor(HIGH, LOW, HIGH, LOW);  }
void turnBack()    { setMotor(LOW,  HIGH, LOW,  HIGH); }
void moveLeft()    { setMotor(LOW,  HIGH, HIGH, LOW);  } // Gentle left curve
void moveRight()   { setMotor(HIGH, LOW, LOW,  HIGH); } // Gentle right curve
void stopMotors()  { analogWrite(ENA, 0); analogWrite(ENB, 0); }

// ===== Servo Helpers =====
void setupServos() {
  baseServo.attach(PIN_BASE);
  forwardServo.attach(PIN_FORW);
  verticalServo.attach(PIN_VERT);
  clawServo.attach(PIN_CLAW);
  baseNow = basePos;
  writeAllServos();
}

int stepToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void writeAllServos() {
  unsigned long now = millis();
  if (now - lastBaseServoUpdate >= BASE_SERVO_UPDATE_MS) {
    lastBaseServoUpdate = now;
    baseNow = stepToward(baseNow, basePos, BASE_SERVO_STEP_DEG);
  }

  baseServo.write(baseNow);
  forwardServo.write(forPos);
  verticalServo.write(vertPos);
  clawServo.write(clawPos);
}
