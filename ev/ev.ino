#include <WiFi.h>
#include <PID_v1.h>

// WiFi Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Telnet Server
WiFiServer server(23);
WiFiClient client;

// Motor Pins
#define MotorPinA 18
#define MotorPinB 16
#define MotorSpeedPin 22

#define a 21
#define b 15

volatile int motorPosition = 0;
double Setpoint, Input, Output;

// PID Tuning Parameters (Adjusted for small motors)
double Kp = 0.7, Ki = 0.01, Kd = 0.45;  
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

bool targetReached = false;

// Interrupt for Encoder
void IRAM_ATTR updatePosition() {
  static int lastA = LOW;
  int currentA = digitalRead(a);
  int currentB = digitalRead(b);

  if (lastA == LOW && currentA == HIGH) {
    motorPosition += (currentB == LOW) ? 1 : -1;
  } else if (lastA == HIGH && currentA == LOW) {
    motorPosition += (currentB == HIGH) ? 1 : -1;
  }

  lastA = currentA;
}

void setup() {
  Serial.begin(115200);
  
  // // WiFi Setup
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nConnected to WiFi!");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());

  // server.begin();  // Start Telnet server

  // Motor and Encoder Setup
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorPinB, OUTPUT);
  pinMode(MotorSpeedPin, OUTPUT);  
  pinMode(a, INPUT_PULLUP);
  pinMode(b, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(a), updatePosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(b), updatePosition, CHANGE);

  Setpoint = 11914;  // Target position

  //SPEED - 180
  // 1489.25/meter
  // 14888 --> 10m / Drift - 12 inches
  // 11914 --> 8m / Drift - 7.5 inches
  // 10428 --> 7m / Drift - 3 inches

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-180, 180);  

  delay(2000);
}

void loop() {
  // handleTelnetClient();

  if (!targetReached) {  
    Input = motorPosition;
    myPID.Compute();

    if (abs(Setpoint - Input) < 5) {  
      targetReached = true;
      stopMotor();
      // sendToTelnet("Target reached!");
    } else {
      moveMotor(Output);
    }

    // String log = "Motor Position: " + String(motorPosition) + " Output: " + String(Output);
    // sendToTelnet(log);

    Serial.printf("Encoder:");
    Serial.println(motorPosition);
    Serial.println("Output");
    Serial.println(Output);

    delay(10);
  }
}

void moveMotor(double output) {
  if (output > 0) {
    digitalWrite(MotorPinA, LOW);   
    digitalWrite(MotorPinB, HIGH);
    analogWrite(MotorSpeedPin, (int)output);  
  } else if (output < 0) {
    digitalWrite(MotorPinA, HIGH);   
    digitalWrite(MotorPinB, LOW);
    analogWrite(MotorSpeedPin, (int)-output);  
  } else {
    stopMotor();
  }
}

void stopMotor() {
  digitalWrite(MotorPinA, LOW);   
  digitalWrite(MotorPinB, LOW);
  analogWrite(MotorSpeedPin, 0);   
}

// // Telnet Handling
// void handleTelnetClient() {
//   if (!client || !client.connected()) {
//     client = server.available();
//   }
// }

// void sendToTelnet(String message) {
//   Serial.println(message); // Also print to USB serial
//   if (client && client.connected()) {
//     client.println(message);
//   }
// }
