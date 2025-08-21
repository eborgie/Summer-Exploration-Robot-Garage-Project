
#include <Servo.h>

#define Ch0_p 2
#define Ch1_p 3
#define Ch2_p 4
#define Ch3_p 5
#define Ch4_p 6
#define Ch5_p 7

// Board 1
#define motorW1_speed 8 // Left front speed
#define motorW1_in1 9 // Left front dir 1
#define motorW1_in2 10 // Left front dir 2
#define motorW2_speed 11 // right front speed
#define motorW2_in3 13  // right front dir1
#define motorW2_in4 12  //right front dir 2

// Board 2
#define motorW3_speed 22 // Left mid speed
#define motorW3_in1 24 // Left mid dir 1
#define motorW3_in2 26 // Left mid dir 2
#define motorW4_speed 27 // right mid speed
#define motorW4_in3 23 // right mid dir 1
#define motorW4_in4 25 // right mid dir 2

// Board 3
#define motorW5_speed 28 // left back speed
#define motorW5_in1 30 // left back dir 1
#define motorW5_in2 32 // left back dir 2
#define motorW6_speed 29 // right back speed
#define motorW6_in3 31 // right back dir 1
#define motorW6_in4 33  // right back dir 2

// servo pins
#define SERVO_FL_PIN 35  // Front Left steering servo
#define SERVO_FR_PIN 37  // Front Right steering servo
#define SERVO_RL_PIN 39  // Rear Left steering servo
#define SERVO_RR_PIN 41  // Rear Right steering servo

Servo servoFL, servoFR, servoBL, servoBR;

// rover geometry constants 
float d1 = 472;  // Distance from center to front/back wheels (mm)
float d2 = 441;  // Distance from center to middle wheels (mm) 
float d3 = 472;  // Distance from center to outer wheels (mm) usually equal to d1
float d4 = 457 ;  // Half of wheelbase (center to left/right wheels) (mm)


int SERVO_CENTER = 180/2;       // Center position 
int SERVO_MAX_ANGLE = 90;     // Max steering angle from center (±90°)
int SERVO_MIN_POS = 0;       // Min servo position 
int SERVO_MAX_POS = 180/2 + 90;      // Max servo position 
float SERVO_SCALE = 1.5;      // scaling factor 

// Calibration variables
int ch0_neutral = 1500;  // Will be auto-calibrated
int ch2_neutral = 1500;  // Will be auto-calibrated
int deadband = 4;
int dcdeadband = 50;       // ±15µs around neutral



float r = 1000;  // Turning radius (mm)
float s = 0;     // Speed percentage (-100% to +100%)

// Speed variables 
float speed1, speed2, speed3;  // Calculated speeds for each wheel position

// Servo angles 
int thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack;

void setup() {
  Serial.begin(9600);
  
  // Set receiver pins as inputs
  pinMode(Ch0_p, INPUT);
  pinMode(Ch1_p, INPUT);
  pinMode(Ch2_p, INPUT);
  pinMode(Ch3_p, INPUT);
  pinMode(Ch4_p, INPUT);
  pinMode(Ch5_p, INPUT);
  
 
  // Board 1
  pinMode(motorW1_speed, OUTPUT);
  pinMode(motorW1_in1, OUTPUT);
  pinMode(motorW1_in2, OUTPUT);
  pinMode(motorW2_speed, OUTPUT);
  pinMode(motorW2_in3, OUTPUT);
  pinMode(motorW2_in4, OUTPUT);
  
  // Board 2
  pinMode(motorW3_speed, OUTPUT);
  pinMode(motorW3_in1, OUTPUT);
  pinMode(motorW3_in2, OUTPUT);
  pinMode(motorW4_speed, OUTPUT);
  pinMode(motorW4_in3, OUTPUT);
  pinMode(motorW4_in4, OUTPUT);
  
  // Board 3
  pinMode(motorW5_speed, OUTPUT);
  pinMode(motorW5_in1, OUTPUT);
  pinMode(motorW5_in2, OUTPUT);
  pinMode(motorW6_speed, OUTPUT);
  pinMode(motorW6_in3, OUTPUT);
  pinMode(motorW6_in4, OUTPUT);
  
  // Initialize servos
  servoFL.attach(SERVO_FL_PIN);
  servoFR.attach(SERVO_FR_PIN);
  servoBL.attach(SERVO_RL_PIN);
  servoBR.attach(SERVO_RR_PIN);
  
  
  centerAllServos();
  
  // Stop all motors initially
  stopAllMotors();
  
  Serial.println("=== Complete GoBuilda Rover Control System ===");
  Serial.println("6-Wheel Drive + 4-Wheel Ackermann Steering");
  Serial.println("RC Control with GoBuilda Element-6 System");
  Serial.println("Starting calibration...");
  
  // Auto-calibrate neutral positions
  calibrateChannels();
  
  Serial.println("Calibration complete! Starting rover control...");
  Serial.println("Format: CH0, CH2, Radius(mm), Speed(%), Servo_Angles");
  delay(1000);
}

void loop() {
  // Read PWM signals from GoBuilda Element receiver
  int ch0 = pulseIn(Ch0_p, HIGH, 30000); // Steering
  int ch1 = pulseIn(Ch1_p, HIGH, 30000); 
  int ch2 = pulseIn(Ch2_p, HIGH, 30000); // Throttle
  int ch3 = pulseIn(Ch3_p, HIGH, 30000);
  int ch4 = pulseIn(Ch4_p, HIGH, 30000);
  int ch5 = pulseIn(Ch5_p, HIGH, 30000);
  Serial.print(" ");
  Serial.print(ch0);
  Serial.print(" ");
  Serial.print(ch1);
  Serial.print(" ");
  Serial.print(ch2);
  Serial.print(" ");
  Serial.print(ch3);
  Serial.print(" ");
  Serial.print(ch4);
  Serial.print(" ");
  Serial.print(ch5);
  Serial.print(" ");
  int a= analogRead(SERVO_FL_PIN);
  int b = analogRead(SERVO_FR_PIN);
  Serial.print(" ");
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  
  // Only process if we get valid signals
  if (ch0 > 900 && ch0 < 2100 && ch2 > 900 && ch2 < 2100) {
    
    // Convert steering input to turning radius
    if (ch0 > (ch0_neutral + deadband)) {
      // Steering right
      r = map(ch0, ch0_neutral + deadband, 2100, 1500, d4 + 3);
    }
    else if (ch0 < (ch0_neutral - deadband)) {
      // Steering left  
      r = map(ch0, 900, ch0_neutral - deadband, d4 + 2, 1500);
    }
    else {
      // Steering neutral - go straight
      r = 5000; // Very large radius = straight line
    }
    
    // Convert throttle to speed percentage
    if (ch2 > (ch2_neutral + dcdeadband)) {
      // Forward
      s = map(ch2, ch2_neutral + dcdeadband, 2000, 0, 100);
    }
    else if (ch2 < (ch2_neutral - dcdeadband)) {
      // Reverse
      s = map(ch2, ch2_neutral - dcdeadband, 1000, 0, -100);
    }
    else {
      // neutral
      s = 0;
    }
    
    // Control the rover (drive + steering)
    controlRover();
    
    // Print control values for debugging
    Serial.print("CH0: "); Serial.print(ch0);
    Serial.print(", CH2: "); Serial.print(ch2);
    Serial.print(", Radius: "); Serial.print(r);
    Serial.print("mm, Speed: "); Serial.print(s); Serial.print("%");
    Serial.print(", Servos: FL="); Serial.print(thetaInnerFront);
    Serial.print(" FR="); Serial.print(thetaOuterFront);
    Serial.print(" RL="); Serial.print(thetaInnerBack);
    Serial.print(" RR="); Serial.println(thetaOuterBack);
  }
  else {
    // No valid signal received - emergency stop
    Serial.println("No signal - EMERGENCY STOP");
    stopAllMotors();
    centerAllServos();
    s = 0;
    r = 5000;
  }
  
  delay(50); // 20Hz update rate
}

void controlRover() {
  int ch0 = pulseIn(Ch0_p, HIGH, 30000);
  // Calculate motor speeds and servo angles
  calculateMotorsSpeed();
  calculateServoAngles();
  
  // Apply steering angles to servos
  controlSteering();
  
  
  // Apply speeds to motors based on turning direction
  if (abs(ch0 - ch0_neutral) <= deadband) {
    // Going straight - all wheels same speed
    controlLeftMotors(s, s, s);
    controlRightMotors(s, s, s);
  }
  else if (ch0 > (ch0_neutral + deadband)) {
    // Turning RIGHT - left wheels are outer (faster), right wheels are inner (slower)
    controlLeftMotors(speed1, speed2, speed3);   // Left side = outer wheels
    controlRightMotors(speed3, speed2, speed1);  // Right side = inner wheels (reversed order)
  }
  else {
    // Turning LEFT - right wheels are outer (faster), left wheels are inner (slower)  
    controlLeftMotors(speed3, speed2, speed1);   // Left side = inner wheels (reversed order)
    controlRightMotors(speed1, speed2, speed3);  // Right side = outer wheels
  }
}

void calculateMotorsSpeed() {
  // If going straight, all wheels same speed
  if (r > 1000) {
    speed1 = speed2 = speed3 = s;
  }
  else {
    // Outer wheels (furthest from turning point) have maximum speed
    speed1 = s;
    // Inner front and back wheels are closer to turning point
    speed2 = s * sqrt(pow(d3, 2) + pow((r - d1), 2)) / (r + d4);
    // Inner middle wheel is closest to turning point, has lowest speed
    speed3 = s * (r - d4) / (r + d4);
  }
  
  speed1 = constrain(speed1, -100, 100);
  speed2 = constrain(speed2, -100, 100);
  speed3 = constrain(speed3, -100, 100);
}

void calculateServoAngles() {
  // Calculate steering angles for Ackermann geometry
  if (r > 1000) {
    // Going straight - all servos centered
    thetaInnerFront = 0;
    thetaInnerBack = 0;
    thetaOuterFront = 0;
    thetaOuterBack = 0;
  }
  else {
    // Calculate angles based on turning radius
    thetaInnerFront = round((atan((d1 / (r + d4)))) * 180 / PI);
    thetaInnerBack = round((atan((d1 / (r + d4)))) * 180 / PI);
    thetaOuterFront = round((atan((d1 / (r - d4)))) * 180 / PI);
    thetaOuterBack = round((atan((d1 / (r - d4)))) * 180 / PI);
    
    thetaInnerFront = constrain(thetaInnerFront * SERVO_SCALE, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    thetaInnerBack = constrain(thetaInnerBack * SERVO_SCALE, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    thetaOuterFront = constrain(thetaOuterFront * SERVO_SCALE, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    thetaOuterBack = constrain(thetaOuterBack * SERVO_SCALE, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
  }
}

void controlSteering() {
  // Apply steering angles to servos based on turning direction
  int ch0 = pulseIn(Ch0_p, HIGH, 30000);
  if (ch0 > (ch0_neutral + deadband)) {
    // Turning RIGHT
    int leftAngle = constrain(SERVO_CENTER + thetaOuterFront, SERVO_MIN_POS, SERVO_MAX_POS);
    int rightAngle = constrain(SERVO_CENTER - thetaInnerFront, SERVO_MIN_POS, SERVO_MAX_POS);
    
    servoFL.write(leftAngle);    // Left front = outer
    servoFR.write(rightAngle);  // Right front = inner
    servoBL.write(constrain(SERVO_CENTER + thetaOuterBack, SERVO_MIN_POS, SERVO_MAX_POS));
    servoBR.write(constrain(SERVO_CENTER - thetaInnerBack, SERVO_MIN_POS, SERVO_MAX_POS));
    Serial.print(" ");
    Serial.print(leftAngle);
    Serial.print(" ");
    Serial.print(rightAngle);
    Serial.print(" ");
  }
  else if (ch0 < (ch0_neutral - deadband)) {
    // Turning LEFT
    int leftAngle = constrain(SERVO_CENTER - thetaInnerFront, SERVO_MIN_POS, SERVO_MAX_POS);
    int rightAngle = constrain(SERVO_CENTER + thetaOuterFront, SERVO_MIN_POS, SERVO_MAX_POS);
    
    servoFL.write(leftAngle);    // Left front = inner
    servoFR.write(rightAngle);  // Right front = outer
    servoBL.write(constrain(SERVO_CENTER - thetaInnerBack, SERVO_MIN_POS, SERVO_MAX_POS));
    servoBR.write(constrain(SERVO_CENTER + thetaOuterBack, SERVO_MIN_POS, SERVO_MAX_POS));
    Serial.print(" ");
    Serial.print(leftAngle);
    Serial.print(" ");
    Serial.print(rightAngle);
    Serial.print(" ");
  }
  else {
    // Going straight - center all servos
    centerAllServos();
  }
}

void centerAllServos() {
  servoFL.write(SERVO_CENTER);
  servoFR.write(SERVO_CENTER);
  servoBL.write(SERVO_CENTER);
  servoBR.write(SERVO_CENTER);
}

void controlLeftMotors(float frontSpeed, float midSpeed, float backSpeed) {
  // Left Front Motor (Wheel 1) - Driver Board 1, Motor A
  controlSingleMotor(motorW1_speed, motorW1_in1, motorW1_in2, frontSpeed);
  
  // Left Middle Motor (Wheel 2) - Driver Board 1, Motor B
  controlL298NMotorB(motorW2_speed, motorW2_in3, motorW2_in4, midSpeed);
  
  // Left Back Motor (Wheel 3) - Driver Board 2, Motor A
  controlSingleMotor(motorW3_speed, motorW3_in1, motorW3_in2, backSpeed);
}

void controlRightMotors(float frontSpeed, float midSpeed, float backSpeed) {
  // Right Front Motor (Wheel 4) - Driver Board 2, Motor B
  controlL298NMotorB(motorW4_speed, motorW4_in3, motorW4_in4, frontSpeed);
  
  // Right Middle Motor (Wheel 5) - Driver Board 3, Motor A
  controlSingleMotor(motorW5_speed, motorW5_in1, motorW5_in2, midSpeed);
  
  // Right Back Motor (Wheel 6) - Driver Board 3, Motor B
  controlL298NMotorB(motorW6_speed, motorW6_in3, motorW6_in4, backSpeed);
}

void controlSingleMotor(int enablePin, int in1Pin, int in2Pin, float speed) {
  // Convert speed percentage to PWM value (0-255)
  int pwmValue = map(round(abs(speed)), 0, 100, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Set motor speed
  analogWrite(enablePin, pwmValue);
  
  // Set motor direction
  if (speed > 0) {
    // forward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  else if (speed < 0) {
    // reverse
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  else {
    // Stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void controlL298NMotorB(int enablePin, int in3Pin, int in4Pin, float speed) {
  // Same as controlSingleMotor but for Motor B (in3/in4) on L298N
  int pwmValue = map(round(abs(speed)), 0, 100, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Set motor speed
  analogWrite(enablePin, pwmValue);
  
  // Set motor direction
  if (speed > 0) {
    // Forward
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  }
  else if (speed < 0) {
    // Reverse
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  }
  else {
    // Stop
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
  }
}

void stopAllMotors() {
  // Stop all 6 motors using correct L298N pin assignments
  controlSingleMotor(motorW1_speed, motorW1_in1, motorW1_in2, 0);        // Left Front
  controlL298NMotorB(motorW2_speed, motorW2_in3, motorW2_in4, 0);        // Left Middle  
  controlSingleMotor(motorW3_speed, motorW3_in1, motorW3_in2, 0);        // Left Back
  controlL298NMotorB(motorW4_speed, motorW4_in3, motorW4_in4, 0);        // Right Front
  controlSingleMotor(motorW5_speed, motorW5_in1, motorW5_in2, 0);        // Right Middle
  controlL298NMotorB(motorW6_speed, motorW6_in3, motorW6_in4, 0);        // Right Back
}

void calibrateChannels() {
  Serial.println("Place steering and throttle in NEUTRAL position");
  Serial.println("Calibrating in 3 seconds...");
  delay(3000);
  
  // Calibrate steering channel (CH0)
  Serial.println("Calibrating steering neutral...");
  ch0_neutral = readChannelAverage(Ch0_p, 50);
  Serial.print("CH0 neutral: "); Serial.println(ch0_neutral);
  
  // Calibrate throttle channel (CH2)  
  Serial.println("Calibrating throttle neutral...");
  ch2_neutral = readChannelAverage(Ch2_p, 50);
  Serial.print("CH2 neutral: "); Serial.println(ch2_neutral);
  
  Serial.println();
}

int readChannelAverage(int pin, int samples) {
  long sum = 0;
  int validSamples = 0;
  
  for(int i = 0; i < samples; i++) {
    int value = pulseIn(pin, HIGH, 25000);
    if (value > 900 && value < 2100) { // Valid PWM range
      sum += value;
      validSamples++;
    }
    delay(20);
  }
  
  if (validSamples > 0) {
    return sum / validSamples;
  } else {
    Serial.println("Warning: No valid signals detected");
    return 1500; // Default fallback
  }
}