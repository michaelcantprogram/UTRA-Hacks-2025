#include <Arduino.h>

#include <Servo.h>

// Motor Pins
#define ENA 9  // Left motor speed (PWM)
#define ENB 10 // Right motor speed (PWM)
#define IN1 8  // Left motor direction 1
#define IN2 7  // Left motor direction 2
#define IN3 11 // Right motor direction 1
#define IN4 12 // Right motor direction 2

#define OPEN 40
#define CLOSE 90

#define RED 777
#define BLUE 888
#define GREEN 999
#define BLACK -1

// Ultrasonic Sensor Pins
#define TRIG 2 // Trigger Pin
#define ECHO 3 // Echo Pin

// Servo Motor Pin
#define SERVO_PIN 6

#define S0 0   // Frequency scaling selection
#define S1 1   // Frequency scaling selection
#define S2 4   // Color selection
#define S3 5   // Color selection
#define OUT 13 // Color frequency output

Servo myServo;

void stop();
void setServoAngle(int angle);
void Uturn();

// Function to move the servo to a specific angle
void setServoAngle(int angle) {
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    Serial.print("Moving Servo to: ");
    Serial.println(angle);

    myServo.write(angle);
    delay(500);
}

void setup() {
    Serial.begin(9600);

    // Motor Setup
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Ultrasonic Sensor Setup
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    // Color Sensor Setup
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(OUT, INPUT);

    // Set frequency scaling to 20% (balanced performance)
    digitalWrite(S0, HIGH);
    digitalWrite(S1, HIGH);

    // Servo Setup
    myServo.attach(SERVO_PIN);
    setServoAngle(20); // Initialize servo at 0 degrees

    Serial.println("Robot Initialized.");
}

// Function to measure distance using ultrasonic sensor
float getDistance() {
    // Ensure the trigger pin is low
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);  // Wait a brief moment
    
    // Send a 10 microsecond pulse to trigger the ultrasonic burst
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    
    // Read the duration of the echo pulse in microseconds
    long duration = pulseIn(ECHO, HIGH);

    if (duration == 0) {
        Serial.println("No echo received. Returning max distance.");
        // Return a high distance value to indicate "no obstacle"
        return -1; // You can adjust this value based on your application
    }
    
    // Calculate the distance in centimeters
    // (Speed of sound ~0.034 cm per microsecond, divided by 2 for the round trip)
    float distance = duration * 0.034 / 2;
    
    return distance;
}

// Function for challenge 2:



// Function to detect color frequency
int getColorFrequency(int s2State, int s3State) {
    digitalWrite(S2, s2State);
    digitalWrite(S3, s3State);
    delay(50);
    return pulseIn(OUT, LOW);
}

// Function to read color
int detectColor() {
    int red = getColorFrequency(LOW, LOW);
    int green = getColorFrequency(HIGH, HIGH);
    int blue = getColorFrequency(LOW, HIGH);

    Serial.print("R: \n");
    Serial.print(red);
    Serial.print("G: \n");
    Serial.print(green);
    Serial.print("B: \n");
    Serial.println(blue);

    if (green < red && green < blue)
        return GREEN; // Green detection condition
    if (red < green && red < blue)
        return RED;
    if (blue < red && blue < green)
        return BLUE;

    return -1;
}

// Move forward
void moveForward(int speed, int duration) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);

    if (duration > 0)
        delay(duration);

        stop();
}

void Uturn() {
    stop();

    for (int i = 1; i < 20; i++) {
        if (i % 2) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, 64);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, 64);
        }
        delay(1000);
        stop();
        delay(500);
    }

    stop();
}

// Turn Left: left motor goes backward, right motor goes forward
void turnLeft(int speed) {
    Serial.println("Turning Left");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
    
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
    
    delay(700); // Adjust for a 90° turn
    stop();
}

// Turn Right: left motor goes forward, right motor goes backward
void turnRight(int speed) {
    Serial.println("Turning Right");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
    
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
    
    delay(700); // Adjust for a 90° turn
    stop();
}

void challenge2() {
    int motorSpeed = 200;  // Set the motor speed (adjust as needed)

    while (true) {  // Continuous loop
        float distance = getDistance();
        
        if (distance > 15 || distance < 0) {
            // No obstacle: keep going straight.
            moveForward(motorSpeed, -1);
        } else {
            // Obstacle detected: stop the robot.
            stop();
            delay(100);  // Brief pause to stabilize
            
            // Read the ground color.
            int currentColor = detectColor();
            if (currentColor == BLACK) {
                // On a black square: stop permanently.
                Serial.println("Obstacle detected on a BLACK square. Stopping.");
                stop();
                while (true) {
                    delay(1000);  // Remain stopped indefinitely.
                }
            } else if (currentColor == BLUE) {
                turnLeft(motorSpeed);
            } else if (currentColor == GREEN) {
                turnRight(motorSpeed);
            } else if (currentColor == RED) {
                Uturn();
            } else {
                // In case of an unexpected value, continue forward.
                Serial.println("Unknown color detected; continuing straight.");
            }
        }
    }
}

// Stop motors
void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void loop() {
    Uturn();
    delay(10000);
}
