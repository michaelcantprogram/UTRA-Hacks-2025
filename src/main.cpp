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
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    return pulseIn(ECHO, HIGH) * 0.034 / 2; // Convert to cm
}

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

    delay(duration);

    stop();
}

void Uturn() {
    stop();

    for (int i = 1; i <= 6 ; i++) {
        if (i % 2) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, 64);
            delay(800);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, 64);
            delay(850);
        }
        stop();
        delay(500);
    }

    stop();
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
