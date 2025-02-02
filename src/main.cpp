#include <Arduino.h>
#include <Ultrasonic.h>
#include <Servo.h>
#include <HCSR04.h>

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

byte triggerPin = 2;
byte echoPin = 3;

// Servo Motor Pin
#define SERVO_PIN 6

#define S0 0   // Frequency scaling selection
#define S1 1   // Frequency scaling selection
#define S2 4   // Color selection
#define S3 5   // Color selection
#define OUT 13 // Color frequency output

Servo myServo;

Ultrasonic ultrasonic(TRIG, ECHO);

bool isStopped = false;

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
    // pinMode(TRIG, OUTPUT);
    // pinMode(ECHO, INPUT);
    HCSR04.begin(triggerPin, echoPin);


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

float getDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(5); // Ensure a clean LOW pulse
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10); // Send a 10µs pulse (as required)
    digitalWrite(TRIG, LOW);

    // Measure the echo pulse duration
    long duration = pulseIn(ECHO, HIGH);

    if (duration == 0) {
        return 999; // Indicates no valid measurement
    }

    // Convert duration to distance in cm (Using formula from timing diagram)
    float distance = duration * 0.034 / 2;

    // Ensure the reading is within a valid range
    if (distance < 2 || distance > 400) {
        Serial.println("Out-of-range reading.");
        return 999;
    }

    delay(50); // Small delay to stabilize readings

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

// Function to read Red Pulse Widths
int getRedPW() {
    // Set sensor to read Red only
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(OUT, LOW);
    // Return the value
    return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
    // Set sensor to read Green only
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(OUT, LOW);
    // Return the value
    return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
    // Set sensor to read Blue only
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    // Define integer to represent Pulse Width
    int PW;
    // Read the output Pulse Width
    PW = pulseIn(OUT, LOW);
    // Return the value
    return PW;
}

int detectColor() {
    int numSamples = 5; // Take multiple readings for accuracy
    int red = 0, green = 0, blue = 0;

    for (int i = 0; i < numSamples; i++) {
        red += getRedPW();
        green += getGreenPW();
        blue += getBluePW();
        delay(10); // Small delay between samples
    }

    // Average the readings
    red /= numSamples;
    green /= numSamples;
    blue /= numSamples;

    // Serial.print("Raw R: ");
    // Serial.print(red);
    // Serial.print(" G: ");
    // Serial.print(green);
    // Serial.print(" B: ");
    // Serial.println(blue);

    // Normalize readings by computing the inverse (pulse width is inversely proportional to color intensity)
    float sum = red + green + blue;
    float redRatio = 1.0 / red;
    float greenRatio = 1.0 / green;
    float blueRatio = 1.0 / blue;

    // Serial.print("Normalized R: ");
    // Serial.print(redRatio, 3);
    // Serial.print(" G: ");
    // Serial.print(greenRatio, 3);
    // Serial.print(" B: ");
    // Serial.println(blueRatio, 3);

    if (sum > 450) {
        return BLACK;
    }

    int redbluediff = abs(red - blue);
    int redgreendiff = abs(red - green);
    int bluegreendiff = abs(blue - green);

    if (redbluediff < 20 && redgreendiff < 20 && bluegreendiff < 20) {
        Serial.println("Black");
        return BLACK;
    }

    // Determine dominant color using normalized values
    if (greenRatio > redRatio * 1.1 && greenRatio > blueRatio * 1.1) {
        Serial.println("Green");
        return GREEN;
    }
    if (redRatio > greenRatio * 1.1 && redRatio > blueRatio * 1.1) {
        Serial.println("Red");
        return RED;
    }
    if (blueRatio > redRatio * 1.1 && blueRatio > greenRatio * 1.1) {
        Serial.println("Blue");
        return BLUE;
    }

    Serial.println("Black");
    return BLACK; // Default case if no dominant color is found
}

// Move forward
void moveForward(int duration) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 1);
    analogWrite(ENB, 1);

    if (duration > 0) {
        delay(duration);
        stop();
    }
}

void moveBackward(int duration) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, 32);
    analogWrite(ENB, 32);

    if (duration > 0) {
        delay(duration);
        stop();
    }
}

void Uturn() {
    stop();

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 32);
    analogWrite(ENB, 32);
    delay(780);

    stop();
}

// Turn Left: left motor goes backward, right motor goes forward
void turnLeft(int duration = 400) {
    stop();
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 32);
    analogWrite(ENB, 32);
    delay(duration);
    stop();
}

// Turn Right: left motor goes forward, right motor goes backward
void turnRight(int duration = 400) {
    stop();
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 32);
    analogWrite(ENB, 32);
    delay(duration);
    stop();
}

void challenge2() {
    while (1) {
        moveForward(0);

        if (detectColor() != BLACK && *HCSR04.measureDistanceCm() > 20) {
            continue;
        }

        if (detectColor() == BLACK && *HCSR04.measureDistanceCm() <= 25) {
            stop();
            return;
        } else if (detectColor() == BLACK) {
            stop();
            moveBackward(2000);
        }

        if (detectColor() == RED) {
            stop();
            Uturn();
        } else if (detectColor() == GREEN) {
            stop();
            turnRight(400);
        } else if (detectColor() == BLUE) {
            stop();
            turnLeft(400);
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

void challenge1() {
    setServoAngle(CLOSE);
    int prev = BLACK, curr = BLACK;
    int num = 0;

    moveForward(2000);
    int j = 0;
    while (detectColor() == BLACK) {
        j++;
        moveBackward(2000);
        if (j % 2 == 0) {
            turnLeft(min(100 * j, 800));
        } else {
            turnRight(min(100 * j, 800));
        }
        moveForward(2000);
        delay(500);
    }
    moveBackward(2000);
    while (detectColor() == curr) {
        moveForward(30);
        delay(10);
    }
    num++;
    curr = detectColor();
    delay(200);

    while (num < 5) {
        int i = 0;
        moveForward(400);
        while (detectColor() == curr || detectColor() == prev) {
            i++;
            moveBackward(400);
            if (i % 2 == 0) {
                turnLeft(min(100 * i, 800));
            } else {
                turnRight(min(100 * i, 800));
            }
            moveForward(400);
            delay(500);
        }
        moveBackward(400);
        num++;
        while (detectColor() == curr) {
            moveForward(30);
            delay(10);
        }
        prev = curr;
        curr = detectColor();
        delay(200);
    }

    moveBackward(600);
    setServoAngle(OPEN);
}

int distance = 0;

void loop() {
    // challenge1();
    // while (1)
    //     ;

    double* distances = HCSR04.measureDistanceCm();
  
  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  Serial.println("---");
  delay(250);
}
