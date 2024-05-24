#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PinChangeInterrupt.h>

// button pins
const int buttonPin1 = 2; // pin connected to button 1
const int buttonPin2 = 3; // pin connected to button 2

// IR sensor pins
const int irSensorPin1 = 6; // first IR sensor pin
const int irSensorPin2 = 7; // second IR sensor pin

// ultrasonic sensor pins
const int ultrasonicEchoPin = 12;  // echo pin
const int ultrasonicTrigPin = 13; // trigger pin

// led pin
const int ledPin = 11; // led pin

// servo pins
const int servoBarrierPin = 5; // pin for the servomotor used as barrier
const int servoElevatorPin = 8; // pin for the servomotor used as elevator

// create a servo object for the barrier
Servo servoBarrier;

// create a servo object for the elevator
Servo servoElevator;

// Create LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);

// variables used for working with the ultrasonic sensor data
long duration;
int distance = 1000;

// Car count variable
volatile int carCount = 0;
volatile bool carDetected = false;
volatile unsigned long lastMillis = 0;

// Button press flags
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;

void setup() {
  // initialize serial communication
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);

  // set the button pins as input with internal pull-up resistors
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  // attach interrupts to the button pins
  attachInterrupt(digitalPinToInterrupt(buttonPin1), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), button2ISR, FALLING);

  // set the IR sensor pins as input
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);

  // attach interrupts to the IR sensor pins
  attachPCINT(digitalPinToPCINT(irSensorPin1), irSensorsISR, CHANGE);
  attachPCINT(digitalPinToPCINT(irSensorPin2), irSensorsISR, CHANGE);

  // set the ultrasonic sensor pins (echo as input and trigger as output)
  pinMode(ultrasonicEchoPin, INPUT);
  pinMode(ultrasonicTrigPin, OUTPUT);

  // set the barrier servo angle to 0 degrees
  servoBarrier.attach(servoBarrierPin);
  servoBarrier.write(0);

  // set the elevator servo to a still position
  servoElevator.attach(servoElevatorPin);
  servoElevator.write(90);

  // initialize the LCD
  lcd.begin();

  // turn on the blacklight and print current number of cars which entered parking space
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Cars entered: 0");
}

// Interrupt Service Routine (ISR) for button 1
void button1ISR() {
  button1Pressed = true;
}

// Interrupt Service Routine (ISR) for button 2
void button2ISR() {
  button2Pressed = true;
}

// Interrupt Service Routine (ISR) for the 2 IR sensors used
void irSensorsISR() {
  if (digitalRead(irSensorPin1) == HIGH && digitalRead(irSensorPin2) == HIGH) { 
    carDetected = true;
    lastMillis = millis();
  } else {
    openBarrier();
  }
}

// function used for measuring distance using an ultrasonic sensor
unsigned int measureDistance(int trigPin, int echoPin) {
  // Send a 10us pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure the duration of the echo pulse
  unsigned long duration = pulseIn(echoPin, HIGH);

  // calculate the distance in centimeters and then return it
  unsigned int distance = duration * 0.034 / 2;

  return distance;
}

// function that opens the barrier
void openBarrier() {
  servoBarrier.write(90);
}

// function that closes the barrier
void closeBarrier() {
  servoBarrier.write(0);
}

// function used for increasing LED intensity
void increaseLedIntensity() {
  for (int i = 0; i < 256; i++) {
    analogWrite(ledPin, i);
    delay(5);
  }
}

// function used for decreasing LED intensity
void decreaseLedIntensity() {
  for (int i = 255; i >= 0; i--) {
    analogWrite(ledPin, i);
    delay(5);
  }
}

// Function used for moving the elevator up
void moveElevatorUp()
{
  if (distance > 3)
  {
    servoElevator.write(180); // start the elevator if it isn't already at the second floor

    for (int i = 0; i < 10; i++) {
      // get distance from the lift to the ultrasonic sensor; if it's lower than 3, stop
      distance = measureDistance(ultrasonicTrigPin, ultrasonicEchoPin);

      if (distance <= 3)
      {
        servoElevator.write(90); // stop the elevator if we arrived at the second floor
        Serial.println("We arrived at the second floor!");
        break;
      }

      lcd.setCursor(1, 1);
      lcd.print("Height: ");
      lcd.print(max(12 - distance, 0));
      lcd.print(" cm");
      delay(325);
    }

    // stop the elevator after the allocated time passed or the elevator arrived at the second floor
    servoElevator.write(90);
    
    decreaseLedIntensity();
  }
}

// Function used for moving the elevator down
void moveElevatorDown()
{
  if (distance < 10) {
    servoElevator.write(0); // start the elevator if it isn't already at the first floor
    
    for (int i = 0; i < 10; i++) {
      // get distance from the lift to the ultrasonic sensor; if it's higher than 3, stop
      distance = measureDistance(ultrasonicTrigPin, ultrasonicEchoPin);

      if (distance >= 13.5) {
        servoElevator.write(90); // stop the elevator if we arrived at the first floor
        Serial.println("We arrived at the first floor!");
        break;
      }

      lcd.setCursor(1, 1);
      lcd.print("Height: ");
      lcd.print(max(12 - distance, 0));
      lcd.print(" cm");
      delay(275);
    }

    // stop the elevator after the allocated time passed or the elevator arrived at the first floor
    servoElevator.write(90);
    distance = 14;

    increaseLedIntensity();
  }
}

void loop()
{
  // Check if a second has passed since the last car entered the parking space
  if (carDetected && millis() - lastMillis >= 1000
    && digitalRead(irSensorPin1) == HIGH && digitalRead(irSensorPin2) == HIGH) {
      carCount++;
      Serial.println(carCount);

      closeBarrier();
      carDetected = false;  // reset the flag for the detected car
      lcd.setCursor(15, 0);
      lcd.print(carCount);
  }

  if (button1Pressed) {
    button1Pressed = false; // reset the flag for the first button (go up)
    Serial.println("We're going to the second floor!");
    moveElevatorUp();
  }

  if (button2Pressed) {
    button2Pressed = false; // reset the flag for the second button (go down)
    Serial.println("We're going to the first floor!");
    moveElevatorDown();
  }
}
