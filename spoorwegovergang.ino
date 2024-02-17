


#include <Servo.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;

int Sensor1 = 4; // define the obstacle avoidance sensor interface
int Sensor2 = 5; // define the obstacle avoidance sensor interface

int led1 = 2;
int led2 = 3;

int Sensorval1;
int Sensorval2;
int bomen = 0;
int Sensordetected1 = 0;
int Sensordetected2 = 0;
int OldSensorValue = 0;
const int ledPin =  13;      // the number of the LED pin

int servoclose = 145;
int servoopen = 50;
int servopos1 = servoclose;
int servoMoved1 = 0;
int servopos2 = servoclose;
int servoMoved2 = 0;
int antidender = 0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers
int lastSensorval1 = LOW;   // the previous reading from the input pin
int lastSensorval2 = LOW;   // the previous reading from the input pin

bool led1high = false;
unsigned long intervalLed = 300;
unsigned long previousMillisLed;
unsigned long currentMillis;


void setup() {
  // put your setup code here, to run once:
  // initialize the LED pin as an output:
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode (ledPin, OUTPUT);
  pinMode (Sensor1, INPUT) ;// define the obstacle avoidance sensor output interface
  pinMode (Sensor2, INPUT) ;// define the obstacle avoidance sensor output interface
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Serial.begin(15200);
}
//          servo1.write(servoopen);
//         servo2.write(servoopen);

void loop() {
  currentMillis = millis();
  if (bomen == 1) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisLed >= intervalLed) {
      if (led1high) {
        led1high = false;
        digitalWrite(led2, HIGH);
        digitalWrite(led1, LOW);
      } else {
        led1high = true;
        digitalWrite(led2, LOW);
        digitalWrite(led1, HIGH);
      }
      previousMillisLed = millis();
    }


  }
  else {
    digitalWrite(led2, LOW);
    digitalWrite(led1, LOW);
  }

  // put your main code here, to run repeatedly:
  Sensorval1 = digitalRead (Sensor1);// Reading from the AvoidancePin
  Sensorval2 = digitalRead (Sensor2);// Reading from the AvoidancePin


  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (Sensorval1 != lastSensorval1) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if (Sensorval2 != lastSensorval2) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if (bomen == 0) {
      if ((Sensorval1 == LOW) || (Sensorval2 == LOW)) {
        bomen = 1;
        servo1.write(servoclose);
        servo2.write(servoclose);
        digitalWrite (ledPin, HIGH);
        if (Sensorval1 == LOW) {
          Sensordetected1 = 1;
        } else {
          Sensordetected2 = 1;
        }
      }
      else {
        servo1.write(servoopen);
        servo2.write(servoopen);
      }
    }

    if ((bomen == 1) && (Sensordetected1 == 1)) {
      // wait for sensor 2 downward slope
      if (Sensorval2 == LOW) {
        OldSensorValue = 1; //sensor2 detects train
      }
      else {
        if (OldSensorValue == 1) {
          //train passed sensor 2
          bomen = 0;
          servo1.write(servoopen);
          servo2.write(servoopen);
          digitalWrite (ledPin, LOW);
          Sensordetected1 = 0;
          Sensordetected2 = 0;
          OldSensorValue = 0;
          delay(2000);
        }
      }
    }

    if ((bomen == 1) && (Sensordetected2 == 1)) {
      // wait for sensor 1 downward slope
      if (Sensorval1 == LOW) {
        OldSensorValue = 1; //sensor1 detects train
      }
      else {
        if (OldSensorValue == 1) {
          //train passed sensor 1
          bomen = 0;
          servo1.write(servoopen);
          servo2.write(servoopen);
          digitalWrite (ledPin, LOW);
          Sensordetected1 = 0;
          Sensordetected2 = 0;
          OldSensorValue = 0;
          delay(2000);
        }
      }
    }
    //     delay(100);

    //  Serial.print("sensors = ");
    //  Serial.print(Sensorval1);
    //  Serial.print(Sensorval2);
    //  Serial.print("\n");
  }
}
