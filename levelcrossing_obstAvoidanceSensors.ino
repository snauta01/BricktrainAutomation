


#include <Servo.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;


int Sensor1 = 4; // define the obstacle avoidance sensor interface
int Sensor2 = 5; // define the obstacle avoidance sensor interface

int led1 = 2;
int led2 = 3;

int Sensorval1;
int Sensorval2;

int Sensordetected1 = 0;
int Sensordetected2 = 0;
int OldSensorValue = 0;
const int ledPin =  13;      // the number of the LED pin
bool trainFromSensor1 = false;
//bool trainFromSensor2 = false;
bool gateClose = false;
bool PreviouseSensorval1 = false;
bool PreviouseSensorval2 = false;


int servoclose = 149;
int servoopen = 50;
int servopos1 = servoclose;
int servoMoved1 = 0;
int servopos2 = servoclose;
int servoMoved2 = 0;

int counter = 0;
int sensorcounter1 = 0;
int sensorcounter2 = 0;



// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers
int Sensorarray1[30];   // the previous reading from the input pin
int Sensorarray2[30];   // the previous reading from the input pin

bool led1high = false;
unsigned long intervalLed = 300;
unsigned long previousMillisLed;
unsigned long intervalDebounce = 5;
unsigned long previousMillisDebounce;
unsigned long intervalOpendelay = 1000;
unsigned long previousMillisOpendelay;
unsigned long Sensorval1SetTrue;
unsigned long Sensorval2SetTrue;
unsigned long SensorvalDelay = 1000;
unsigned long currentMillis;



void setup() {
  Serial.begin(115200);

  Serial.println("Spoorwegovergang");
  // put your setup code here, to run once:
  // initialize the LED pin as an output:
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode (ledPin, OUTPUT);
  pinMode (Sensor1, INPUT) ;// define the obstacle avoidance sensor output interface
  pinMode (Sensor2, INPUT) ;// define the obstacle avoidance sensor output interface
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);


  for (int i = 0; i < 30 ; i++) {
    Sensorarray1[i] = HIGH;
    Sensorarray2[i] = HIGH;
  }
  gateClose = false;
  gate(gateClose);

}
//          servo1.write(servoopen);
//         servo2.write(servoopen);

void loop() {
  currentMillis = millis();


  
  if (gateClose) {
    //   unsigned long currentMillis = millis();
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

  if ((currentMillis - previousMillisDebounce >= intervalDebounce) )
  {

    Sensorarray1[counter] = digitalRead (Sensor1);// Reading from the AvoidancePin
    Sensorarray2[counter] = digitalRead (Sensor2);// Reading from the AvoidancePin
    if (counter < 30) {
      counter++;
    }
    else {
      counter = 0;
    }
    previousMillisDebounce  = millis();

    sensorcounter1 = 0;
    sensorcounter2 = 0;
    for (int i = 0; i < 30 ; i++) {
      if (Sensorarray1[i] == LOW)
        sensorcounter1++;
      if (Sensorarray2[i] == LOW)
        sensorcounter2++;
    }


    for (int i = 0; i < 30 ; i++) {
      Serial.print(Sensorarray1[i]);
    }
    Serial.print("\t\t");
    for (int i = 0; i < 30 ; i++) {
      Serial.print(Sensorarray2[i]);
    }
    Serial.println();



    if (sensorcounter1 > 5) {
      Sensorval1 = true;
      Sensorval1SetTrue = millis();
    } else {
      if (currentMillis - Sensorval1SetTrue >= SensorvalDelay) {
        Sensorval1 = false;
      }
    }
    if (sensorcounter2 > 5) {
      Sensorval2 = true;
      Sensorval2SetTrue = millis();
    } else {
      if (currentMillis - Sensorval2SetTrue >= SensorvalDelay) {
        Sensorval2 = false;
      }

    }
  }


  if (!gateClose) {
    if (Sensorval1) {
      trainFromSensor1 = true;
      gateClose = true;
      gate(gateClose);
    }
    if (Sensorval2) {
      trainFromSensor1 = false;
      gateClose = true;
      gate(gateClose);
    }
  } else {
    // gate close, open?
    if (trainFromSensor1) {
      if ((Sensorval2) and (!gateClose)) {
        previousMillisOpendelay = millis();
      }
      if ((!Sensorval2) and (PreviouseSensorval2)) {
        gateClose = false;
      }
    } else {
      if ((Sensorval1) and (!gateClose)) {
        previousMillisOpendelay = millis();
      }
      if ((!Sensorval1) and (PreviouseSensorval1)) {
        gateClose = false;
       }
    }
    if ((!gateClose) and (currentMillis - intervalOpendelay >= previousMillisOpendelay)) {
      gate(gateClose);
    }
  }
  PreviouseSensorval2 = Sensorval2;
  PreviouseSensorval1 = Sensorval1;
}

void gate(bool close) {

  if (close) {
 
  servo1.write(servoclose);
    servo2.write(servoclose);
  }
  else {
      servo1.write(servoopen);
    servo2.write(servoopen);
  }
}
