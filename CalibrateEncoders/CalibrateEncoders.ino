/*
  CalibrateEncoders

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in a GitHub repository.

  https://github.com/iamazadi/unicycle
*/


// Optimized implementation of Bubble sort
#include <arduino-timer.h>
#include <FIR.h>
#include "ArduPID.h"
#include "Tools.cpp"

// These constants won't change. They're used to give names to the pins used:
const int oneAPin = 2;
const int twoAPin = 3;
const int enablePin = 22;
const int buttonPin = 23;
const int buttonAPin = 24;
const int buttonBPin = 25;
const int redPin = 4;
const int greenPin = 5;
const int bluePin = 6;
const int samplesNumber = 512;
const int period = 100; // the time interval between sensor readings in microseconds

int calibrated = 0;
int samplesCounter = 0;
int tableIndex = 0;
int tableBegin = 0;
int tableEnd = samplesNumber - 1;
int directionOfRotation = 0;
int minIndex = 0;
int secondMinIndex = 0;
int maxIndex = samplesNumber - 1;
int countdown = 64;
int debugMinMaxValues = 1;
double table1[samplesNumber] = {};
double table2[samplesNumber] = {};
double processedReading1 = 0.0;
double processedReading2 = 0.0;
double sensorPosition = 0.0;
double minValue = 1023.0;
double maxValue = 0.0;
double averageVelocity = 0.0;
double minVelocity = 1023.0;
double maxVelocity = 0.0;
float hue = 0.0;

// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir1;
FIR<float, 8> fir2;

Timer<1, micros> timer; // create a timer with 1 task and microsecond resolution
Timer<1, micros> timer1; // create a timer with 1 task and microsecond resolution

// create a timer that can hold 1 concurrent task, with microsecond resolution
// and a custom handler type of 'const char *
Timer<1, micros, const char *> u_timer;

ArduPID myController;

double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 511;
double kp = 7.0;
double ki = 0.01;
double kd = 0.001;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonAState = 0;
int buttonBState = 0;
int myTime = millis();


double getPosition(double a, double b) {
  return sqrt(pow(a, 2) + pow(b, 2));
}


bool changeSetpoint(void *) {
  if (calibrated == 1) {
    if (directionOfRotation == 0)
      tableIndex = min(tableIndex + 1, tableEnd);
    if (directionOfRotation == 100)
      tableIndex = max(tableIndex - 1, tableBegin);
    if (tableIndex <= tableBegin || tableIndex >= tableEnd)
      directionOfRotation = 100 - directionOfRotation;
    setpoint = getPosition(table1[tableIndex], table2[tableIndex]);
  }
  return true;
}


bool readSensors(void *) {
  processedReading1 = fir1.processReading(analogRead(A0));
  processedReading2 = fir2.processReading(analogRead(A1));
  sensorPosition = getPosition(processedReading1, processedReading2);
  if (calibrated == 1) {
    input = sensorPosition;
    myController.compute();
  }

  // Need to run at least the length of the filter for reliable processed values.
  if (countdown > 0)
    countdown--;
  if (countdown == 0 && calibrated == 0 && samplesCounter < samplesNumber) {
    table1[samplesCounter] = processedReading1;
    table2[samplesCounter] = processedReading2;
    samplesCounter++;
  }
  return true; // repeat? true
}

bool calibrate(const char *m) {
  digitalWrite(enablePin, LOW);

  double _position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double accumulator = 0.0;
  double threshold = 1.0;
  double threshold2 = 5.0;
  int counter = 0;

  // find the minimum velocity
  for (int i = 1; i < samplesNumber; i++) {
    _position = getPosition(table1[i], table2[i]);
    velocity = _position - getPosition(table1[i - 1], table2[i - 1]);
    accumulator += abs(velocity);
    counter++;
    if (abs(velocity) < minVelocity) {
      minVelocity = abs(velocity);
    }
    if (abs(velocity) > maxVelocity) {
      maxVelocity = abs(velocity);
    }
  }
  // Find the average distance between two consecutive sensor positions
  averageVelocity = accumulator / counter;

  threshold = maxVelocity / 4.0;
  // threshold2 = 1.0 * maxVelocity;
  

  // Find the minimum position value
  for (int i = 2; i < samplesNumber; i++) {
    _position = getPosition(table1[i], table2[i]);
    velocity = _position - getPosition(table1[i - 1], table2[i - 1]);
    acceleration = velocity - (getPosition(table1[i - 1], table2[i - 1]) - getPosition(table1[i - 2], table2[i - 2]));
    // the first derivative at critical points is approximately equal to zero
    if (abs(velocity) <= threshold) {
      // the second derivative at valleys is positive
      // look in the first third of samples
      if (acceleration > 0.0) {
        if (_position < minValue && i < samplesNumber / 3) {
          minValue = _position;
          minIndex = i;
        }
      }
    }
  }

  // Find the maximum position value
  for (int i = minIndex + 2; i < samplesNumber; i++) {
    _position = getPosition(table1[i], table2[i]);
    velocity = _position - getPosition(table1[i - 1], table2[i - 1]);
    acceleration = velocity - (getPosition(table1[i - 1], table2[i - 1]) - getPosition(table1[i - 2], table2[i - 2]));
    // the first derivative at critical points is approximately equal to zero
    if (abs(velocity) <= threshold) {
      // the second derivative at peaks is negative
      // ensure that the maximum value occurs after the minimum value
      if (acceleration < 0.0) { 
        if (_position > maxValue && i > minIndex * 1.1 && i <  samplesNumber * 2 / 3) {
          maxValue = _position;
          maxIndex = i;
        }
      }
    }
  }

  int distance = 0;
  if (maxIndex > minIndex * 1.1)
    distance = maxIndex - minIndex;
  else
    return false;

  // Find the second occurance of the minimum value for getting the whole period
  for (int i = maxIndex + 2; i < samplesNumber; i++) {
    _position = getPosition(table1[i], table2[i]);
    velocity = _position - getPosition(table1[i - 1], table2[i - 1]);
    acceleration = velocity - (getPosition(table1[i - 1], table2[i - 1]) - getPosition(table1[i - 2], table2[i - 2]));
    // the first derivative at critical points is approximately equal to zero
    if (abs(velocity) <= threshold) {
      // the second derivative at valleys is positive
      // look in the first third of samples
      if (acceleration > 0.0 && abs(_position - minValue) <= 2.0 * averageVelocity && i < maxIndex + distance * 1.1) {
        secondMinIndex = i;
      }
    }
  }

  if (maxValue > minValue && maxIndex > minIndex && secondMinIndex > maxIndex && abs((maxIndex - minIndex + maxIndex) - secondMinIndex) < 100)
    calibrated = 1;

  return false; // repeat? true
}


// the setup routine runs once when you press reset:
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  minValue = getPosition(1023.0, 1023.0);
  maxValue = getPosition(0.0, 0.0);
  minVelocity = minValue;
  maxVelocity = maxValue;

  // For a moving average we want all of the coefficients to be unity.
  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};

  // Set the coefficients
  fir1.setFilterCoeffs(coef);
  fir2.setFilterCoeffs(coef);

  // call the calculate_velocity function every 1000 micros (1 second)
  timer.every(period, readSensors);
  // call the calculate_velocity function every 1000 micros (1 second)
  timer1.every(100000, changeSetpoint);

  // set ADC pins A0 and A1 as input
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  // set pins 2 and 3 as outputs:
  pinMode(oneAPin, OUTPUT);
  pinMode(twoAPin, OUTPUT);
  // set pin 22 as output:
  pinMode(enablePin, OUTPUT);
  // set the button pin as input
  pinMode(buttonPin, INPUT);
  pinMode(buttonAPin, INPUT);
  pinMode(buttonBPin, INPUT);
  // set the RGB pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(enablePin, HIGH);
  // Run the motor at a constant speed for calibration
  int dutyCycleOneA = 255;
  int dutyCycleTwoA = 0;
  analogWrite(oneAPin, dutyCycleOneA);
  analogWrite(twoAPin, dutyCycleTwoA);

  myController.begin(&input, &output, &setpoint, kp, ki, kd);

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(0, 255);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)

  // call print_message in 5 seconds, but with microsecond resolution
  u_timer.in(6000000, calibrate, "delayed five seconds using microseconds");
}

// the loop routine runs over and over again forever:
void loop() {
  if (calibrated == 1) {

    int _output = 255;
    int dutyCycleOneA = _output;
    int dutyCycleTwoA = 255 - _output;

    hue = (sensorPosition - minValue) * 360.0 / (maxValue - minValue);
    float saturation = 1.0;
    float brightness = 1.0;
    hsv hsvColor;
    hsvColor.h = hue;
    hsvColor.s = saturation;
    hsvColor.v = brightness;
    rgb rgbColor = hsv2rgb(hsvColor);
    int dutyCycleRed = 255 - (int) rgbColor.r * 255;
    int dutyCycleGreen = 255 - (int) rgbColor.g * 255;
    int dutyCycleBlue = 255 - (int) rgbColor.b * 255;

    analogWrite(oneAPin, dutyCycleOneA);
    analogWrite(twoAPin, dutyCycleTwoA);

    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
    buttonAState = digitalRead(buttonAPin);
    buttonBState = digitalRead(buttonBPin);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == HIGH && calibrated == 1) {
      // turn motor on:
      digitalWrite(enablePin, HIGH);
      analogWrite(redPin, dutyCycleRed);
      analogWrite(greenPin, dutyCycleGreen);
      analogWrite(bluePin, dutyCycleBlue);
    } else {
      // turn motor off:
      digitalWrite(enablePin, LOW);
      analogWrite(redPin, 255);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 255);
    }
  }
  
  // print out the value you read:
  if (debugMinMaxValues == 1) {
    Serial.print("i: "); Serial.print(minIndex); Serial.print(" ");
    Serial.print("j: "); Serial.print(maxIndex); Serial.print(" ");
    Serial.print("k: "); Serial.print(secondMinIndex); Serial.print(" ");
    Serial.print("n: "); Serial.print(minValue); Serial.print(" | ");
    Serial.print("x: "); Serial.print(maxValue); Serial.print(" | ");
  }
  Serial.print("s: "); Serial.print(sensorPosition); Serial.print(" ");
  
  Serial.print("h: "); Serial.print(hue); Serial.print(" ");
  Serial.print("c: "); Serial.print(samplesCounter); Serial.print(" ");
  Serial.print("u: "); Serial.print(minVelocity); Serial.print(" ");
  Serial.print("v: "); Serial.print(averageVelocity); Serial.print(" ");
  Serial.print("w: "); Serial.print(maxVelocity); Serial.print(" | ");
  
  Serial.println("uT");
  timer.tick(); // tick the timer
  timer1.tick();
  u_timer.tick();
}
