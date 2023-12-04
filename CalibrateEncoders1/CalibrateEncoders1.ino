#include <arduino-timer.h>
#include <FIR.h>
#include "Tools.cpp"

// These constants won't change. They're used to give names to the pins used:
const int oneAPin = 2;
const int twoAPin = 3;
const int enablePin = 22;
const int buttonPin = 26;
const int redPin = 4;
const int greenPin = 5;
const int bluePin = 6;

const int samplesNumber = 650; // the total number of raw pints in tables
const int period = 5000; // the time interval between sensor readings in microseconds
const int kernelSize = 1; // the kernel size of convolutional filters

// variables will change:
int calibrated = 0;
int tableIndex = 0; // the current index of the table
int tableBegin = 0;
int tableMiddle = samplesNumber / 2;
int tableEnd = tableMiddle + 1;
int countdown = 16; // for discarding at least 8 points so that the moving average stabilizes
int samplesCounter = 0; // for keeping track of the number of recorded points in tables
int buttonState = 0; // variable for reading the pushbutton status

double table1[samplesNumber] = {};
double table2[samplesNumber] = {};
double minKernel[kernelSize] = {};
double maxKernel[kernelSize] = {};
double processedReading1 = 0.0;
double processedReading2 = 0.0;
double processedReading = 0.0;
double wheelPosition = 0.0;
double wheelVelocity = 0.0;

// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir1;
FIR<float, 8> fir2;

// For reading sensors
Timer<1, micros> timer; // create a timer with 1 task and microsecond resolution
// For calibrating sensors at startup
Timer<1, micros> u_timer; // create a timer with 1 task and microsecond resolution


bool readSensors(void *) {
  processedReading1 = fir1.processReading(analogRead(A0));
  processedReading2 = fir2.processReading(analogRead(A1));
  double reading = getPosition(processedReading1, processedReading2);
  double change = reading - processedReading;
  processedReading = reading;

  if (calibrated == 1) {
    int firstHit = tableBegin;
    int secondHit = tableEnd;
    double distance = 1000000.0;
    double _distance;
    double _position;

    // For when the sensor position is at a critical point (peaks and valleys)
    if (abs(tableIndex - tableBegin) < 3 || abs(tableIndex - tableMiddle) < 3 || abs(tableIndex - tableEnd) < 3) {
      double entropy1 = 0.0;
      double entropy2 = 0.0;
      // Determine which table has the steepest slope with tableIndex
      for (int i = 0; i < kernelSize; i++) {
        entropy1 += abs(table1[tableIndex + i] - table1[tableIndex + i - 1]);
        entropy2 += abs(table2[tableIndex + i] - table2[tableIndex + i - 1]);
      }
      // Use table 1
      if (entropy1 >= entropy2) {
        distance = 1000000.0;
        for (int i = tableBegin; i < tableMiddle; i++) {
          _distance = abs(table1[i] - processedReading1);
          if (_distance < distance) {
            distance = _distance;
            firstHit = i;
          }
        }
        distance = 1000000.0;
        for (int i = tableMiddle; i <= tableEnd; i++) {
          _distance = abs(table1[i] - processedReading1);
          if (_distance < distance) {
            distance = _distance;
            secondHit = i;
          }
        }
        if (abs(tableIndex - firstHit) < abs(tableIndex - secondHit))
          tableIndex = firstHit;
        else
          tableIndex = secondHit;
      }
      // Use table 2
      else {
        distance = 1000000.0;
        for (int i = tableBegin; i < tableMiddle; i++) {
          _distance = abs(table2[i] - processedReading2);
          if (_distance < distance) {
            distance = _distance;
            firstHit = i;
          }
        }
        distance = 1000000.0;
        for (int i = tableMiddle; i <= tableEnd; i++) {
          _distance = abs(table2[i] - processedReading2);
          if (_distance < distance) {
            distance = _distance;
            secondHit = i;
          }
        }
        if (abs(tableIndex - firstHit) < abs(tableIndex - secondHit))
          tableIndex = firstHit;
        else
          tableIndex = secondHit;
      }
    }
    // For when the sensor position is far from critical points (peaks and valleys)
    else {
      distance = 1000000.0;
      for (int i = tableBegin; i < tableMiddle; i++) {
        _position = getPosition(table1[i], table2[i]);
        _distance = abs(_position - processedReading);
        if (_distance < distance) {
          distance = _distance;
          firstHit = i;
        }
      }
      distance = 1000000.0;
      for (int i = tableMiddle; i <= tableEnd; i++) {
        _position = getPosition(table1[i], table2[i]);
        _distance = abs(_position - processedReading);
        if (_distance < distance) {
          distance = _distance;
          secondHit = i;
        }
      }
      if (abs(tableIndex - firstHit) <= abs(tableIndex - secondHit))
        tableIndex = firstHit;
      else
        tableIndex = secondHit;
    }

    _position = ((double)tableIndex - (double)tableBegin) / ((double)tableEnd - (double)tableBegin);
    wheelVelocity = _position - wheelPosition;
    wheelPosition = _position;
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


bool calibrate(void *) {
  // turn off the motor before calibration
  digitalWrite(enablePin, LOW);

  Serial.print("samplesCounter: "); Serial.print(samplesCounter); Serial.print(" ");
  Serial.println("uT");

  int minIndex;
  int maxIndex;
  double minValue;
  double maxValue;
  double minimum = getPosition(1023.0, 1023.0);
  double maximum = getPosition(0.0, 0.0);
  double value;
  double product;

  // First, find the minimum and maximum of the sensor position
  // Search the first third of samples since the data contains 3 revolutions of the wheel
  for (int i = 0; i < samplesNumber - kernelSize; i++) {
    value = getPosition(table1[i], table2[i]);
    if (value < minimum) {
      minimum = value;
      minIndex = i;
    }
    if (value > maximum) {
      maximum = value;
      maxIndex = i;
    }
  }

  // Second, initialize the kernels
  for (int i = 0; i < kernelSize; i++) {
    minKernel[i] = getPosition(table1[minIndex + i - kernelSize / 2], table2[minIndex + i - kernelSize / 2]);
    maxKernel[i] = getPosition(table1[maxIndex + i - kernelSize / 2], table2[maxIndex + i - kernelSize / 2]);
  }

  // Third, find the index of the minimum
  double threshold = 0.1;
  for (int i = 0; i < samplesNumber - kernelSize / 2; i++) {
    product = 0.0;
    for (int j = 0; j < kernelSize; j++) {
      product += abs(minKernel[j] - getPosition(table1[i + j - kernelSize / 2], table2[i + j - kernelSize / 2]));
    }
    if (product < threshold) {
      Serial.print("i1: "); Serial.print(i); Serial.print(" ");
      Serial.println("uT");
      value = product;
      tableBegin = i;
      break;
    }
  }

  // Fourth, find the index of the maximum
  for (int i = tableBegin + 1; i < samplesNumber - kernelSize / 2; i++) {
    product = 0.0;
    for (int j = 0; j < kernelSize; j++) {
      product += abs(maxKernel[j] - getPosition(table1[i + j - kernelSize / 2], table2[i + j - kernelSize / 2]));
    }
    if (product < threshold) {
      Serial.print("i2: "); Serial.print(i); Serial.print(" ");
      Serial.println("uT");
      value = product;
      tableMiddle = i;
      break;
    }
  }

  // Fifth, find the second occurence of the minimum to get a full period
  for (int i = tableMiddle + 1; i < samplesNumber - kernelSize / 2; i++) {
    product = 0.0;
    for (int j = 0; j < kernelSize; j++) {
      product += abs(minKernel[j] - getPosition(table1[i + j- kernelSize / 2], table2[i + j - kernelSize / 2]));
    }
    if (product < threshold) {
      Serial.print("i3: "); Serial.print(i); Serial.print(" ");
      Serial.println("uT");
      value = product;
      tableEnd = i;
      break;
    }
  }

  int halfPeriod1 = tableMiddle - tableBegin;
  int halfPeriod2 = tableEnd - tableMiddle;
  double ratio = (double) halfPeriod1 / (double) halfPeriod2;
  if (halfPeriod1 < samplesNumber / 2 && tableBegin < tableEnd && tableBegin < tableMiddle && tableMiddle < tableEnd && abs(ratio - 1.0) < 0.33)
    calibrated = 1;

  Serial.print("ratio: "); Serial.print(ratio); Serial.print(" ");
  Serial.println("uT");
  return false; // repeat? true
}

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 (or 115200) bits per second:
  Serial.begin(115200);

  // For a moving average we want all of the coefficients to be unity.
  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};
  // Set the coefficients
  fir1.setFilterCoeffs(coef);
  fir2.setFilterCoeffs(coef);

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
  // set the RGB pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(enablePin, HIGH);
  // Run the motor at a constant speed for calibration
  analogWrite(oneAPin, 255);
  analogWrite(twoAPin, 0);

  // call the calculate_velocity function every 100 micros (0.0001 second)
  timer.every(period, readSensors);
  // call calibrate in 5 seconds, but with microsecond resolution
  u_timer.in(5000000, calibrate, "delayed five seconds using microseconds");
}

void loop() {
  // put your main code here, to run repeatedly:
  int _output = 255;
  int dutyCycleOneA = _output;
  int dutyCycleTwoA = 255 - _output;
  float saturation = 100.0;
  float brightness = 100.0;
  int dutyCycleRed = 255;
  int dutyCycleGreen = 255;
  int dutyCycleBlue = 255;

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if (calibrated == 1) {
    double hue = wheelPosition * 360.0;

    hsv hsvColor;
    hsvColor.h = hue;
    hsvColor.s = saturation;
    hsvColor.v = brightness;
    rgb rgbColor = HSVtoRGB(hsvColor);
    dutyCycleRed = 255 - rgbColor.r;
    dutyCycleGreen = 255 - rgbColor.g;
    dutyCycleBlue = 255 - rgbColor.b;

    analogWrite(oneAPin, dutyCycleOneA);
    analogWrite(twoAPin, dutyCycleTwoA);
  }

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    analogWrite(redPin, dutyCycleRed);
    analogWrite(greenPin, dutyCycleGreen);
    analogWrite(bluePin, dutyCycleBlue);
  } else {
    // turn the motor off:
    digitalWrite(enablePin, LOW);
    analogWrite(redPin, 255);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255);
  }

  // print out the value you read:
  if (calibrated == 1) {
    Serial.print("p: "); Serial.print(wheelPosition); Serial.print(" ");
    Serial.print("v: "); Serial.print(wheelVelocity); Serial.print(" ");
    Serial.print("s: "); Serial.print(processedReading); Serial.print(" ");
    Serial.print("i: "); Serial.print(tableIndex); Serial.print(" ");
    Serial.print("b: "); Serial.print(tableBegin); Serial.print(" ");
    Serial.print("m: "); Serial.print(tableMiddle); Serial.print(" ");
    Serial.print("e: "); Serial.print(tableEnd); Serial.print(" ");
    Serial.println("uT");
  }


  timer.tick(); // tick the timer
  u_timer.tick(); // tick the timer
}
