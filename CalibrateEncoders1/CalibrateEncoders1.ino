#include <arduino-timer.h>
#include <FIR.h>
#include <SimpleKalmanFilter.h>
#include "MPU9250.h"
#include "ArduPID.h"
#include "Tools.cpp"

// These constants won't change. They're used to give names to the pins used:
const int oneAPin = 2;
const int twoAPin = 3;
const int enablePin = 22;
const int buttonPin = 26;
const int redPin = 4;
const int greenPin = 5;
const int bluePin = 6;

const int samplesNumber = 550; // the total number of raw pints in tables
const int period = 1000; // the time interval between sensor readings in microseconds
const int kernelSize = 2; // the kernel size of convolutional filters

// variables will change:
int calibrated = 1;
int tableIndex = 0; // the current index of the table
int tableBegin = 0;
int tableEnd = samplesNumber;
int countdown = 32; // for discarding at least 8 points so that the moving average stabilizes
int samplesCounter = 0; // for keeping track of the number of recorded points in tables
int buttonState = 0; // variable for reading the pushbutton status

double table1[samplesNumber] = {};
double table2[samplesNumber] = {};
double minKernel[kernelSize] = {};
double maxKernel[kernelSize] = {};
double processedReading1 = 0.0;
double processedReading2 = 0.0;
double processedReading = 0.0;
double processedReadingVelocity = 0.0;
double wheelPosition = 0.0;
double wheelVelocity = 0.0;
double pitchValue = 0.0;
double pitchVelocity = 0.0;
double gyroPitchOffset = 5.0;
double accXPitchOffset = 0.08;
double accYPitchOffset = -0.01;
double accZPitchOffset = 0.95;
double pitchValueGyro = 0.0;
double pitchValueAcc = 0.0;

// the reaction wheel controller's parameters
double k1 = 10.0;
double k2 = 1.0;
double k3 = 1.0;

ArduPID myController;
double input;
double output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 127.0;
double p = 200.0;
double i = 0.01;
double d = 300.0;

MPU9250 mpu; // You can also use MPU9255 as is

double e_mea = 0.1; // Measurement Uncertainty
double e_est = 0.1; // Estimation Uncertainty
double q = 0.01; // Process Noise
SimpleKalmanFilter simpleKalmanFilter(e_mea, e_est, q);
double e_mea1 = 0.1; // Measurement Uncertainty
double e_est1 = 0.1; // Estimation Uncertainty
double q1 = 0.01; // Process Noise
SimpleKalmanFilter simpleKalmanFilter1(e_mea1, e_est1, q1);

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
  processedReadingVelocity = reading - processedReading;
  processedReading = reading;
  /*
    if (calibrated == 1) {
      int firstHit = tableBegin;
      double distance = 1000000.0;
      double _distance;
      double _position;

      for (int i = tableBegin; i <= tableEnd; i++) {
        _position = getPosition(table1[i], table2[i]);
        _distance = abs(_position - processedReading);
        if (_distance < distance) {
          distance = _distance;
          tableIndex = i;
        }
      }

      _position = ((double)tableIndex - (double)tableBegin) / ((double)tableEnd - (double)tableBegin);
      wheelVelocity = _position - wheelPosition;
      wheelPosition = _position;
    }
  */
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
  double threshold = 0.01;
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
      tableEnd = i;
      break;
    }
  }

  int halfPeriod = tableEnd - tableBegin;
  if (halfPeriod < samplesNumber / 2 && tableBegin < tableEnd)
    calibrated = 1;

  return false; // repeat? true
}

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 (or 115200) bits per second:
  Serial.begin(115200);

  Wire.begin();
  delay(1000);

  mpu.setup(0x68);  // change to your own address

  delay(2000);

  // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  /*
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();
  */
  // print_calibration();
  mpu.verbose(false);

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
  digitalWrite(buttonPin, HIGH);
  // set the RGB pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(enablePin, HIGH);
  // Run the motor at a constant speed for calibration
  analogWrite(oneAPin, 127);
  analogWrite(twoAPin, 0);

  myController.begin(&input, &output, &setpoint, p, i, d);

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(0, 255);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up

  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)

  // call the calculate_velocity function every 100 micros (0.0001 second)
  timer.every(period, readSensors);
  // call calibrate in 5 seconds, but with microsecond resolution
  // u_timer.in(7000000, calibrate, "delayed five seconds using microseconds");
}

void loop() {
  // put your main code here, to run repeatedly:
  int mpuUpdate = mpu.update();
  if (mpuUpdate) {
    double xBuffer = mpu.getAccX() - accXPitchOffset;
    double yBuffer = mpu.getAccY() - accYPitchOffset;
    double zBuffer = mpu.getAccZ() - accZPitchOffset;
    double PitchBuffer = mpu.getPitch() - gyroPitchOffset;
    //double PitchBuffer = simpleKalmanFilter1.updateEstimate(mpu.getPitch() - gyroPitchOffset); //calculate the high-pass signal 
    //double measuredValue = atan2((- xBuffer) , sqrt(yBuffer * yBuffer + zBuffer * zBuffer)) * 57.3;
    double estimatedValue = xBuffer;
    // double estimatedValue = simpleKalmanFilter.updateEstimate(measuredValue);
    pitchValueAcc = estimatedValue;
    pitchValueGyro = PitchBuffer;
    double _pitchValue = pitchValueAcc;// - 0.05 * pitchValueGyro;
    pitchVelocity = _pitchValue - pitchValue;
    pitchValue = _pitchValue;
  }
/*
  // variate target angle
  double ANGLE_FIXRATE = 1.0;
  if (127.0 + pitchValue < setpoint) {
    setpoint -= ANGLE_FIXRATE;
  }
  else {
    setpoint += ANGLE_FIXRATE;
  }
  setpoint = min(255.0, max(0.0, setpoint));
  */
  setpoint = 127.0;
  input = 127.0 + k1 * pitchValue + k2 * pitchVelocity + k3 * processedReadingVelocity; // Replace with sensor feedback
  // output = k1 * processedReadingVelocity + k2 * pitchValue + k3 * pitchVelocity;
  myController.compute();
  int dutyCycleOneA = 255 - output;
  int dutyCycleTwoA = output;
  float saturation = 100.0;
  float brightness = 100.0;
  int dutyCycleRed = 255;
  int dutyCycleGreen = 255;
  int dutyCycleBlue = 255;

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if (calibrated == 1) {
    double hue = pitchValue + 0.5 * 360.0;

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
  // Added safety check for too large angles
  if (buttonState == LOW && abs(pitchValue) < 0.25) {
    digitalWrite(enablePin, HIGH);
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

  if (calibrated == 1 && mpuUpdate == 1) {
    //Serial.print("AccX: "); Serial.print(mpu.getAccX() - accXPitchOffset); Serial.print(" ");
    //Serial.print("AccY: "); Serial.print(mpu.getAccY() - accYPitchOffset); Serial.print(" ");
    //Serial.print("AccZ: "); Serial.print(mpu.getAccZ() - accZPitchOffset); Serial.print(" ");
    //Serial.print("Gyro: "); Serial.print(mpu.getPitch() - gyroPitchOffset); Serial.print(" ");
    Serial.print("pitch: "); Serial.print(pitchValue); Serial.print(" ");
    //Serial.print("pitchAcc: "); Serial.print(pitchValueAcc); Serial.print(" ");
    //Serial.print("pitchGyro: "); Serial.print(pitchValueGyro); Serial.print(" ");
    Serial.print("input: "); Serial.print(input); Serial.print(" ");
    Serial.print("setpoint: "); Serial.print(setpoint); Serial.print(" ");
    Serial.print("output: "); Serial.print(output); Serial.print(" ");
    //Serial.print("pos: "); Serial.print(wheelPosition); Serial.print(" ");
    //Serial.print("vel: "); Serial.print(wheelVelocity); Serial.print(" ");
    //Serial.print("pitch: "); Serial.print(pitchValue); Serial.print(" ");
    //Serial.print("pitchVel: "); Serial.print(pitchVelocity); Serial.print(" ");
    //Serial.print("s: "); Serial.print(processedReading); Serial.print(" ");
    //Serial.print("i: "); Serial.print(tableIndex); Serial.print(" ");
    //Serial.print("b: "); Serial.print(tableBegin); Serial.print(" ");
    //Serial.print("e: "); Serial.print(tableEnd); Serial.print(" ");
    Serial.println("uT");
  }
  /*
    Serial.print("i: "); Serial.print(input); Serial.print(" ");
    Serial.print("s: "); Serial.print(setpoint); Serial.print(" ");
    Serial.print("o: "); Serial.print(output); Serial.print(" ");
    Serial.println("uT");
  */
  /*
    double getAccBiasX = mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    double getAccBiasY = mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    double getAccBiasZ = mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
    Serial.print("getAccBiasX: "); Serial.print(getAccBiasX); Serial.print(" ");
    Serial.print("getAccBiasY: "); Serial.print(getAccBiasY); Serial.print(" ");
    Serial.print("getAccBiasZ: "); Serial.print(getAccBiasZ); Serial.print(" ");
  */
  /*
    Serial.print("Pitch: "); Serial.print(pitchValueGyro); Serial.print(" ");
    Serial.print("Pitch: "); Serial.print(pitchValueAcc); Serial.print(" ");
    Serial.print("Pitch: "); Serial.print(pitchValue); Serial.print(" ");
    Serial.println("uT");
  */

  timer.tick(); // tick the timer
  u_timer.tick(); // tick the timer
}
