#include <arduino-timer.h>
#include <FIR.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h>
#include "MPU9250.h"
#include "ArduPID.h"
#include <SoftwareSerial.h>
#include "Tools.cpp"

// These constants won't change. They're used to give names to the pins used:
const int button1Pin = 23;
const int button2Pin = 25;
const int button3Pin = 27;
const int actuatorPin = 13;
const int enable34Pin = 52;
const int enable12Pin = 53;
const int oneAPin = 8;
const int twoAPin = 9;
const int threeAPin = 2;
const int fourAPin = 3;
const int redPin = 6;
const int greenPin = 5;
const int bluePin = 4;
const int period = 1000; // the time interval between sensor readings in microseconds

int mpuUpdate = 0;
int dutyCycleOneA = 127;
int dutyCycleTwoA = 127;
int dutyCycleThreeA = 127;
int dutyCycleFourA = 127;
int actuatorValue = 0;
int sensorState = 0;
int currentSensorState = 0; // connected to pin A1
int counter = 0;
int dutyCycleRed = 255;
int dutyCycleGreen = 255;
int dutyCycleBlue = 255;
int button1State = 0; // variable for reading the pushbutton status
int button2State = 0;
int button3State = 0;
int mode = 0;
int buttonCounter = 0;
int maxButtonCount = 5;
int resetParams = 0;
long unsigned int countPulses = 0;

// variables will change:
double reading;
double processedReading1 = 0.0;
double processedReading2 = 0.0;
double processedReading = 0.0;
double processedReadingVelocity = 0.0;
double processedReadingVelocity1 = 0.0;
double processedReadingVelocity2 = 0.0;
double phi = 0.0;
double phiDot = 0.0;
double phiDotDot = 0.0;
double theta = 0.0;
double thetaDot = 0.0;
double thetaDotDot = 0.0;
double phiGyro = 0.0;
double thetaGyro = 0.0;
double velocity = 0.0;
double acceleration = 0.0;
double wheelSpeed = 0.0;
double phiAccOffset = 0.0;
double thetaAccOffset = 0.0;
double phiGyroOffset = 0.0;
double thetaGyroOffset = 0.0;
double phiAcc = 0.0;
double thetaAcc = 0.0;
double lowReading = 0.0;
double highReading = 1023.0;
float saturation = 100.0;
float brightness = 100.0;
float hue = 0.0;
float safetyAnglePhi = 20.0;
float safetyAngleTheta = 35.0;
double k1 = 38.0;
double k2 = 150.0;
double k3 = 450.0;
double k4 = 24.0;

ArduPID myController;
double input;
double output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 0.0;
double kp = 30.0;
double ki = 1.0;
double kd = 1.0;

SoftwareSerial EEBlue(15, 14); // RX | TX

MPU9250 mpu; // You can also use MPU9255 as is

float e_mea = 0.01; // Measurement Uncertainty
float e_est = 0.01; // Estimation Uncertainty
float q = 0.01; // Process Noise
SimpleKalmanFilter phiKalmanFilter(e_mea, e_est, q);
SimpleKalmanFilter thetaKalmanFilter(e_mea, e_est, q);

// For reading sensors
Timer<1, micros> timer; // create a timer with 1 task and microsecond resolution

Servo myServo;  // create servo object to control a servo
FIR<float, 4> fir;
// FIR<float, 8> fir2;

bool readSensors(void *) {
  // double reading = fir.processReading(analogRead(A0));
  // double reading2 = fir2.processReading(analogRead(A1));
  //processedReadingVelocity1 = reading1 - processedReading1;
  // processedReadingVelocity2 = reading2 - processedReading2;
  //processedReading = reading;
  //processedReading2 = reading2;
  // double reading = getPosition(processedReading1, processedReading2);

  // processedReadingVelocity = reading - processedReading;
  // processedReading = reading;
  double raw = analogRead(A0);
  velocity = abs(raw - reading);
  reading = raw;
  currentSensorState = analogRead(A1);
//  if (abs(reading - lowReading) < abs(reading - highReading)) {
//    lowReading = 0.5 * reading + 0.5 * lowReading;
//    sensorState = 0;
//  }
//  else {
//    highReading = 0.5 * reading + 0.5 * highReading;
//    if (sensorState == 0) {
//      sensorState = 1;
//      countPulses++;
//    }
//  }
//  counter++;
//  if (counter > 30) {
//    acceleration = velocity - countPulses;
//    velocity = countPulses;
//    counter = 0;
//    countPulses = 0;
//  }
  return true; // repeat? true
}


void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 (or 115200) bits per second:
  Serial.begin(115200);
  EEBlue.begin(9600); // Default communication rate of the Bluetooth module

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
  fir.setFilterCoeffs(coef);
  // fir2.setFilterCoeffs(coef);

  // set ADC pins A0 and A1 as input
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  // set the button pin as input
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  // digitalWrite(buttonPin, HIGH);
  // set pins 2 and 3 as outputs:
  pinMode(oneAPin, OUTPUT);
  pinMode(twoAPin, OUTPUT);
  pinMode(threeAPin, OUTPUT);
  pinMode(fourAPin, OUTPUT);
  // set pin 22 as output:
  pinMode(enable12Pin, OUTPUT);
  pinMode(enable34Pin, OUTPUT);
  pinMode(actuatorPin, OUTPUT);

  myController.begin(&input, &output, &setpoint, kp, ki, kd);
  //myController.reverse();               // Uncomment if controller output is "reversed"
  //myController.setSampleTime(7);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  //myController.setOutputLimits(1000, 2000);
  //myController.setBias(500);
  myController.setOutputLimits(0, 255);
  myController.setBias(50);
  myController.setWindUpLimits(-100, 100); // Growth bounds for the integral term to prevent integral wind-up
  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)

  //myServo.attach(actuatorPin);  // attaches the servo on actuatorPin1A to the servo object
  //myServo.writeMicroseconds(actuatorValue); // sets the servo position according to the scaled value
  // myServo.write(actuatorValue);
  delay(1000);

  double counter = 0.0;
  int mpuUpdate = 0;
  for (int i = 0; i < 300; i++) {
    mpuUpdate = mpu.update();
    delay(10);
    if (mpuUpdate) {
      counter += 1.0;
      phiAccOffset += mpu.getAccX();
      thetaAccOffset += mpu.getAccY();
      phiGyroOffset += mpu.getEulerY();
      thetaGyroOffset += mpu.getEulerX();
    }
  }
  phiAccOffset /= counter;
  thetaAccOffset /= counter;
  phiGyroOffset /= counter;
  thetaGyroOffset /= counter;

  // call the calculate_velocity function every 100 micros (0.0001 second)
  timer.every(period, readSensors);
}


void loop() {
  // put your main code here, to run repeatedly:
  // read the state of the pushbutton value:
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);
  button3State = digitalRead(button3Pin);
  buttonCounter++;
  if (button1State == HIGH && buttonCounter > maxButtonCount) {
    buttonCounter = 0;
    mode++;
    if (mode > 7)
      mode = 0;
  }
  if (button2State == HIGH && buttonCounter > maxButtonCount) {
    buttonCounter = 0;
    if (mode == 1)
      k1 += 1.0;
    if (mode == 2)
      k2 += 1.0;
    if (mode == 3)
      k3 += 1.0;
    if (mode == 4)
      k4 += 1.0;
    if (mode == 5)
      kp += 1.0;
    if (mode == 6)
      ki += 1.0;
    if (mode == 7)
      kd += 1.0;
    if (k1 > 1000.0)
      k1 = 1000.0;
    if (k2 > 1000)
      k2 = 1000.0;
    if (k3 > 1000)
      k3 = 1000.0;
    if (k4 > 1000)
      k4 = 1000.0;
    if (kp > 1000)
      kp = 1000.0;
    if (ki > 1000)
      ki = 1000.0;
    if (kd > 1000)
      kd = 1000.0;
    if (mode == 5 || mode == 6 || mode == 7)
      resetParams = 1;
  }
  if (button3State == HIGH && buttonCounter > 10) {
    buttonCounter = 0;
    if (mode == 1)
      k1 -= 1.0;
    if (mode == 2)
      k2 -= 1.0;
    if (mode == 3)
      k3 -= 1.0;
    if (mode == 4)
      k4 -= 1.0;
    if (mode == 5)
      kp -= 1.0;
    if (mode == 6)
      ki -= 1.0;
    if (mode == 7)
      kd -= 1.0;
    if (k1 < 0)
      k1 = 0;
    if (k2 < 0)
      k2 = 0;
    if (k3 < 0)
      k3 = 0;
    if (k4 < 0)
      k4 = 0;
    if (kp < 0)
      kp = 0.0;
    if (ki < 0)
      ki = 0.0;
    if (kd < 0)
      kd = 0.0;
    if (mode == 5 || mode == 6 || mode == 7)
      resetParams = 1;
  }

  if (resetParams == 1) {
    myController.stop();
    myController.reset();
    myController.begin(&input, &output, &setpoint, kp, ki, kd);
    myController.setOutputLimits(0, 255);
    myController.setBias(0);
    myController.setWindUpLimits(-20, 20);
    myController.start();
    resetParams = 0;
  }

  //  if (EEBlue.available() > 0) { // Checks whether data is comming from the serial port
  //    state = EEBlue.read(); // Reads the data from the serial port
  //  }

  mpuUpdate = mpu.update();
  if (mpuUpdate) {
    phiAcc = phiKalmanFilter.updateEstimate(mpu.getAccX() - phiAccOffset);
    phiGyro = mpu.getEulerY() * 3.14;
    double fusedPhi = 90.0 * phiAcc;// 0.95 * (90.0 * phiAcc) + 0.05 * phiGyro;
    phiDotDot = (fusedPhi - phi) - phiDot;
    phiDot = fusedPhi - phi;
    phi = fusedPhi;

    thetaAcc = thetaKalmanFilter.updateEstimate(mpu.getAccY() - thetaAccOffset);
    thetaGyro = 180.0 - mpu.getEulerX();
    if (thetaGyro > 180) {
      thetaGyro -= 360.0;
      thetaGyro /= 3.14;
    }
    double fusedTheta = 0.05 * (90.0 * thetaAcc) * -0.95 * thetaGyro;;
    thetaDotDot = (fusedTheta - theta) - thetaDot;
    thetaDot = fusedTheta - theta;
    theta = fusedTheta;

    setpoint = 0.0;
    input = theta; // Replace with sensor feedback
    myController.compute();

    hue = abs(theta) / safetyAngleTheta * 360.0;
    hsv hsvColor;
    hsvColor.h = hue;
    hsvColor.s = saturation;
    hsvColor.v = brightness;
    rgb rgbColor = HSVtoRGB(hsvColor);
    dutyCycleRed = 255 - rgbColor.r;
    dutyCycleGreen = 255 - rgbColor.g;
    dutyCycleBlue = 255 - rgbColor.b;
    analogWrite(redPin, dutyCycleRed);
    analogWrite(greenPin, dutyCycleGreen);
    analogWrite(bluePin, dutyCycleBlue);

    actuatorValue = max(0, min(255, abs(k1 * phi + k2 * phiDot + k3 * phiDotDot + k4 * velocity)));
    if (abs(phi) < safetyAnglePhi && abs(theta) < safetyAngleTheta) { // Added safety check for too large angles
      digitalWrite(enable12Pin, HIGH);
      digitalWrite(enable34Pin, HIGH);

      if (phi >= 0) {
        dutyCycleThreeA = 0;
        dutyCycleFourA = actuatorValue;
      }
      else {
        dutyCycleThreeA = actuatorValue;
        dutyCycleFourA = 0;
      }

      dutyCycleOneA = 255 - output;
      dutyCycleTwoA = output;

      analogWrite(oneAPin, dutyCycleOneA);
      analogWrite(twoAPin, dutyCycleTwoA);
      analogWrite(threeAPin, dutyCycleThreeA);
      analogWrite(fourAPin, dutyCycleFourA);
    }
    else {
      digitalWrite(enable12Pin, LOW);
      digitalWrite(enable34Pin, LOW);
    }
  }

  // print out the value you read:
  if (mpuUpdate == 1) {
    //    Serial.print("v0: "); Serial.print(analogRead(A0)); Serial.print(" ");
    //    Serial.print("v1: "); Serial.print(value); Serial.print(" ");
    //    Serial.print("v2: "); Serial.print(valueDot); Serial.print(" ");
    //    Serial.print("v3: "); Serial.print(valueDotDot); Serial.print(" ");
    //    Serial.print("v4: "); Serial.print(velocity); Serial.print(" ");
    //    Serial.print("act: "); Serial.print(actuatorValue); Serial.print(" ");
    //    Serial.print("pitch: "); Serial.print(value); Serial.print(" ");
    //    Serial.print("pitchD: "); Serial.print(valueDot); Serial.print(" ");
    //    Serial.print("enc: "); Serial.print(velocity); Serial.print(" ");
    //    Serial.print("curr: "); Serial.print(currentSensorState); Serial.print(" ");
    //    Serial.println("uT");

    if (mode == 1) {
      EEBlue.print("k1*: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 2) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2*: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 3) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3*: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 4) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4*: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 5) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp*: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 6) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki*: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode == 7) {
      EEBlue.print("k1: "); EEBlue.print(k1); EEBlue.print(" ");
      EEBlue.print("k2: "); EEBlue.print(k2); EEBlue.print(" ");
      EEBlue.print("k3: "); EEBlue.print(k3); EEBlue.print(" ");
      EEBlue.print("k4: "); EEBlue.print(k4); EEBlue.print(" ");
      EEBlue.print("kp: "); EEBlue.print(kp); EEBlue.print(" ");
      EEBlue.print("ki: "); EEBlue.print(ki); EEBlue.print(" ");
      EEBlue.print("kd*: "); EEBlue.print(kd); EEBlue.print(" ");
    }
    if (mode != 0) {
      // EEBlue.print("phi: "); EEBlue.print(phiGyro); EEBlue.print(" ");
      //EEBlue.print("phiOffset: "); EEBlue.print(phiGyro + phiGyroOffset); EEBlue.print(" ");
      // EEBlue.print("theta: "); EEBlue.print(thetaGyro); EEBlue.print(" ");
      //EEBlue.print("thetaOffset: "); EEBlue.print(thetaGyro + thetaGyroOffset); EEBlue.print(" ");
      EEBlue.print("phi: "); EEBlue.print(phi); EEBlue.print(" ");
      EEBlue.print("phiDot: "); EEBlue.print(phiDot); EEBlue.print(" ");
      EEBlue.print("phiDotDot: "); EEBlue.print(phiDotDot); EEBlue.print(" ");
      EEBlue.print("theta: "); EEBlue.print(theta); EEBlue.print(" ");
      EEBlue.print("thetaDot: "); EEBlue.print(thetaDot); EEBlue.print(" ");
      EEBlue.print("thetaDotDot: "); EEBlue.print(thetaDotDot); EEBlue.print(" ");
      EEBlue.print("velocity: "); EEBlue.print(velocity); EEBlue.print(" ");
      EEBlue.print("actuatorValue: "); EEBlue.print(actuatorValue); EEBlue.print(" ");
      EEBlue.print("output: "); EEBlue.print(output); EEBlue.print(" ");
      EEBlue.println("uT");
    }
  }

  timer.tick(); // tick the timer
}
