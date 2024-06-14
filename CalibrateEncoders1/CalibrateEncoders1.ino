#include <arduino-timer.h>
#include <SimpleKalmanFilter.h>
#include "ArduPID.h"
#include "MPU9250.h"
#include "Tools.cpp"

// These constants won't change. They're used to give names to the pins used:
const int button1Pin = 22;
const int button2Pin = 23;
const int button3Pin = 24;
const int button4Pin = 25;
const int out1Pin = 18; // for interrupts
const int out2Pin = 19; // for interrupts
const int enable34Pin = 52;
const int enable12Pin = 53;
const int oneAPin = 8;
const int twoAPin = 9;
const int threeAPin = 2;
const int fourAPin = 3;
//const int redPin = 6;
//const int greenPin = 5;
//const int bluePin = 4;
const int period = 5000; // the time interval between sensor readings in microseconds

Orientation phi = {
  23.0, // safetyAngle
  0.0, // angle
  0.0, // dot
  0.0, // dotDot
  0.0, // acc
  0.0, // euler
  0.0, // gyro
  0.0, // _acc
  0.0, // _euler
  0.0, // _gyro
  90.0, // accScale
  -0.49, // eulerScale (negative range)
  -0.32, // eulerScale (positive range)
  1.0, // gyroScale
  0.0, // accOffset
  0.0, // eulerOffset
  0.0, // gyroOffset
  1, // phi axis
  10, // positivePin
  11, // negativePin
  false // updated
};

Orientation theta = {
  23.0, // safetyAngle
  0.0, // angle
  0.0, // dot
  0.0, // dotDot
  0.0, // acc
  0.0, // euler
  0.0, // gyro
  0.0, // _acc
  0.0, // _euler
  0.0, // _gyro
  90.0, // accScale
  0.38, // eulerScale (negative range)
  0.39, // eulerScale (positive range)
  -1.0, // gyroScale
  0.0, // accOffset
  0.0, // eulerOffset
  0.0, // gyroOffset
  2, // theta axis
  13, // positivePin
  12, // negativePin
  false // updated
};

int dutyCycleOneA = 127;
int dutyCycleTwoA = 127;
int dutyCycleThreeA = 127;
int dutyCycleFourA = 127;
int actuatorValue = 0;
int sensorState = 0;
int currentSensorState = 0; // connected to pin A1
int dutyCycleRed = 255;
int dutyCycleGreen = 255;
int dutyCycleBlue = 255;
int button1State = 0; // variable for reading the pushbutton status
int button2State = 0;
int button3State = 0;
int button4State = 0;
int mode = 0;
int buttonCounter = 0;
int maxButtonCount = 20; // for delaying between successive pushes of the buttons
int resetParams = 0;
long unsigned motionCounter = 0;
volatile long pulse;
unsigned long start;
const uint16_t end = 100; // 0.1 second update time
int pps,       // pulses per second
    oldpps,
    ppr = 26, // pulses per revolution
    rpm;

// variables will change:
double reading;
double minReading = 1023.0;
double maxReading = 0.0;
double minReading1 = 1023.0;
double maxReading1 = 0.0;
double velocity = 0.0;
double acceleration = 0.0;
double lowReading = 0.0;
double highReading = 1023.0;
float saturation = 100.0;
float brightness = 100.0;
float hue = 0.0;
double ratio = 0.05;
double k1 = 7.0;
double k2 = 15.0;
double k3 = 11.0;
double k4 = 0.35;
volatile double incrementValue = 0.1;
volatile int printCounter = 0;

ArduPID myController;
double input;
double output;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 11.0;
double kp = 9.0;
double ki = 3.0;
double kd = 0.3;

MPU9250 mpu; // You can also use MPU9255 as is

//float e_mea = 0.01; // Measurement Uncertainty
//float e_est = 0.01; // Estimation Uncertainty
//float q = 0.01; // Process Noise
// SimpleKalmanFilter phiFilter(e_mea, e_est, q);
// SimpleKalmanFilter thetaFilter(e_mea, e_est, q);

// For reading sensors
// Timer<1, micros> timer; // create a timer with 1 task and microsecond resolution

Orientation getOrientation(Orientation input, MPU9250 mpu, double ratio) {
  Orientation output = input;
  if (output.updated) {
    if (output.axis == 1) {
      output._acc = mpu.getAccX();
      output._euler = mpu.getEulerY();
      output._gyro = mpu.getGyroY();
    }
    if (output.axis == 2) {
      output._acc = mpu.getAccY();
      output._euler = mpu.getEulerX();
      //double euler = mpu.getEulerX();
      // Apply reflections because of the orientation of the microchip
      //if (euler < 0)
      //  output._euler = euler - 270.0;
      //else
      //  output._euler = 225.0 + euler;
      output._gyro = mpu.getGyroX();
    }

    output.acc = output.accScale * (output._acc - output.accOffset);
    output.gyro = output.gyroScale * (output._gyro - output.gyroOffset);
    output.dotDot = output.gyro - input.gyro;
    output.dot = output.gyro;

    if (output._euler < 0)
      output.euler = output.eulerScaleNegative * (output._euler - output.eulerOffset);
    else
      output.euler = output.eulerScalePositive * (output._euler - output.eulerOffset);

    output.angle = ratio * output.acc + (1.0 - ratio) * output.euler;
  }
  return output;
}

bool readSensors(void *) {
  double raw = analogRead(A0);
  acceleration = abs(raw - reading) - velocity;
  velocity = abs(raw - reading);
  reading = raw;
  if (reading > maxReading)
    maxReading = reading;
  if (reading < minReading)
    minReading = reading;

  currentSensorState = analogRead(A2);
  if (currentSensorState > maxReading1)
    maxReading1 = currentSensorState;
  if (currentSensorState < minReading1)
    minReading1 = currentSensorState;
  //  motionCounter++;
  //  if (motionCounter > 5000000 / period) {
  //    motionCounter = 0;
  //    setpoint = 1.0;
  //    myController.stop();
  //    myController.reset();
  //    myController.begin(&input, &output, &setpoint, kp, ki, kd);
  //    myController.setOutputLimits(0, 255);
  //    myController.setBias(0);
  //    myController.setWindUpLimits(-10, 10);
  //    myController.start();
  //  }
  //  if (motionCounter == 500000 / period) {
  //    setpoint = 0.0;
  //    myController.stop();
  //    myController.reset();
  //    myController.begin(&input, &output, &setpoint, kp, ki, kd);
  //    myController.setOutputLimits(0, 255);
  //    myController.setBias(0);
  //    myController.setWindUpLimits(-10, 10);
  //    myController.start();
  //  }
  return true; // repeat? true
}

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 (or 115200) bits per second:
  Serial.begin(115200);
  Serial3.begin(9600); // Default communication rate of the Bluetooth module

  Wire.begin(); // for communicating with the IMU over I2C
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

  // set ADC pins A0 and A1 as input
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  // set the button pin as input
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(button4Pin, INPUT_PULLUP);
  // set pins 2 and 3 as outputs:
  pinMode(oneAPin, OUTPUT);
  pinMode(twoAPin, OUTPUT);
  pinMode(threeAPin, OUTPUT);
  pinMode(fourAPin, OUTPUT);
  pinMode(phi.positivePin, OUTPUT);
  pinMode(phi.negativePin, OUTPUT);
  pinMode(theta.positivePin, OUTPUT);
  pinMode(theta.negativePin, OUTPUT);
  // set pin 22 as output:
  pinMode(enable12Pin, OUTPUT);
  pinMode(enable34Pin, OUTPUT);

  pinMode(out1Pin, INPUT_PULLUP);
  pinMode(out2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(out1Pin), readEncoder, FALLING);

  myController.begin(&input, &output, &setpoint, kp, ki, kd);
  myController.setOutputLimits(0, 255);
  myController.setBias(0);
  myController.setWindUpLimits(-10, 10); // Growth bounds for the integral term to prevent integral wind-up
  myController.start();

  delay(1000);

  int mpuUpdate = 0;
  double counter = 0.0;
  for (int i = 0; i < 300; i++) {
    delay(10);
    mpuUpdate = mpu.update();
    if (mpuUpdate) {
      counter += 1.0;
      phi.accOffset += mpu.getAccX();
      phi.eulerOffset += mpu.getEulerY();
      phi.gyroOffset += mpu.getGyroY();
      theta.accOffset += mpu.getAccY();
      theta.euler = mpu.getEulerX();
      theta.gyroOffset += mpu.getGyroX();
    }
  }
  phi.accOffset /= counter;
  phi.eulerOffset /= counter;
  phi.gyroOffset /= counter;
  theta.accOffset /= counter;
  theta.eulerOffset /= counter;
  theta.gyroOffset /= counter;

  // call the calculate_velocity function every 100 micros (0.0001 second)
  // timer.every(period, readSensors);
}


void loop() {
  // put your main code here, to run repeatedly:
  // read the state of the pushbutton value:
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);
  button3State = digitalRead(button3Pin);
  button4State = digitalRead(button4Pin);
  buttonCounter++;
  if (button1State == LOW && buttonCounter > maxButtonCount) {
    buttonCounter = 0;
    mode++;
    if (mode > 10)
      mode = 0;
  }
  if (button2State == LOW && buttonCounter > maxButtonCount) {
    buttonCounter = 0;
    if (mode == 1)
      k1 += incrementValue;
    if (mode == 2)
      k2 += incrementValue;
    if (mode == 3)
      k3 += incrementValue;
    if (mode == 4)
      k4 += incrementValue;
    if (mode == 5)
      kp += incrementValue;
    if (mode == 6)
      ki += incrementValue;
    if (mode == 7)
      kd += incrementValue;
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
  if (button3State == LOW && buttonCounter > 10) {
    buttonCounter = 0;
    if (mode == 1)
      k1 -= incrementValue;
    if (mode == 2)
      k2 -= incrementValue;
    if (mode == 3)
      k3 -= incrementValue;
    if (mode == 4)
      k4 -= incrementValue;
    if (mode == 5)
      kp -= incrementValue;
    if (mode == 6)
      ki -= incrementValue;
    if (mode == 7)
      kd -= incrementValue;
    if (k1 < 0)
      k1 = 0.0;
    if (k2 < 0)
      k2 = 0.0;
    if (k3 < 0)
      k3 = 0.0;
    if (k4 < 0)
      k4 = 0.0;
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
    myController.setWindUpLimits(-10, 10);
    myController.start();
    resetParams = 0;
  }

  //  if (Serial3.available() > 0) { // Checks whether data is comming from the serial port
  //    state = Serial3.read(); // Reads the data from the serial port
  //  }

  phi.updated = mpu.update();
  theta.updated = phi.updated;
  phi = getOrientation(phi, mpu, 1.0 - ratio);
  theta = getOrientation(theta, mpu, 1.0 - ratio);

//  hue = (reading - minReading) / (maxReading - minReading) * 360.0;
//  Hsv hsvColor;
//  hsvColor.h = hue;
//  hsvColor.s = saturation;
//  hsvColor.v = brightness;
//  Rgb rgbColor = HSVtoRGB(hsvColor);
//  dutyCycleRed = 255 - rgbColor.r;
//  dutyCycleGreen = 255 - rgbColor.g;
//  dutyCycleBlue = 255 - rgbColor.b;
//  analogWrite(redPin, dutyCycleRed);
//  analogWrite(greenPin, dutyCycleGreen);
//  analogWrite(bluePin, dutyCycleBlue);

  if(millis() - start > end) {
     start += end;
     noInterrupts();
     pps = pulse - oldpps;
     oldpps = pulse;
     interrupts();
     rpm = abs(pps * 60L / ppr);
  }

  velocity = (float)rpm;

  // ui state
  if (phi.updated) {
//    if (phi.angle >= 0.0) {
//      analogWrite(phi.positivePin, abs(phi.angle / phi.safetyAngle) * 255.0);
//      analogWrite(phi.negativePin, 0);
//    } else {
//      analogWrite(phi.positivePin, 0);
//      analogWrite(phi.negativePin, abs(phi.angle / phi.safetyAngle) * 255.0);
//    }
    actuatorValue = max(0, min(255, abs(k1 * phi.angle + k2 * phi.dot + k3 * phi.dotDot) + k4 * velocity));
    if (phi.angle >= 0) {
      dutyCycleThreeA = 0;
      dutyCycleFourA = actuatorValue;
    }
    else {
      dutyCycleThreeA = actuatorValue;
      dutyCycleFourA = 0;
    }

    if (theta.angle >= 0) {
      dutyCycleOneA = 255 - output;
      dutyCycleTwoA = output;
    }
    else {
      dutyCycleOneA = output;
      dutyCycleTwoA = 255 - output;
    }
  }
  if (theta.updated) {
//    if (theta.angle >= 0.0) {
//      analogWrite(theta.positivePin, abs(theta.angle / theta.safetyAngle) * 255.0);
//      analogWrite(theta.negativePin, 0);
//    } else {
//      analogWrite(theta.positivePin, 0);
//      analogWrite(theta.negativePin, abs(theta.angle / theta.safetyAngle) * 255.0);
//    }
    input = theta.angle; // Replace with sensor feedback
    myController.compute();
    dutyCycleOneA = output;
    dutyCycleTwoA = 255 - output;
  }

  // motor actions
  if (abs(phi.angle) < phi.safetyAngle && abs(theta.angle) < theta.safetyAngle) { // Added safety check for too large angles
    analogWrite(oneAPin, dutyCycleOneA);
    analogWrite(twoAPin, dutyCycleTwoA);
    analogWrite(threeAPin, dutyCycleThreeA);
    analogWrite(fourAPin, dutyCycleFourA);
    digitalWrite(enable12Pin, HIGH);
    if (button4State == HIGH)
      digitalWrite(enable34Pin, HIGH);
  }
  else {
    digitalWrite(enable12Pin, LOW);
    digitalWrite(enable34Pin, LOW);
  }
  

//  if (button4State == HIGH) {
//    analogWrite(threeAPin, 128);
//    analogWrite(fourAPin, 0);
//    digitalWrite(enable34Pin, HIGH);
//  } else {
//    analogWrite(threeAPin, 0);
//    analogWrite(fourAPin, 255);
//    digitalWrite(enable34Pin, HIGH);
//  }

//  Serial3.print("output1: "); Serial3.print(analogRead(A0)); Serial3.print(" ");
//  Serial3.print("output2: "); Serial3.print(analogRead(A1)); Serial3.print(" ");
//  if (pps > 5) {
//    Serial3.print("pulse: "); Serial3.print(pulse); Serial3.print(" ");
//    Serial3.print("rpm: "); Serial3.print(rpm); Serial3.print(" ");
//  }
//  Serial3.print("button4State: "); Serial3.print(button4State); Serial3.print(" ");
//  Serial3.println("uT");
  ++printCounter;
  // print out the value you read:
  if ((phi.updated || theta.updated) && ((printCounter % 5) == 0)) {
    printCounter = 0;

    if (mode == 1) {
      Serial3.print("k1*: "); Serial3.print(k1); Serial3.print(" ");
      //Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
      //Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      //Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
      //Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
      //Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
      //Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 2) {
      //Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
      Serial3.print("k2*: "); Serial3.print(k2); Serial3.print(" ");
      //Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      //Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
//      Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
//      Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
//      Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 3) {
      //Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
      //Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
      Serial3.print("k3*: "); Serial3.print(k3); Serial3.print(" ");
      //Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
//      Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
//      Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
//      Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 4) {
//      Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
//      Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
//      Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      Serial3.print("k4*: "); Serial3.print(k4); Serial3.print(" ");
//      Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
//      Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
//      Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 5) {
      Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
      Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
      Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
      Serial3.print("kp*: "); Serial3.print(kp); Serial3.print(" ");
      Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
      Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 6) {
      Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
      Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
      Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
      Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
      Serial3.print("ki*: "); Serial3.print(ki); Serial3.print(" ");
      Serial3.print("kd: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 7) {
      Serial3.print("k1: "); Serial3.print(k1); Serial3.print(" ");
      Serial3.print("k2: "); Serial3.print(k2); Serial3.print(" ");
      Serial3.print("k3: "); Serial3.print(k3); Serial3.print(" ");
      Serial3.print("k4: "); Serial3.print(k4); Serial3.print(" ");
      Serial3.print("kp: "); Serial3.print(kp); Serial3.print(" ");
      Serial3.print("ki: "); Serial3.print(ki); Serial3.print(" ");
      Serial3.print("kd*: "); Serial3.print(kd); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 8) {

      Serial3.print("acc: "); Serial3.print(phi.acc); Serial3.print(" ");
      Serial3.print("_acc: "); Serial3.print(phi._acc); Serial3.print(" ");
      Serial3.print("accOffset: "); Serial3.print(phi.accOffset); Serial3.print(" ");
      Serial3.print("accScale: "); Serial3.print(phi.accScale); Serial3.print(" ");

      Serial3.print("euler: "); Serial3.print(phi.euler); Serial3.print(" ");
      Serial3.print("_euler: "); Serial3.print(phi._euler); Serial3.print(" ");
      Serial3.print("eulerOffset: "); Serial3.print(phi.eulerOffset); Serial3.print(" ");
      Serial3.print("eulerScaleNegative: "); Serial3.print(phi.eulerScaleNegative); Serial3.print(" ");
      Serial3.print("eulerScalePositive: "); Serial3.print(phi.eulerScalePositive); Serial3.print(" ");

      Serial3.print("gyro: "); Serial3.print(phi.gyro); Serial3.print(" ");
      Serial3.print("_gyro: "); Serial3.print(phi._gyro); Serial3.print(" ");
      Serial3.print("gyroOffset: "); Serial3.print(phi.gyroOffset); Serial3.print(" ");
      Serial3.print("gyroScale: "); Serial3.print(phi.gyroScale); Serial3.print(" ");

      Serial3.println("uT");
    }
    if (mode == 9) {

      Serial3.print("acc: "); Serial3.print(theta.acc); Serial3.print(" ");
      Serial3.print("_acc: "); Serial3.print(theta._acc); Serial3.print(" ");
      Serial3.print("accOffset: "); Serial3.print(theta.accOffset); Serial3.print(" ");
      Serial3.print("accScale: "); Serial3.print(theta.accScale); Serial3.print(" ");

      Serial3.print("euler: "); Serial3.print(theta.euler); Serial3.print(" ");
      Serial3.print("_euler: "); Serial3.print(theta._euler); Serial3.print(" ");
      Serial3.print("eulerOffset: "); Serial3.print(theta.eulerOffset); Serial3.print(" ");
      Serial3.print("eulerScaleNegative: "); Serial3.print(theta.eulerScaleNegative); Serial3.print(" ");
      Serial3.print("eulerScalePositive: "); Serial3.print(theta.eulerScalePositive); Serial3.print(" ");

      Serial3.print("gyro: "); Serial3.print(theta.gyro); Serial3.print(" ");
      Serial3.print("_gyro: "); Serial3.print(theta._gyro); Serial3.print(" ");
      Serial3.print("gyroOffset: "); Serial3.print(theta.gyroOffset); Serial3.print(" ");
      Serial3.print("gyroScale: "); Serial3.print(theta.gyroScale); Serial3.print(" ");
      Serial3.println("uT");
    }
    if (mode == 10) {
      Serial3.print("phi: "); Serial3.print(phi.angle); Serial3.print(" ");
      Serial3.print("phiDot: "); Serial3.print(phi.dot); Serial3.print(" ");
      Serial3.print("phiDotDot: "); Serial3.print(phi.dotDot); Serial3.print(" ");
      Serial3.print("theta: "); Serial3.print(theta.angle); Serial3.print(" ");
      Serial3.print("thetaDot: "); Serial3.print(theta.dot); Serial3.print(" ");
      Serial3.print("thetaDotDot: "); Serial3.print(theta.dotDot); Serial3.print(" ");
      Serial3.print("velocity: "); Serial3.print(velocity); Serial3.print(" ");
      Serial3.print("actuator: "); Serial3.print(actuatorValue); Serial3.print(" ");
      Serial3.print("output: "); Serial3.print(output); Serial3.print(" ");
      Serial3.println("uT");
    }
  }

  // timer.tick(); // tick the timer
}

void readEncoder() // ISR
{
  ++pulse; // rotation direction
}  
