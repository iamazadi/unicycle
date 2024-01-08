// This file includes tools for converting between color spaces

#include "math.h"

typedef struct {
  float r;       // a fraction between 0 and 1
  float g;       // a fraction between 0 and 1
  float b;       // a fraction between 0 and 1
} rgb;

typedef struct {
  float h;       // angle in degrees
  float s;       // a fraction between 0 and 1
  float v;       // a fraction between 0 and 1
} hsv;

static rgb HSVtoRGB(hsv in);
static double getPosition(double a, double b);
static int getPulse(int x, int width, int minWidth);
static bool calibrate(int tableBegin, int tableEnd, double * table1, double * table2, int samplesNumber, double * minKernel, double * maxKernel, int kernelSize);

double getPosition(double a, double b) {
  return sqrt(pow(a, 2) + pow(b, 2));
}

int getPulse(int x, int width, int minWidth) {
  float fraction = (x >= 20) ? x / 180.0 : 20 / 180.0;
  int z = minWidth + (int)(width * fraction);
  return z; // return the value
}

bool calibrate(int tableBegin, int tableEnd, double * table1, double * table2, int samplesNumber, double * minKernel, double * maxKernel, int kernelSize) {
  // turn off the motor before calibration
  // digitalWrite(enablePin, LOW);

  // Serial.print("samplesCounter: "); Serial.print(samplesCounter); Serial.print(" ");
  // Serial.println("uT");

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
      double dummy = minKernel[j] - getPosition(table1[i + j - kernelSize / 2], table2[i + j - kernelSize / 2]);
      product += (dummy >= 0.0) ? dummy : -dummy;
    }
    if (product < threshold) {
      // Serial.print("i1: "); Serial.print(i); Serial.print(" ");
      // Serial.println("uT");
      value = product;
      tableBegin = i;
      break;
    }
  }

  // Fourth, find the index of the maximum
  for (int i = tableBegin + 1; i < samplesNumber - kernelSize / 2; i++) {
    product = 0.0;
    for (int j = 0; j < kernelSize; j++) {
      double dummy = maxKernel[j] - getPosition(table1[i + j - kernelSize / 2], table2[i + j - kernelSize / 2]);
      product += (dummy >= 0.0) ? dummy : -dummy;
    }
    if (product < threshold) {
      // Serial.print("i2: "); Serial.print(i); Serial.print(" ");
      // Serial.println("uT");
      value = product;
      tableEnd = i;
      break;
    }
  }

  int halfPeriod = tableEnd - tableBegin;
  int calibrated = 0;
  if (halfPeriod < samplesNumber / 2 && tableBegin < tableEnd)
    calibrated = 1;

  return calibrated;
}

rgb HSVtoRGB(hsv in) {
  float H = in.h;
  float S = in.s;
  float V = in.v;
  rgb out;
  out.r = 0;
  out.g = 0;
  out.b = 0;

  if (H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0) {
    // cout<<"The givem HSV values are not in valid range"<<endl;
    return out;
  }
  float s = S / 100;
  float v = V / 100;
  float C = s * v;
  float mod = fmod(H / 60.0, 2.0);
  // float absolute = abs(mod - 1);
  float absolute = mod - 1;
  absolute = (absolute >= 0.0) ? absolute : -absolute;
  float X = C * (1 - absolute);
  float m = v - C;
  float r, g, b;
  if (H >= 0 && H < 60) {
    r = C, g = X, b = 0;
  }
  else if (H >= 60 && H < 120) {
    r = X, g = C, b = 0;
  }
  else if (H >= 120 && H < 180) {
    r = 0, g = C, b = X;
  }
  else if (H >= 180 && H < 240) {
    r = 0, g = X, b = C;
  }
  else if (H >= 240 && H < 300) {
    r = X, g = 0, b = C;
  }
  else {
    r = C, g = 0, b = X;
  }
  out.r = (r + m) * 255;
  out.g = (g + m) * 255;
  out.b = (b + m) * 255;
  return out;
}
