// This file includes tools for converting between color spaces among other utilities

#include "math.h"

typedef struct {
  double safetyAngle;
  double angle;
  double dot;
  double dotDot;

  double acc;
  double euler;
  double gyro;
  
  double _acc;
  double _euler;
  double _gyro;

  double accScale;
  double eulerScaleNegative;
  double eulerScalePositive;
  double gyroScale;
  double accOffset;
  double eulerOffset;
  double gyroOffset;

  int axis;
  int positivePin;
  int negativePin;
  bool updated;
} Orientation;

typedef struct {
  float r;       // a fraction between 0 and 1
  float g;       // a fraction between 0 and 1
  float b;       // a fraction between 0 and 1
} Rgb;

typedef struct {
  float h;       // angle in degrees
  float s;       // a fraction between 0 and 1
  float v;       // a fraction between 0 and 1
} Hsv;

//static Orientation getOrientation(Orientation in, MPU9250 mpu, SimpleKalmanFilter phiFilter, SimpleKalmanFilter thetaFilter, double ratio);
static Rgb HSVtoRGB(Hsv in);

Rgb HSVtoRGB(Hsv input) {
  float H = input.h;
  float S = input.s;
  float V = input.v;
  Rgb output;
  output.r = 0;
  output.g = 0;
  output.b = 0;

  if (H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0) {
    // cout<<"The givem HSV values are not in valid range"<<endl;
    return output;
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
  output.r = (r + m) * 255;
  output.g = (g + m) * 255;
  output.b = (b + m) * 255;
  return output;
}
