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

double getPosition(double a, double b) {
  return sqrt(pow(a, 2) + pow(b, 2));
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
