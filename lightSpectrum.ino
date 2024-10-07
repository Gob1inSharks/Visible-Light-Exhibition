//
//
//
//

#include <FastLED.h>
#include <ezButton.h>
#include <cmath>
#include <array>

#define NUM_LEDS 72 //per strip
#define NUM_STRIPS 9

#define LPIN_1 2
#define LPIN_2 3
#define LPIN_3 4
#define LPIN_4 5
#define LPIN_5 6
#define LPIN_6 7
#define LPIN_7 8
#define LPIN_8 9
#define LPIN_9 10

#define DELAY_TIME 90

#define PI 3.1415926

#define PMAX 1023

CRGB lightBoard[NUM_STRIPS][NUM_LEDS];
bool of[NUM_STRIPS][NUM_LEDS];

float PLANCK_CONSTANT = 6.62 * pow(10,-34);

float wavelength = 430; //nanometers 400 to 700
float frequency = 1/wavelength; //Hz
float energy = frequency; //PLANCK_CONSTANT * Joules
int color; //rgb

void setup() {

  Serial.begin(9600);

  // todo clean this shit up

  FastLED.addLeds<NEOPIXEL, LPIN_1>(lightBoard[0], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_2>(lightBoard[1], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_3>(lightBoard[2], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_4>(lightBoard[3], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_5>(lightBoard[4], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_6>(lightBoard[5], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_7>(lightBoard[6], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_8>(lightBoard[7], NUM_LEDS);
  FastLED.addLeds<NEOPIXEL, LPIN_9>(lightBoard[8], NUM_LEDS);

}





/**
 * Convert XYZ to RGB in the sRGB color space
 *
 * The conversion matrix and color component transfer function is taken from http://www.color.org/srgb.pdf, which
 * follows the International Electrotechnical Commission standard IEC 61966-2-1 "Multimedia systems and equipment -
 * Colour measurement and management - Part 2-1: Colour management - Default RGB colour space - sRGB"
 *
 * @param xyz XYZ values in a std::array<double, 3> in the order of X, Y, Z. each value in the range of [0.0, 1.0]
 * @return RGB values in a std::array<double, 3>, in the order of R, G, B. each value in the range of [0.0, 1.0]
 */
std::array<double, 3> srgbXYZ2RGB(const std::array<double, 3>& xyz) {
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    double rl =  3.2406255 * x + -1.537208  * y + -0.4986286 * z;
    double gl = -0.9689307 * x +  1.8757561 * y +  0.0415175 * z;
    double bl =  0.0557101 * x + -0.2040211 * y +  1.0569959 * z;

    return {
        srgbXYZ2RGBPostprocess(rl),
        srgbXYZ2RGBPostprocess(gl),
        srgbXYZ2RGBPostprocess(bl)
    };
}

/**
 * helper function for srgbXYZ2RGB
 */
double srgbXYZ2RGBPostprocess(double c) {
    // clip if c is out of range
    c = c > 1 ? 1 : (c < 0 ? 0 : c);

    // apply the color component transfer function
    c = c <= 0.0031308 ? c * 12.92 : 1.055 * std::pow(c, 1. / 2.4) - 0.055;

    return c;
}

/**
 * A multi-lobe, piecewise Gaussian fit of CIE 1931 XYZ Color Matching Functions by Wyman el al. from Nvidia. The
 * code here is adopted from the Listing 1 of the paper authored by Wyman et al.
 *
 * Reference: Chris Wyman, Peter-Pike Sloan, and Peter Shirley, Simple Analytic Approximations to the CIE XYZ Color
 * Matching Functions, Journal of Computer Graphics Techniques (JCGT), vol. 2, no. 2, 1-11, 2013.
 *
 * @param wavelength wavelength in nm
 * @return XYZ in a std::array<double, 3> in the order of X, Y, Z. each value in the range of [0.0, 1.0]
 */
std::array<double, 3> cie1931WavelengthToXYZFit(double wavelength) {

    double wave = wavelength;

    double x;
    {
        double t1 = (wave - 442.0) * ((wave < 442.0) ? 0.0624 : 0.0374);
        double t2 = (wave - 599.8) * ((wave < 599.8) ? 0.0264 : 0.0323);
        double t3 = (wave - 501.1) * ((wave < 501.1) ? 0.0490 : 0.0382);

        x =   0.362 * std::exp(-0.5 * t1 * t1)
            + 1.056 * std::exp(-0.5 * t2 * t2)
            - 0.065 * std::exp(-0.5 * t3 * t3);
    }

    double y;
    {
        double t1 = (wave - 568.8) * ((wave < 568.8) ? 0.0213 : 0.0247);
        double t2 = (wave - 530.9) * ((wave < 530.9) ? 0.0613 : 0.0322);

        y =   0.821 * std::exp(-0.5 * t1 * t1)
            + 0.286 * std::exp(-0.5 * t2 * t2);
    }

    double z;
    {
        double t1 = (wave - 437.0) * ((wave < 437.0) ? 0.0845 : 0.0278);
        double t2 = (wave - 459.0) * ((wave < 459.0) ? 0.0385 : 0.0725);

        z =   1.217 * std::exp(-0.5 * t1 * t1)
            + 0.681 * std::exp(-0.5 * t2 * t2);
    }

    return { x, y, z };
}

/**
 * Convert a wavelength in the visible light spectrum to a RGB color value that is suitable to be displayed on a
 * monitor
 *
 * @param wavelength wavelength in nm
 * @return RGB color encoded in int. each color is represented with 8 bits and has a layout of
 * 00000000RRRRRRRRGGGGGGGGBBBBBBBB where MSB is at the leftmost
 */

int wavelengthToRGB(double wavelength) {
  std::array<double, 3> xyz = cie1931WavelengthToXYZFit(wavelength);
  std::array<double, 3> rgb = srgbXYZ2RGB(xyz);

  int c = 0;
  c |= (static_cast<int>(rgb[0] * 0xFF) & 0xFF) << 16;
  c |= (static_cast<int>(rgb[1] * 0xFF) & 0xFF) << 8;
  c |= (static_cast<int>(rgb[2] * 0xFF) & 0xFF) << 0;

  return c;
}







void updateBoard() {

  for (int strip = 0; strip < NUM_STRIPS; strip ++)
  {
    for (int led = 0; led < NUM_LEDS; led ++)
    {
      if (of[strip][led]) {lightBoard[strip][led] = color;
      }else{lightBoard[strip][led] = (0,0,0);}
      //delay(1);
    }
  }
}

void cleanBoard() 
{
  for (int strip = 0; strip < NUM_STRIPS; strip ++)
  {
    for (int led = 0; led < NUM_LEDS; led ++)
    {
      of[strip][led] = false;
      //delay(1);
    }
  }
}

void clear() 
{
  for (int strip = 0; strip < NUM_STRIPS; strip ++)
  {
    for (int led = 0; led < NUM_LEDS; led ++)
    {
      lightBoard[strip][led] = CRGB::Black;
      //delay(1);
    }
  }
  //FastLED.show(); for debug
}

void wave(int period, int transposition) //period minimum is 14
{
  for (int x = 0; x < NUM_LEDS; x++)
  {
    int y = (int)(4 * sin(((float)2/period) * x * PI + transposition)+4.5);
    //Serial.println(y); //for debug
    of[y][x] = true;
  }
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int getWavelength()
{
  int analogValue = analogRead(A0);

  float volt = floatMap(analogValue, 0, PMAX, 0, 470);

  return volt + 385 ;
}

int period;
int transposition = 0;

void loop() 
{
  // put your main code here, to run repeatedly:

  color = wavelengthToRGB(wavelength);
  delay(30);

  cleanBoard();

  int period = (int)((wavelength)/20);
  //Serial.println(period);
  wave(period,transposition);

  Serial.println(getWavelength());

  wavelength = getWavelength();

  //speed = (int)(period/5+0.5); //if you want speed to be dependent on period

  if (transposition < 1200){10*transposition++;
  }else{transposition = 0;}

  updateBoard();

  delay(30);
  
  FastLED.show();

  delay(DELAY_TIME);

}
