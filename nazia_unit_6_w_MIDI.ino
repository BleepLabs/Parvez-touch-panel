
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

#include <Wire.h>
#include "Adafruit_MPR121.h"

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

byte pb0, b0;
uint32_t bcal;

Adafruit_MPR121 cap = Adafruit_MPR121();

Adafruit_USBD_MIDI usb_midi;
// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

#include <FastLED.h>
#define DATA_PIN 9
#define CLOCK_PIN 8
#define NUM_LEDS 44
CRGB leds[NUM_LEDS];
float max_brightness = .9;
float response_curve = -0.55;
byte threshold_offset = 12;
byte lb_rx[16];
byte hb_rx[16];
byte noteon[16];
int pot_rx[8];
uint32_t cm, pm0, pm1, pm2, pm3, pm4;
byte cdc, cdt;
byte pb, bu;
int touch_raw;
byte cal_mode;
int find_low, find_high;
int thresh1;
float touch_map, touch_map_lerp, prev_touch_map_lerp;
byte blink1;
byte cal_once = 0;
byte b0_pin = 0;
byte bcled;

void setup() {
  pinMode(b0_pin, INPUT_PULLUP);

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BRG>(leds, NUM_LEDS);
  if (0) {
    for (int j; j < NUM_LEDS; j++) {
      leds[j].setHSV(0, 0, 255 * max_brightness);
    }
    FastLED.show();
    delay(500);
  }

  MIDI.begin(MIDI_CHANNEL_OMNI);

  Serial.begin(9600);

  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    delay(1000);
    Serial.println("MPR121 not found, check wiring?");
    delay(1000);
    //while (1);
  } else {
    Serial.println("MPR121 found");
  }

  //13 not working but auto cal is better
  //cap.writeRegister(0x5C, B00010000);
  //cap.writeRegister(0x5D, B00100100);
  // cap.writeRegister(0x5E, B00110000);

  cap.writeRegister(0x7B, 0x0B);
  cap.writeRegister(0x7D, 0x9C);
  cap.writeRegister(0x7E, 0x65);
  cap.writeRegister(0x7F, 0x8C);
  cal_mode = 1;
  delay(500);
  bcal = 1;
  find_low = 1000;
  find_high = 0;

  while (!TinyUSBDevice.mounted()) {
    delay(1);
  }
}



void loop() {
  cm = millis();
  pb0 = b0;
  b0 = digitalRead(b0_pin);

  touch_raw = smooth(0, 19, cap.filteredData(0));


  if (pb0 == 1 && b0 == 0) {
    bcal = cm;
    find_low = 1000;
    find_high = 0;
  }

  if (bcal > 0 && cm > 5000) {
    if (touch_raw < find_low) {
      find_low = touch_raw;
    }
    if (touch_raw > find_high) {
      find_high = touch_raw;
    }
    if (cm - bcal > 5000 && cm - bcal < 9000) {
      thresh1 = touch_raw - threshold_offset;
      bcled = 2;
    }
    if (cm - bcal >= 9000) {
      bcal = 0;
      bcled = 0;
    }
  }

  // using midi to set curve. not used
  int type, note, velocity, channel, d1, d2;
  if (MIDI.read() && 0) {
    byte type = MIDI.getType();
    if (type == 176) {
      d1 = MIDI.getData1();
      d2 = MIDI.getData2();
      if (d1 < 10) {
        lb_rx[d1] = d2;
      }
      if (d1 >= 10) {
        hb_rx[d1 - 10] = d2;
      }
    }
    if (type == 144) {
      Serial.print("                          no ");
      Serial.println(d1);
      Serial.println();

      noteon[d1] = 1;
    }
    for (byte j = 0; j < 8; j++) {
      pot_rx[j] = (hb_rx[j] << 7) + lb_rx[j];
    }
  }

  //touch_raw = smooth(0, 19, cap.filteredData(0));
  prev_touch_map_lerp = touch_map_lerp;
  if (touch_map_lerp < touch_map) {
    touch_map_lerp += 4;
  }
  if (touch_map_lerp > touch_map) {
    touch_map_lerp -= 4;
  }

  if (cm - pm1 > 1) {
    pm1 = cm;

    if (cal_mode == 1) {
    }

    //float response_curve = (pot_rx[5] / 2084.0) - 1.0;

    touch_map = fscale(touch_raw, response_curve, find_low, thresh1, 1000, 0);
    if (touch_map < 0) {
      touch_map = 0;
    }
    if (touch_map > 1000) {
      touch_map = 0;
    }

    int maplow = ((find_high - find_low) * .7) + find_low;
    //thresh1 = map(pot_rx[0], 0, 4095, maplow, find_high * 1.02);
    //thresh1 = find_high - 20;
  }

  if (cm - pm0 > 40) {
    pm0 = cm;
    byte m = 0;
    if (1) {
      Serial.print(0);
      Serial.print(" ");
      Serial.print(1000);
      Serial.print(" ");
      Serial.print(find_low);
      Serial.print(" ");
      Serial.print(find_high);
      Serial.print(" ");
      Serial.print(thresh1);
      Serial.print(" ");
      Serial.print(touch_raw);
      Serial.print(" ");
      Serial.print(touch_map);
      Serial.print(" ");
      Serial.print(touch_map_lerp);
      Serial.println(" ");
    }
    if (0) {
      Serial.print(pot_rx[0]);
      Serial.println(" ");
    }
    if (0) {
      for (byte j = 0; j < 8; j++) {
        Serial.print(pot_rx[j]);
        Serial.print(" ");
      }
      Serial.println(" ");
    }
  }

  if (cm - pm2 > 20) {
    pm2 = cm;

    for (int j; j < NUM_LEDS - 3; j++) {
      if (bcal == 0) {
        leds[j].setHSV(0, 0, (touch_map_lerp / 1000.0) * max_brightness * 255.0);
      }
      //leds[j].setHSV(0, 0, !blink1 * max_brightness * 255.0);
    }

    if (bcal > 0) {
      leds[0].setHSV(0, 1, 100);
    }

    if (bcled == 1) {
      for (int j; j < NUM_LEDS; j++) {
        //leds[j].setHSV(200, 0, (touch_map / 1000.0) * max_brightness * 255.0);
        leds[j].setHSV(0, 255, 20);
      }
    }

    if (bcled == 2) {
      for (int j; j < NUM_LEDS; j++) {
        //leds[j].setHSV(200, 0, (touch_map / 1000.0) * max_brightness * 255.0);
        leds[j].setHSV(100, 255, 20);
      }
    }

    leds[40].setHSV(100, 255, 10 * blink1);
  }

  if (cm - pm3 > 500) {
    pm3 = cm;
    blink1 = !blink1;
  }



  if (cm - pm4 > 10) {

    pm4 = cm;
    if (prev_touch_map_lerp != touch_map_lerp) {
      byte cc_out = map(touch_map_lerp, 0, 1000, 0, 127);
      MIDI.sendControlChange(30, cc_out, 1);
    }

    if (prev_touch_map_lerp < thresh && touch_map_lerp >= thresh) {
      MIDI.sendNoteOn(10, 100, 1);
    }
    if (prev_touch_map_lerp > thresh && touch_map_lerp <= thresh) {
      MIDI.sendNoteOff(10, 100, 1);
    }
  }

  FastLED.show();
}


//based on https://playground.arduino.cc/Main/DigitalSmooth/
// This function continuously samples an input and puts it in an array that is "samples" in length.
// This array has a new "raw_in" value added to it each time "smooth" is called and an old value is removed
// It throws out the top and bottom 15% of readings and averages the rest

#define maxarrays 8    //max number of different variables to smooth
#define maxsamples 99  //max number of points to sample and
//reduce these numbers to save RAM

unsigned int smoothArray[maxarrays][maxsamples];

// sel should be a unique number for each occurrence
// samples should be an odd number greater that 7. It's the length of the array. The larger the more smooth but less responsive
// raw_in is the input. positive numbers in and out only.

unsigned int smooth(byte sel, unsigned int samples, unsigned int raw_in) {
  int j, k, temp, top, bottom;
  long total;
  static int i[maxarrays];
  static int sorted[maxarrays][maxsamples];
  boolean done;

  i[sel] = (i[sel] + 1) % samples;    // increment counter and roll over if necessary. -  % (modulo operator) rolls over variable
  smoothArray[sel][i[sel]] = raw_in;  // input new data into the oldest slot

  for (j = 0; j < samples; j++) {  // transfer data array into anther array for sorting and averaging
    sorted[sel][j] = smoothArray[sel][j];
  }

  done = 0;            // flag to know when we're done sorting
  while (done != 1) {  // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (samples - 1); j++) {
      if (sorted[sel][j] > sorted[sel][j + 1]) {  // numbers are out of order - swap
        temp = sorted[sel][j + 1];
        sorted[sel][j + 1] = sorted[sel][j];
        sorted[sel][j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((samples * 15) / 100), 1);
  top = min((((samples * 85) / 100) + 1), (samples - 1));  // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for (j = bottom; j < top; j++) {
    total += sorted[sel][j];  // total remaining indices
    k++;
  }
  return total / k;  // divide by number of samples
}


float fscale(float inputValue, float curve, float originalMin, float originalMax, float newBegin, float newEnd) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  curve *= 10.0;

  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1);   // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve);  // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange;  // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  } else  // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}