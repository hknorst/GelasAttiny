#include <Arduino.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
//#define CLK 0  // PB0
//#define DIO 4  // PB4
#define CLK 1  // PB2
#define DIO 2  // PB1

//#define NTC 1   // PB2
#define NTC 2   // A2 - PB4
//#define RELE 1  // PB1
#define RELE 0  // PB0

#define SAMPLES 10  // number of samples to average
#define REFRESH 3   // update interval (seconds)

#define TEMP_UP 6    // turn on temperature
#define TEMP_DOWN 4  // turn off temperature

bool run = false;  // enable control after full samples buffer
bool blink = false;
bool output = false;

TM1637Display display(CLK, DIO);

int Ro = 100, B = 4036;  // Nominal resistance 100K, Beta constant
int Rseries = 100;       // Series resistor 100K
float To = 298.15;       // Nominal Temperature 25°C in Kelvin
float Tc = 0;            // Thermocouple reading var

int samples[SAMPLES];  // samples array
int index = 0;         // samples array index

const unsigned long eventTemp = REFRESH * 1000;  // Temperature readings delay (seconds)
const unsigned long eventDisp = 500;             // Display refresh rate
unsigned long previousTemp = 0;
unsigned long previousDisp = 0;

const uint8_t SEG_ERR[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,  // E
  SEG_G | SEG_E,                          // r
  SEG_G | SEG_E,                          // r
  SEG_G | SEG_C | SEG_D | SEG_E           // o
};

const uint8_t SEG_HIFEN[] = {
  SEG_G,  // -
  SEG_G,  // -
  SEG_G,  // -
  SEG_G   // -
};

void setup() {
  pinMode(RELE, OUTPUT);    // output relay
  digitalWrite(RELE, LOW);  // output off

  display.clear();
  display.setBrightness(0x0f);  // full brightness

  for (int i = 0; i < SAMPLES; i++) {  // clear array
    samples[i] = 0;
  }
}

void loop() {
  /* Updates frequently */
  unsigned long currentTime = millis();

  if ((currentTime - previousTemp >= eventTemp)) {
    previousTemp = currentTime;

    samples[index] = analogRead(NTC);
    index++;
    if (index == SAMPLES) {
      index = 0;
      run = true;  // fill the samples array then enable temp control
    }

    int average = 0;  // calculate average measure
    for (int i = 0; i < SAMPLES; i++) {
      average += samples[i];
    }
    average = average / SAMPLES;

    float Vi = average * (5.0 / 1023.0);
    // Convert voltage measured to resistance value
    // All Resistance are in kilo ohms.
    float R = (Vi * Rseries) / (5 - Vi);
    /*Use R value in steinhart and hart equation
    Calculate temperature value in kelvin*/
    float T = 1 / ((1 / To) + ((log(R / Ro)) / B));
    Tc = T - 273.15;  // Converting kelvin to celsius
  }

  if ((currentTime - previousDisp >= eventDisp)) {
    previousDisp = currentTime;

    if ((Tc > 50 || Tc < 0) && run) {  // if sensor out of range and run enable, show "Erro" and disable output
      display.setSegments(SEG_ERR);
      digitalWrite(RELE, LOW);  // turn the compressor off
      output = false;

    } else if (!run) {  // if is populating the reading buffer, shows "----"
      display.setSegments(SEG_HIFEN);
      digitalWrite(RELE, LOW);  // turn the compressor off
      output = false;

    } else {  // or shows current measure and control output

      if ((Tc > TEMP_UP) && run) {
        digitalWrite(RELE, HIGH);  // turn the compressor on
        output = true;
      }

      if ((Tc < TEMP_DOWN) && run) {
        digitalWrite(RELE, LOW);  // turn the compressor off
        output = false;
      }


      if (blink && output) {  // if output is enabled, blink the dot
        display.showNumberDecEx((int)(Tc * 100), 64, false, 4, 0);
        blink = !blink;  // toggle the dot control flag
      } else if (!blink && output) {
        display.showNumberDec((int)(Tc * 100), false, 4, 0);
        blink = !blink;  // toggle the dot control flag
      } else {           // if the output is disabled, keep the dot on
        display.showNumberDecEx((int)(Tc * 100), 64, false, 4, 0);
      }
    }
  }
}
