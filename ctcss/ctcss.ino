#include <EEPROM.h>

#include "Wire.h"
#include "Adafruit_GFX.h"
#include "OakOLED.h"

// Note: i'm using https://github.com/netguy204/OakOLED and the oakled module which
// doesn't have a reset line.

OakOLED oled;

const byte k1_button_pin = 7; // Up
const byte k2_button_pin = 8; // Down
const byte k3_button_pin = 6; // Enable/disable
const byte k4_button_pin = 5; // PTT
const byte dds_out_pin = 9; // OCR1A output pin

#define NUM_CTCSSFREQ 50

#define CLK_FREQ_MHZ  16 // 16MHz clock, used for OCR math

// ctcss tones
const float ctcssFreq[] =   {  67.0,    69.3,    71.9,    74.4,    77.0,   79.7,    82.5,    85.4,    88.5,    91.5,
                         94.8,    97.4,   100.0,   103.5,   107.2,  110.9,   114.8,   118.8,   123.0,   127.3, 
                        131.8,   136.5,   141.3,   146.2,   151.4,  156.7,   159.8,   162.2,   165.5,   167.9,  
                        171.3,   173.8,   177.3,   179.9,   183.5,  186.2,   189.9,   192.8,   196.6,   199.5,
                        203.5,   206.5,   210.7,   218.1,   225.7,  229.1,   233.6,   241.8,   250.3,    254.1      
};

// These are the tables generated at boot to calculate the OCR programming
// values for each CTCSS tone.
byte ocrL[NUM_CTCSSFREQ]; byte ocrH[NUM_CTCSSFREQ];

volatile unsigned char ctcssFreqIdx = 0;
volatile byte isEnabled = 0;
volatile byte isPtt = 0;

// Yes, wasteful to use bytes here instead of a bitmap
// Yes, and should refactor out the code
volatile byte k1_button_state = 0;
volatile byte k2_button_state = 0;
volatile byte k3_button_state = 0;
volatile byte k4_button_state = 0;

// This is a quick counter to limit how often we're writing
// configuration values to EEPROM.
unsigned int eeprom_write_check = 0;

void eeprom_write_state()
{
  EEPROM.update(0, ctcssFreqIdx);
  EEPROM.update(1, isEnabled);
}

void
eeprom_read_state()
{
  // Read the enabled/ctcss freq index.
  // If they're not valid then initialise things to default.
  ctcssFreqIdx = EEPROM.read(0);
  isEnabled = EEPROM.read(1);

  if (ctcssFreqIdx >= NUM_CTCSSFREQ) {
    goto ini;
  }
  if (isEnabled > 1) {
    goto ini;
  }
  return;
ini:
  isEnabled = 0;
  ctcssFreqIdx = 0;
}

void
eeprom_start_check()
{
  // Kick-start the wait check...
  eeprom_write_check = 1;
}

void ctcsscalc()
{
  int N = 8; // Pre-scalar?
  int clk = CLK_FREQ_MHZ; // 16 MHz clock

  for (int n=0; n<NUM_CTCSSFREQ; n++) {
  
    int ocrLH = ( clk * 1000000 ) / (( 2 * N * ctcssFreq[n] ) - 1 ); 
    ocrH[n] = ocrLH >> 8;
    ocrL[n] = ocrLH & 0x00FF;
  }
}

void
update_display()
{
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(1);
  oled.setCursor(0, 0);
  oled.print(ctcssFreq[ctcssFreqIdx]);
  oled.print(" ");
  if (isEnabled && isPtt)
    oled.print("PTT");
  else if (isEnabled)
     oled.print("ON");
  else
    oled.print("OFF");

  oled.display();
}

void
update_oscillator()
{
 // configure Timer1 - clk/8, CTC mode
 if (isEnabled) { // XXX TODO: isPtt too!
   TCCR1A = 0x40; // Enable output of OCR1A to toggle; WGM10+11 are 0
   TCCR1B = 0x0A; // Bits 0..2 - clk/8 from prescaler; bit 3..4 - waveform generation WGM12+13
 } else {
   TCCR1A = 0;
   TCCR1B = 0;
 }

 TCCR1C = 0x00; // force output comparison
 OCR1AH = ocrH[ctcssFreqIdx];
 OCR1AL = ocrL[ctcssFreqIdx];
}

// button 1 - up button
void k1_button_isr()
{
   byte r;
   // XXX TODO: debounce
   r = digitalRead(k1_button_pin);

   // Invert; the board I'm using is active-low.
   r = !r;
   
   if (r == 1 && k1_button_state == 0) {
     // pressed
     ctcssFreqIdx = (ctcssFreqIdx + 1) % NUM_CTCSSFREQ;

     update_display();
     update_oscillator();
     eeprom_start_check();
   }
   else if (r == 0 && k1_button_state == 1) {
     // released
   }
   k1_button_state = r;
}

// button 2 - down button
void k2_button_isr()
{
    byte r;
   // XXX TODO: debounce
   r = digitalRead(k2_button_pin);

   // Invert; the board I'm using is active-low.
   r = !r;
   
   if (r == 1 && k2_button_state == 0) {
     // pressed
     if (ctcssFreqIdx == 0)
       ctcssFreqIdx = NUM_CTCSSFREQ-1;
     else
       ctcssFreqIdx--;

     update_display();
     update_oscillator();
     eeprom_start_check();
   }
   else if (r == 0 && k2_button_state == 1) {
     // released
   }
   k2_button_state = r;
}

// button 3 - enable/disable button
void k3_button_isr()
{
   byte r;
   // XXX TODO: debounce
   r = digitalRead(k3_button_pin);

   // Invert; the board I'm using is active-low.
   r = !r;
   
   if (r == 1 && k3_button_state == 0) {
     // pressed
     isEnabled = !isEnabled;
     update_display();
     update_oscillator();
     eeprom_start_check();

   }
   else if (r == 0 && k3_button_state == 1) {
     // released
   }
   k3_button_state = r;
}

// button 4 - PTT from radio
void
k4_button_isr()
{
   byte r;
   // XXX TODO: debounce
   r = digitalRead(k4_button_pin);

   // Invert; the board I'm using is active-low.
   r = !r;
   
   if (r == 1 && k4_button_state == 0) {
     // pressed
     isPtt = 1;
     update_display();
     update_oscillator();
   }
   else if (r == 0 && k4_button_state == 1) {
     // released
     isPtt = 0;
     update_display();
     update_oscillator();
   }
   k4_button_state = r;
}

void setup() {
  Serial.begin(115200);
  oled.begin();
  ctcsscalc();

  // The pins are active-ground, so use pull-ups here
  pinMode(k1_button_pin, INPUT_PULLUP);
  pinMode(k2_button_pin, INPUT_PULLUP);
  pinMode(k3_button_pin, INPUT_PULLUP);

  // PTT - active-high
  pinMode(k4_button_pin, INPUT);

  // DDS output, the OCR1A comparison
  pinMode(dds_out_pin, OUTPUT);
  
  // I'd use interrupts for this, but unfortunately 32u4 board I'm using only
  // allows interrupts on 0,1,2,3,7 - and I'm using SPI. So 1,2,3 aren't available.
#if DO_INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(k1_button_pin), k1_button_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(k2_button_pin), k2_button_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(k3_button_pin), k3_button_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(k4_button_pin), k4_button_isr, CHANGE);

#endif

  eeprom_read_state();

  update_display();
  update_oscillator();
}

void loop() {
#if DO_INTERRUPTS
  // XXX TODO: some halt instruction?
#else
  // Polling
  k1_button_isr();
  k2_button_isr();
  k3_button_isr();
  k4_button_isr();

  // Check to see if we have to do an EEPROM write.
  // This is only done if the display is updated from a button press.
  // The hack right now is to wait a million loops before writing
  // the current state to EEPROM.  That SHOULD be a second or two,
  // so scrolling through CTCSS values won't cause an EEPROM write.
  if (eeprom_write_check != 0) {
    eeprom_write_check++;
    if (eeprom_write_check >= 100 * 1000 * 1000) {
      eeprom_write_check = 0;
      eeprom_write_state();
    }
  }
#endif
}
