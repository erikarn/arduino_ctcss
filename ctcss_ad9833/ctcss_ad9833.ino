
#include <EEPROM.h>
#include <AD9833.h>        

#include "Wire.h"
#include "Adafruit_GFX.h"
#include "OakOLED.h"

// Note: i'm using https://github.com/netguy204/OakOLED and the oakled module which
// doesn't have a reset line.

// The Oakled is wired up to the SPI pins.
OakOLED oled;

// These lines are connected to buttons that short-to-ground.
const byte k1_button_pin = 7; // Up
const byte k2_button_pin = 8; // Down
const byte k3_button_pin = 6; // Enable/disable

// This is for the eventual PTT input line to gate transmitting the PL tone on/off.
// If I get around to it, it should also be used to generate a 180 degree phase shifted
// tone for a little interval after PTT is released to "quieten" the repeater.
// (Don't ask; it's an old school hardware repeater PL tone decode thingy that somehow
// persists even today..)
const byte k4_button_pin = 5; // PTT

const byte FNC_PIN = 4; // AD9833 enable line
// Note: The AD9833 is wired up to the MOSI/CLK lines.

byte display_active = 0;

#define NUM_CTCSSFREQ 50

// ctcss tones
const float ctcssFreq[] =   {  67.0,    69.3,    71.9,    74.4,    77.0,   79.7,    82.5,    85.4,    88.5,    91.5,
                         94.8,    97.4,   100.0,   103.5,   107.2,  110.9,   114.8,   118.8,   123.0,   127.3, 
                        131.8,   136.5,   141.3,   146.2,   151.4,  156.7,   159.8,   162.2,   165.5,   167.9,  
                        171.3,   173.8,   177.3,   179.9,   183.5,  186.2,   189.9,   192.8,   196.6,   199.5,
                        203.5,   206.5,   210.7,   218.1,   225.7,  229.1,   233.6,   241.8,   250.3,    254.1      
};

volatile unsigned char ctcssFreqIdx = 0;
volatile byte isEnabled = 0; // Whether output is enabled or not
volatile byte isPtt = 0; // Whether PTT is asserted or not
volatile byte doPtt = 0; // Whether we should be using PTT to flip on/off CTCSS or not

// Yes, wasteful to use bytes here instead of a bitmap
// Yes, and should refactor out the code
volatile byte k1_button_state = 0;
volatile byte k2_button_state = 0;
volatile byte k3_button_state = 0;
volatile byte k4_button_state = 0;

// This is a quick counter to limit how often we're writing
// configuration values to EEPROM.
unsigned int eeprom_write_check = 0;

AD9833 gen(FNC_PIN);

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

void
blank_display()
{
  oled.clearDisplay();
  oled.display();
  display_active = 0;
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
  if (eeprom_write_check != 0) {
    oled.print("*");
  }

  oled.display();
  display_active = 1;
}

void
update_oscillator()
{
  if (isEnabled) {
    gen.ApplySignal(SINE_WAVE, REG1, ctcssFreq[ctcssFreqIdx]);
    gen.SetOutputSource(REG1);
    gen.EnableOutput(true);
  } else {
    gen.EnableOutput(false);
  }
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
     if (display_active == 1) {
       ctcssFreqIdx = (ctcssFreqIdx + 1) % NUM_CTCSSFREQ;
     }
     eeprom_start_check();
     update_display();
     update_oscillator();
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
     if (display_active == 1) {
       if (ctcssFreqIdx == 0)
         ctcssFreqIdx = NUM_CTCSSFREQ-1;
       else
         ctcssFreqIdx--;
     }
     eeprom_start_check();
     update_display();
     update_oscillator();
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
     if (display_active == 1) {
       isEnabled = !isEnabled;
     }
     eeprom_start_check();
     update_display();
     update_oscillator();
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

   // Note: PTT here is active-high, not inverted.
   // add electronics or hack this routine if you need it inverted.
   if (r == 1 && k4_button_state == 0) {
     // pressed
     isPtt = 1;
     if (display_active == 1) {
       update_display();
     }
     update_oscillator();
   }
   else if (r == 0 && k4_button_state == 1) {
     // released
     isPtt = 0;
     if (display_active == 1) {
       update_display();
     }
     update_oscillator();
   }
   k4_button_state = r;
}


void setup() {
  Serial.begin(115200);
  oled.begin();
  gen.Begin();
  
  // The pins are active-ground, so use pull-ups here
  pinMode(k1_button_pin, INPUT_PULLUP);
  pinMode(k2_button_pin, INPUT_PULLUP);
  pinMode(k3_button_pin, INPUT_PULLUP);

  // PTT - active-high
  pinMode(k4_button_pin, INPUT);
  
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
  // XXX TODO: if this gets implemented, some kind of timer routine will be
  // needed to drive the eeprom write logic.
#else
  // Polling
  k1_button_isr();
  k2_button_isr();
  k3_button_isr();
  k4_button_isr();

#if 1
  // Check to see if we have to do an EEPROM write.
  // This is only done if the display is updated from a button press.
  // The hack right now is to wait a bunch of loops before writing
  // the current state to EEPROM.  That SHOULD be a second or two,
  // so scrolling through CTCSS values won't cause an EEPROM write.
  if (eeprom_write_check != 0) {
    eeprom_write_check++;
    if (eeprom_write_check >= (50000)) {
      eeprom_write_check = 0;
      eeprom_write_state();
      //update_display();
      blank_display();
    }
  }
#endif
#endif
}
