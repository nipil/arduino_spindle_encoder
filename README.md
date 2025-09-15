# Arduino Spindle Encoder

This is the design and files for a lathe spindle encoder.

Available functionalities :

- use the lathe's spindle as a **dividing head** for various circular patterns
  - accurate angular positionning with a 0.1 degrees step
  - setting zero by long button press (in angular mode only)
  - optional : absolute encoder mode (requires Z pin on encoder and activating a #define)

- monitor **real spindle speed**
  - real time display in rotation par minute display (RPM)
  - useful in case of a VFD or DC motor or to monitor motor load during work
  - works in forward and reverse

- optional : raw encoder value display

- optional : interrupt service routine timing
  - if enabled, used LED BUILTIN pin by default, configurable
  - currently, on an Arduino UNO, ISR duration is 20 usec (50000x per second)

# Breadboard prototype

[]!(breadboard.svg)

# Limitations

- display is limited to 4 digits
  - max 9999 RPM then overflow
  - do not use encoders with more than 2000 pulses per revolution
  - 1-digit precision (and display) for angular degrees of rotation

- encoder output bandwitch
  - maximum output frequency is 100kHz for the reference below
  - you choice of PPR must satisfy `RPM * 4 * PPR / 60 < 100_000`
  - because 1 rotation = 4 * number of pulse per revolution = events
  - to chose your encoder PPR, check `desired_RPM * PPR < 1 500 000`

# Components

- 1x Omron E6B2-CWZ6C rotary encoder **with open-collector output**
  - https://www.ia.omron.com/product/item/2453/
  - https://www.ia.omron.com/data_pdf/cat/e6b2-c_ds_e_6_3_csm491.pdf
- 1x Arduino UNO
  - or any other 5V MCU with enough pins (see requirements below)
- Display
  - 1x 5641AS **Common-Cathode** 4-digit 7-segment display
    - https://www.xlitx.com/datasheet/5641AS.pdf
  - 4x 330 ohm resistors for 7-segment display
- User interface
  - 1x momentary normally open push button
  - 1x 10k ohm resistor for push button pullup
- Data input
  - 2x to 3x **3.3k** ohm pullup resistors for the rotary encoder
  - I chose a **strong pullup** because encoder might be in a noisy environment
- Connections
  - 6x to 10x scren terminals
    - 2x for button
    - 2x power supply / power switch
    - 4x to 6x for rotary encoder (VCC/GND/A/B + shield + Z)

# Requirements

- Electrical characteristics
  - 5V power supply (adapt the display resistors in case of a 3.3V MCU)
  - Consumes 26 milli-amperes when the Arduino UNO rev3 is powered from Vin @5V

- Pins
  - 12 digital output pins to directly drive the display (8 segments + 4 digits)
  - 1 digital input for the push button
  - and for the encoder :
    - either 3 digital input pins (for the referenced encoder)
    - or 2 if your encoder lacks the "Z/home" pin

- Sketch information :
  - Does not need internal pullups
  - Sketch uses 2700 bytes (8% of the UNO's program space)
  - Global variables use 85 bytes (4% of UNO's memory space)
