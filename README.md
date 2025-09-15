# Arduino Spindle Encoder

This is the design and files for a lathe spindle encoder, which can be used to :

- use the lathe's spindle as a **dividing head** for various circular patterns
- monitor **real spindle speed** (in case of a VFD or DC motor)

# Components

- 1x Omron E6B2-CWZ6C rotary encoder **with open-collector output**
  - https://www.ia.omron.com/product/item/2453/
  - https://www.ia.omron.com/data_pdf/cat/e6b2-c_ds_e_6_3_csm491.pdf
- 1x Arduino UNO
  - or any other 5V MCU with enough pins (see requirements below)
- Display
  - 1x [5641AS **Common-Cathode** 4-digit 7-segment display]()
  - 4x 330 ohm resistors for 7-segment display
- User interface
  - 1x momentary normally open push button
  - 1x 10k ohm resistor for push button pullup

# Requirements

- Electrical characteristics
  - 5V power supply (adapt the display resistors in case of a 3.3V MCU)
  - Consumes ??? mA

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
