// #define USE_SERIAL

#define ENCODER_PULSE_PER_REV 1024

#define NUM_DELTAS 16

#define IN_QUAD_A 2
#define IN_QUAD_B 3
#define IN_QUAD_Z 4

#define NUM_SEGMENTS 8
#define OUT_SEGMENT_A 5
#define OUT_SEGMENT_B 6
#define OUT_SEGMENT_C 7
#define OUT_SEGMENT_D 8
#define OUT_SEGMENT_E 9
#define OUT_SEGMENT_F 10
#define OUT_SEGMENT_G 11
#define OUT_SEGMENT_P 12

#define NUM_DIGITS 4
#define OUT_DIGIT_1 A0
#define OUT_DIGIT_2 A1
#define OUT_DIGIT_3 A2
#define OUT_DIGIT_4 A3

volatile uint16_t position = 0;

#define POSITION_UNDEFINED 0xFFFF

uint16_t last_position = POSITION_UNDEFINED;

volatile uint8_t quadrature = 0;

volatile uint8_t last_invalid_quadrature = 0;

int8_t delta[NUM_DELTAS] = {
  /* i old new        */
  /*    ba BA         */
  /* 0  00 00 same    */ 0,
  /* 1  00 01 cw 1    */ +1,
  /* 2  00 10 ccw 1   */ -1,
  /* 3  00 11 invalid */ 2,
  /* 4  01 00 ccw 4   */ -1,
  /* 5  01 01 same    */ 0,
  /* 6  01 10 invalid */ 3,
  /* 7  01 11 cw 2    */ +1, /* Z=1 cw */
  /* 8  10 00 cw 4    */ +1,
  /* 9  10 01 invalid */ 4,
  /* A  10 10 same    */ 0,
  /* B  10 11 ccw 2   */ -1,
  /* C  11 00 invalid */ 5,
  /* D  11 01 ccw 3   */ -1, /* Z=1 ccw */
  /* E  11 10 cw 3    */ +1, /* Z=1 cw */
  /* F  11 11 same    */ 0
};

void quad_changed() {
  quadrature = ((quadrature << 2) + (digitalRead(IN_QUAD_B) << 1) + digitalRead(IN_QUAD_A)) & 0xF;

  int8_t change = delta[quadrature];
  if (change >= 2) {
    last_invalid_quadrature = quadrature;
    return;
  }
  position += change;

  // reset using Z pin
  if (digitalRead(IN_QUAD_Z) == 1) {
    if (quadrature == 0x7) {
      // CW Reset
      position = 0;
      last_position = (ENCODER_PULSE_PER_REV << 2) - 1;
    } else if (quadrature == 0xD) {
      // CCW reset
      // position = -1;
      position = (ENCODER_PULSE_PER_REV << 2) - 1;
      last_position = 0;
    }
  }
}

const float DEGREE_PER_CHANGE = 360.0f / ((float)(ENCODER_PULSE_PER_REV << 2));  // times 4 for number of *changes* per revolution

inline float angle_from_position(float pos) {
  return DEGREE_PER_CHANGE * pos;
}

uint8_t current_digit = 0;

uint8_t digit_pins[NUM_DIGITS] = {
  OUT_DIGIT_4,
  OUT_DIGIT_3,
  OUT_DIGIT_2,
  OUT_DIGIT_1,
};

uint8_t current_segment = 0;

uint8_t segment_pins[NUM_SEGMENTS] = {
  OUT_SEGMENT_A,
  OUT_SEGMENT_B,
  OUT_SEGMENT_C,
  OUT_SEGMENT_D,
  OUT_SEGMENT_E,
  OUT_SEGMENT_F,
  OUT_SEGMENT_G,
  OUT_SEGMENT_P,
};

#define NUM_GLYPHS 16

uint8_t glyphs[NUM_GLYPHS] = {
  /* i glyphe   PGFEDCBA */
  /* 0 '0' */ 0b00111111,
  /* 1 '1' */ 0b00000110,
  /* 2 '2' */ 0b01011011,
  /* 3 '3' */ 0b01001111,
  /* 4 '4' */ 0b01100110,
  /* 5 '5' */ 0b01101101,
  /* 6 '6' */ 0b01111101,
  /* 7 '7' */ 0b00000111,
  /* 8 '8' */ 0b01111111,
  /* 9 '9' */ 0b01101111,
  /* A 'H' */ 0b01110110,
  /* B 'E' */ 0b01111001,
  /* C 'L' */ 0b00111000,
  /* D 'P' */ 0b01110011,
  /* E '-' */ 0b01000000,
  /* F '.' */ 0b10000000,
};

void setup() {
#if defined(USE_SERIAL)
  Serial.begin(9600);
#endif  // USE_SERIAL
  pinMode(OUT_SEGMENT_A, OUTPUT);
  pinMode(OUT_SEGMENT_B, OUTPUT);
  pinMode(OUT_SEGMENT_C, OUTPUT);
  pinMode(OUT_SEGMENT_D, OUTPUT);
  pinMode(OUT_SEGMENT_E, OUTPUT);
  pinMode(OUT_SEGMENT_F, OUTPUT);
  pinMode(OUT_SEGMENT_G, OUTPUT);
  pinMode(OUT_SEGMENT_P, OUTPUT);
  pinMode(OUT_DIGIT_1, OUTPUT);
  pinMode(OUT_DIGIT_2, OUTPUT);
  pinMode(OUT_DIGIT_3, OUTPUT);
  pinMode(OUT_DIGIT_4, OUTPUT);
  for (uint8_t index_segment = 0; index_segment < NUM_SEGMENTS; index_segment++) {
    pinMode(segment_pins[index_segment], OUTPUT);
    digitalWrite(segment_pins[index_segment], LOW);
  }
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    pinMode(digit_pins[index_digit], OUTPUT);
    digitalWrite(digit_pins[index_digit], HIGH);
  }
#if defined(USE_SERIAL)
  Serial.print("Angular resolution: ");
  Serial.print(DEGREE_PER_CHANGE);
  Serial.println(" degrees per quadrature edge");
#endif  // USE_SERIAL
  pinMode(IN_QUAD_Z, INPUT);
  pinMode(IN_QUAD_A, INPUT);
  pinMode(IN_QUAD_B, INPUT);
  quadrature = (((digitalRead(IN_QUAD_B) << 1) + digitalRead(IN_QUAD_A)) << 2) & 0xF;
  attachInterrupt(digitalPinToInterrupt(IN_QUAD_A), quad_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IN_QUAD_B), quad_changed, CHANGE);
}

#define GLYPHS_DEFAULT 0xFFFF

#define GLYPHS_INIT 0xEEEE

#define GLYPHS_HELP 0xABCD

void display_glyph(uint8_t index_digit, uint8_t index_glyph) {
  if (index_digit >= NUM_DIGITS || index_glyph >= NUM_GLYPHS) {
    return;
  }
  uint8_t glyph = glyphs[index_glyph];
  uint8_t digit_pin = digit_pins[index_digit];
  digitalWrite(digit_pin, LOW);
  for (uint8_t index_bit = 0; index_bit < 8; index_bit++) {
    uint8_t state = glyph & 0x01;
    uint8_t segment_pin = segment_pins[index_bit];
    digitalWrite(segment_pin, state);
    // delay(1);
    digitalWrite(segment_pin, LOW);
    glyph >>= 1;
  }
  digitalWrite(digit_pin, HIGH);
}

void display_glyphs(uint16_t value) {
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    uint8_t index_glyph = value & 0x0F;
    display_glyph(index_digit, index_glyph);
    value >>= 4;
  }
}

uint16_t glyphs_from_uint16(uint16_t value) {
  uint16_t glyphs = 0;
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    uint8_t units = value % 10;
    glyphs |= units << (index_digit << 2);
    value = (value - units) / 10;
  }
  return glyphs;
}


void invalid_quadrature() {
#if defined(USE_SERIAL)
  Serial.print("Invalid quadrature: ");
  Serial.println(last_invalid_quadrature);
#endif  // USE_SERIAL
  display_glyphs(GLYPHS_HELP);
}

void position_undefined() {
  display_glyphs(GLYPHS_INIT);
}

#define DISPLAY_MODE_RAW 0

uint8_t display_mode = DISPLAY_MODE_RAW;

void display_value() {
#if defined(USE_SERIAL)
  if (position != last_position) {
    last_position = position;
    Serial.print("pos=");
    Serial.print(last_position);
    float angle = angle_from_position(last_position);
    Serial.print(" angle=");
    Serial.println(angle);
  }
#endif  // USE_SERIAL

  switch (display_mode) {
    case DISPLAY_MODE_RAW:
      display_glyphs(glyphs_from_uint16(position));
      break;
    default:
      display_glyphs(GLYPHS_DEFAULT);
      break;
  }
}

void loop() {
  if (last_invalid_quadrature != 0) {
    invalid_quadrature();
  } else if (last_position == POSITION_UNDEFINED) {
    position_undefined();
  } else {
    display_value();
  }
}
