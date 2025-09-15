/************************* customizable code *************************/

// #define USE_SERIAL
// #define USE_Z_RESET

// Button pin
#define PIN_IN_BUTTON_PULLUP A4
#define BUTTON_DEBOUNCE_MS 100
#define BUTTON_LONG_MS 1000

// Encoder pins
#define PIN_IN_QUAD_A 2
#define PIN_IN_QUAD_B 3
#if defined(USE_Z_RESET)
#define PIN_IN_QUAD_Z 4
#endif  // USE_Z_RESET

// Display pins
#define NUM_SEGMENTS 8
#define PIN_OUT_SEGMENT_A 5
#define PIN_OUT_SEGMENT_B 6
#define PIN_OUT_SEGMENT_C 7
#define PIN_OUT_SEGMENT_D 8
#define PIN_OUT_SEGMENT_E 9
#define PIN_OUT_SEGMENT_F 10
#define PIN_OUT_SEGMENT_G 11
#define PIN_OUT_SEGMENT_P 12
#define NUM_DIGITS 4
#define PIN_OUT_DIGIT_1 A0
#define PIN_OUT_DIGIT_2 A1
#define PIN_OUT_DIGIT_3 A2
#define PIN_OUT_DIGIT_4 A3

// Encoder rising edge per revolution (for each A/B signal)
#define ENCODER_PULSES_PER_REVOLUTION 1024

/************************* generic code *************************/

// Encoder count of 1/B signal *changes* in a single turn = max raw value before over/under flowing
#define ENCODER_RAW_VALUE_RANGE (ENCODER_PULSES_PER_REVOLUTION << 2)

// Quadrature encoder
// 2 possible signal values per quadrature signal
// 2 states for A/B (one state before and one state after) to decide direction
// = 4 bits of index for the lookup table
#define NUM_QUADRATURE_LOOKUPS (1 << 4)

#if defined(USE_Z_RESET)
// Do not process until homed
#define POSITION_UNDEFINED 0xFFFF
#endif  // USE_Z_RESET

// Angle decimals
#define NUM_DECIMAL_STEPS 10

// Power on self test duration
#define DISPLAY_MODE_TEST_DURATION_MS 1000

// Quadrature actions
#define QUADRATURE_NOOP 0
#define QUADRATURE_CLOCKWISE 1
#define QUADRATURE_COUNTER_CLOCKWISE (ENCODER_RAW_VALUE_RANGE - 1)
#define QUADRATURE_INVALID 2

// bit order:     PGFEDCBA
#define GLYPH_0 0b00111111
#define GLYPH_1 0b00000110
#define GLYPH_2 0b01011011
#define GLYPH_3 0b01001111
#define GLYPH_4 0b01100110
#define GLYPH_5 0b01101101
#define GLYPH_6 0b01111101
#define GLYPH_7 0b00000111
#define GLYPH_8 0b01111111
#define GLYPH_9 0b01101111
#define GLYPH_I 0b00010000
#define GLYPH_N 0b01010100
#define GLYPH_T 0b01111000
#define GLYPH_E 0b01111001
#define GLYPH_EMPTY 0b00000000
#define GLYPH_PERIOD 0b10000000

// Display rendering lookup
#define NUM_GLYPHS 16

typedef enum {
  ERROR_CODE_NONE = 0,
  ERROR_CODE_QUADRATURE,
  ERROR_CODE_MAX
} ERROR_CODE;

typedef enum {
  DISPLAY_MODE_TEST = 0,
#if defined(USE_Z_RESET)
  DISPLAY_MODE_INIT,
#endif  // USE_Z_RESET
  DISPLAY_MODE_RPM,
  DISPLAY_MODE_DEGREES,
  DISPLAY_MODE_RAW,
  DISPLAY_MODE_ERROR,
  DISPLAY_MODE_MAX
} DISPLAY_MODE;

typedef struct {
  uint8_t pin_number;
  uint8_t last_value;
  uint8_t current_value;
  uint32_t last_change_ms;
} DEBOUNCED_BUTTON;

const int16_t QUADRATURE_LOOKUPS[NUM_QUADRATURE_LOOKUPS] = {
  /* i old new        */
  /*    ba BA         */
  /* 0  00 00 same    */ QUADRATURE_NOOP,
  /* 1  00 01 cw 1    */ QUADRATURE_CLOCKWISE,
  /* 2  00 10 ccw 1   */ QUADRATURE_COUNTER_CLOCKWISE,
  /* 3  00 11 invalid */ QUADRATURE_INVALID,
  /* 4  01 00 ccw 4   */ QUADRATURE_COUNTER_CLOCKWISE,
  /* 5  01 01 same    */ QUADRATURE_NOOP,
  /* 6  01 10 invalid */ QUADRATURE_INVALID,
  /* 7  01 11 cw 2    */ QUADRATURE_CLOCKWISE, /* Z=1 cw */
  /* 8  10 00 cw 4    */ QUADRATURE_CLOCKWISE,
  /* 9  10 01 invalid */ QUADRATURE_INVALID,
  /* A  10 10 same    */ QUADRATURE_NOOP,
  /* B  10 11 ccw 2   */ QUADRATURE_COUNTER_CLOCKWISE,
  /* C  11 00 invalid */ QUADRATURE_INVALID,
  /* D  11 01 ccw 3   */ QUADRATURE_COUNTER_CLOCKWISE, /* Z=1 ccw */
  /* E  11 10 cw 3    */ QUADRATURE_CLOCKWISE,         /* Z=1 cw */
  /* F  11 11 same    */ QUADRATURE_NOOP
};

const uint8_t DIGIT_PINS[NUM_DIGITS] = {
  PIN_OUT_DIGIT_4,
  PIN_OUT_DIGIT_3,
  PIN_OUT_DIGIT_2,
  PIN_OUT_DIGIT_1,
};

const uint8_t SEGMENT_PINS[NUM_SEGMENTS] = {
  PIN_OUT_SEGMENT_A,
  PIN_OUT_SEGMENT_B,
  PIN_OUT_SEGMENT_C,
  PIN_OUT_SEGMENT_D,
  PIN_OUT_SEGMENT_E,
  PIN_OUT_SEGMENT_F,
  PIN_OUT_SEGMENT_G,
  PIN_OUT_SEGMENT_P,
};

const uint8_t GLYPHS[NUM_GLYPHS] = {
  /* i glyphe   PGFEDCBA */
  /* 0 */ GLYPH_0,
  /* 1 */ GLYPH_1,
  /* 2 */ GLYPH_2,
  /* 3 */ GLYPH_3,
  /* 4 */ GLYPH_4,
  /* 5 */ GLYPH_5,
  /* 6 */ GLYPH_6,
  /* 7 */ GLYPH_7,
  /* 8 */ GLYPH_8,
  /* 9 */ GLYPH_9,
  /* A */ GLYPH_I,
  /* B */ GLYPH_N,
  /* C */ GLYPH_T,
  /* D */ GLYPH_EMPTY,
  /* E */ GLYPH_E,
  /* F */ GLYPH_PERIOD,
};

// Prepared strings (see GLYPHS)
#if defined(USE_Z_RESET)
#define GLYPHS_INIT 0xABAC
#endif  // USE_Z_RESET

const uint16_t decimal_steps[NUM_DECIMAL_STEPS] = {
  /* 0 */ 0 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 1 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 2 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 3 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 4 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 5 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 6 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 7 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 8 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
  /* 1 */ 9 * (uint16_t)ENCODER_RAW_VALUE_RANGE / 10,
};

volatile ERROR_CODE error_code = ERROR_CODE_NONE;
volatile uint8_t quadrature_state;
#if defined(USE_Z_RESET)
volatile uint16_t position_value = POSITION_UNDEFINED;
volatile bool position_initialized = false;
#else
volatile uint16_t position_value = 0;
#endif  // USE_Z_RESET

#if defined(USE_Z_RESET)
void position_clockwise_reset() {
  position_value = 0;
  position_initialized = true;
}
#endif  // USE_Z_RESET

#if defined(USE_Z_RESET)
void position_counter_clockwise_reset() {
  position_value = ENCODER_RAW_VALUE_RANGE - 1;
  position_initialized = true;
}
#endif  // USE_Z_RESET

uint16_t degrees_decimal_from_raw_value(uint16_t value) {
  uint32_t numerator = 360 * 10 * ((uint32_t)value);
  uint32_t denominator = ENCODER_RAW_VALUE_RANGE;
  uint32_t result = numerator / denominator;
  return result;
}

void isr_quadrature_changed() {
  // lookup quadrature case situation
  quadrature_state = ((quadrature_state << 2) + (digitalRead(PIN_IN_QUAD_B) << 1) + digitalRead(PIN_IN_QUAD_A)) & 0xF;
  uint16_t position_change = QUADRATURE_LOOKUPS[quadrature_state];
  // check for errors
  if (position_change == QUADRATURE_INVALID) {
    error_code = ERROR_CODE_QUADRATURE;
    return;
  }
  // update position accordingly
  position_value += position_change;
  if (position_value >= ENCODER_RAW_VALUE_RANGE) {
    position_value -= ENCODER_RAW_VALUE_RANGE;
  }
#if defined(USE_Z_RESET)
  // handle specific reset using Z pin
  if (digitalRead(PIN_IN_QUAD_Z) == 1) {
    if (quadrature_state == 0x7 /* from CW waveform */) {
      position_clockwise_reset();
    } else if (quadrature_state == 0xD /* from CCW waveform */) {
      position_counter_clockwise_reset();
    }
  }
#endif  // USE_Z_RESET
}

void setup_display() {
  pinMode(PIN_OUT_SEGMENT_A, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_B, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_C, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_D, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_E, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_F, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_G, OUTPUT);
  pinMode(PIN_OUT_SEGMENT_P, OUTPUT);
  pinMode(PIN_OUT_DIGIT_1, OUTPUT);
  pinMode(PIN_OUT_DIGIT_2, OUTPUT);
  pinMode(PIN_OUT_DIGIT_3, OUTPUT);
  pinMode(PIN_OUT_DIGIT_4, OUTPUT);
  for (uint8_t index_segment = 0; index_segment < NUM_SEGMENTS; index_segment++) {
    pinMode(SEGMENT_PINS[index_segment], OUTPUT);
    digitalWrite(SEGMENT_PINS[index_segment], LOW);
  }
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    pinMode(DIGIT_PINS[index_digit], OUTPUT);
    digitalWrite(DIGIT_PINS[index_digit], HIGH);
  }
}

void setup_encoder() {
#if defined(USE_Z_RESET)
  pinMode(PIN_IN_QUAD_Z, INPUT);
#endif  // USE_Z_RESET
  pinMode(PIN_IN_QUAD_A, INPUT);
  pinMode(PIN_IN_QUAD_B, INPUT);
  quadrature_state = (((digitalRead(PIN_IN_QUAD_B) << 1) + digitalRead(PIN_IN_QUAD_A)) << 2) & 0xF;
  attachInterrupt(digitalPinToInterrupt(PIN_IN_QUAD_A), isr_quadrature_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN_QUAD_B), isr_quadrature_changed, CHANGE);
}

void display_glyph(uint8_t index_digit, uint8_t glyph_bits) {
  if (index_digit >= NUM_DIGITS) {
    return;
  }
  uint8_t digit_pin = DIGIT_PINS[index_digit];
  digitalWrite(digit_pin, LOW);
  for (uint8_t index_bit = 0; index_bit < 8; index_bit++) {
    uint8_t state = glyph_bits & 0x1;
    uint8_t segment_pin = SEGMENT_PINS[index_bit];
    digitalWrite(segment_pin, state);
    // delay(1);
    digitalWrite(segment_pin, LOW);
    glyph_bits >>= 1;
  }
  digitalWrite(digit_pin, HIGH);
}

void display_glyphs(uint16_t value, uint8_t overlay_periods_indices = 0) {
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    uint8_t index_glyph = value & 0xF;
    uint8_t glyph_bits = GLYPHS[index_glyph];
    if (overlay_periods_indices & 0x1) {
      glyph_bits |= GLYPHS[0xF /* '.' */];  // See GLYPH
    }
    display_glyph(index_digit, glyph_bits);
    value >>= 4;
    overlay_periods_indices >>= 1;
  }
}

uint16_t glyphs_from_value(uint16_t value) {
  uint16_t glyphs = 0;
  for (uint8_t index_digit = 0; index_digit < NUM_DIGITS; index_digit++) {
    uint8_t units = value % 10;
    glyphs |= units << (index_digit << 2);
    value = (value - units) / 10;
  }
  return glyphs;
}

uint16_t glyphs_degrees_decimal_from_value(uint16_t value) {
  uint16_t degrees_decimal = degrees_decimal_from_raw_value(value);
  uint16_t glyphs = glyphs_from_value(degrees_decimal);
  return glyphs;
}

uint16_t glyphs_from_error(uint16_t error) {
  uint16_t glyphs = glyphs_from_value(error);
  uint8_t glyphe_index_E = 0xE /* 'E' */;  // see GLYPHS
  glyphs |= (glyphe_index_E << 12);
  return glyphs;
}

void setup() {
#if defined(USE_SERIAL)
  Serial.begin(9600);
#endif  // USE_SERIAL
  setup_display();
  setup_encoder();
  pinMode(PIN_IN_BUTTON_PULLUP, INPUT);
}

void loop() {
  static DISPLAY_MODE display_mode = DISPLAY_MODE_TEST;
#if defined(USE_Z_RESET)
  static uint16_t current_position_value = POSITION_UNDEFINED;
#else
  static uint16_t current_position_value = 0;
#endif  // USE_Z_RESET
  static uint16_t position_value_relative_zero = 0;

  // initialize persistant button state
  static DEBOUNCED_BUTTON button_pullup = { PIN_IN_BUTTON_PULLUP, HIGH, HIGH, 0L };

  // use a non volatile variable for later processing
  current_position_value = position_value;

  // handle button
  uint32_t current_ms = millis();
  uint32_t duration_ms = current_ms - button_pullup.last_change_ms;
  button_pullup.current_value = digitalRead(button_pullup.pin_number);
  if (button_pullup.current_value != button_pullup.last_value) {
    // button has changed
    button_pullup.last_value = button_pullup.current_value;
    button_pullup.last_change_ms = current_ms;

    if (button_pullup.current_value == HIGH) {
      // button has been released
      if (duration_ms > BUTTON_DEBOUNCE_MS && duration_ms < BUTTON_LONG_MS) {
        // it was a short press
        switch (display_mode) {
          case DISPLAY_MODE_RPM:
            display_mode = DISPLAY_MODE_DEGREES;
            break;
          case DISPLAY_MODE_DEGREES:
            display_mode = DISPLAY_MODE_RAW;
            break;
          case DISPLAY_MODE_RAW:
            display_mode = DISPLAY_MODE_RPM;
            break;
          case DISPLAY_MODE_ERROR:
            // clear error and return to default mode
            error_code = ERROR_CODE_NONE;
            display_mode = DISPLAY_MODE_RPM;
          default:
            break;
        }
      }
    }
  } else {
    // button has not changed
    if (button_pullup.current_value == LOW) {
      // button is pressed
      if (duration_ms > BUTTON_LONG_MS) {
        // it is a long press
        if (display_mode == DISPLAY_MODE_DEGREES || display_mode == DISPLAY_MODE_RAW) {
          // use current position as base for relative moves
          position_value_relative_zero = current_position_value;
        }
      }
    }
  }

  // use stored value as new zero for relative work
  current_position_value += ENCODER_RAW_VALUE_RANGE - position_value_relative_zero;
  if (current_position_value >= ENCODER_RAW_VALUE_RANGE) {
    current_position_value -= ENCODER_RAW_VALUE_RANGE;
  }

  // prioritize errors
  if (error_code != ERROR_CODE_NONE) {
    display_mode = DISPLAY_MODE_ERROR;
  }

  switch (display_mode) {
    case DISPLAY_MODE_TEST:
      display_glyphs(glyphs_from_value(8888), 0b1111 /* overlay period on each digit */);
      if (millis() > DISPLAY_MODE_TEST_DURATION_MS) {
#if defined(USE_Z_RESET)
        display_mode = DISPLAY_MODE_INIT;
#else
        display_mode = DISPLAY_MODE_RAW; /* TODO: default to RPM */
#endif  // USE_Z_RESET
      }
      break;
#if defined(USE_Z_RESET)
    case DISPLAY_MODE_INIT:
      display_glyphs(GLYPHS_INIT);
      if (position_initialized) {
        display_mode = DISPLAY_MODE_RAW; /* TODO: default to RPM */
      }
      break;
#endif  // USE_Z_RESET
    case DISPLAY_MODE_RPM:
      display_glyphs(glyphs_from_value(1234)); /* TODO */
      break;
    case DISPLAY_MODE_DEGREES:
      display_glyphs(glyphs_degrees_decimal_from_value(current_position_value), 0b0010 /* overlay period on second-to-last digit */);
      break;
    case DISPLAY_MODE_RAW:
      display_glyphs(glyphs_from_value(current_position_value), 0b1111 /* overlay period on each digit */);
      break;
    case DISPLAY_MODE_ERROR:
      display_glyphs(glyphs_from_error(error_code));
      break;
    default:
      break;
  }
}
