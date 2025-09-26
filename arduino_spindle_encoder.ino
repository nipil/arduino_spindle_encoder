/************************* optional stuff *************************/

// Using an encoder with a Z pin allows for a "fixed" homing
// This allows your position to be found again after a power loss
#define USE_Z_RESET

// Use a digital pin to allow scoping interrupt duration
#define USE_ISR_WAVEFORM

// Serial compile-time flag (use macros below)
// #define USE_SERIAL_PRINT

// Allow raw counter display
#define USE_RAW_DISPLAY

/************************** Configuration ******************************/

// For pins, you can use "native Arduino" pin numbers, or PIN_Pxy port defines from arduino.h,
// The latter allows clearer mapping, when using bare chipsets, like on the perfboards

// Display configuration (Arduino PINS)
#define CONFIG_PIN_OUT_SEGMENT_A 9
#define CONFIG_PIN_OUT_SEGMENT_B 13
#define CONFIG_PIN_OUT_SEGMENT_C 5
#define CONFIG_PIN_OUT_SEGMENT_D 7
#define CONFIG_PIN_OUT_SEGMENT_E 8
#define CONFIG_PIN_OUT_SEGMENT_F 10
#define CONFIG_PIN_OUT_SEGMENT_G A0
#define CONFIG_PIN_OUT_SEGMENT_DP 6
#define CONFIG_PIN_OUT_DIGIT_1 A2
#define CONFIG_PIN_OUT_DIGIT_2 A3
#define CONFIG_PIN_OUT_DIGIT_3 A4
#define CONFIG_PIN_OUT_DIGIT_4 A5
#define CONFIG_SEGMENT_ON_TIME_MICROS 10

// Encoder configuration (Arduino PINS)
#define CONFIG_ENCODER_PULSES_PER_REVOLUTION 1024  // number of rising edges per revolution !! PER signal !!
#define CONFIG_PIN_IN_QUAD_A 2
#define CONFIG_PIN_IN_QUAD_B 3
#if defined(USE_Z_RESET)
#define CONFIG_PIN_IN_QUAD_Z 4
#define CONFIG_PIN_IN_QUAD_Z_ACTIVE_STATE HIGH
#endif  // USE_Z_RESET

// UI configuration (Arduino PINS)
#define CONFIG_PIN_IN_BUTTON 12
#define CONFIG_PIN_IN_BUTTON_ACTIVE_STATE LOW
#define CONFIG_BUTTON_DEBOUNCE_MILLIS 100
#define CONFIG_BUTTON_LONG_PRESS_MILLIS 1000
#define CONFIG_UI_INTERVAL_MILLIS 500
#define CONFIG_DISPLAY_MODE_TEST_DURATION_MILLIS 1000

// Use led pin to scope for interrupt duration (Arduino PINS)
#if defined(USE_ISR_WAVEFORM)
#define CONFIG_PIN_OUT_ISR_WAVEFORM 11
#endif  // USE_ISR_WAVEFORM

/************************* Serial macros *************************/

#if defined(USE_SERIAL_PRINT)
#define SERIAL_BEGIN(X) \
  do { Serial.begin(X); } while (0)
#define SERIAL_PRINT(X) \
  do { Serial.print(X); } while (0)
#define SERIAL_PRINTLN(X) \
  do { Serial.println(X); } while (0)
#define SERIAL_PRINT_2(X, Y) \
  do { Serial.print(X, Y); } while (0)
#define SERIAL_PRINTLN_2(X, Y) \
  do { Serial.println(X, Y); } while (0)
#else
#define SERIAL_BEGIN(X)
#define SERIAL_PRINT(X)
#define SERIAL_PRINTLN(X)
#define SERIAL_PRINT_2(X, Y)
#define SERIAL_PRINTLN_2(X, Y)
#endif

/************************** Library ******************************/

class Error {
public:
  typedef enum {
    ERROR_CODE_NONE = 0,
    ERROR_CODE_QUADRATURE,
  } Code;

  Error(const Code code, const uint32_t value = 0)
    : code(code), value(value) {}

  const Code code;
  const uint32_t value;
};

class Glyph {
public:
  typedef enum {
    // bit order is 0bPGFEDCBA
    DIGIT_ZERO = 0b00111111,
    DIGIT_ONE = 0b00000110,
    DIGIT_TWO = 0b01011011,
    DIGIT_THREE = 0b01001111,
    DIGIT_FOUR = 0b01100110,
    DIGIT_FIVE = 0b01101101,
    DIGIT_SIX = 0b01111101,
    DIGIT_SEVEN = 0b00000111,
    DIGIT_EIGHT = 0b01111111,
    DIGIT_NINE = 0b01101111,
    LETTER_I_LOWERCASE = 0b00010000,
    LETTER_N_LOWERCASE = 0b01010100,
    LETTER_T_LOWERCASE = 0b01111000,
    LETTER_E_UPPERCASE = 0b01111001,
    SYMBOL_SPACE = 0b00000000,
    SYMBOL_PERIOD = 0b10000000,
  } Bits;

  Glyph(const uint8_t bits)
    : bits(bits) {}

  void or_with(const Glyph& other) {
    bits |= other.bits;
  }

  uint8_t bits;
};

class GlyphLibrary {
public:
  typedef enum {
    DIGIT_ZERO = 0,
    DIGIT_ONE,
    DIGIT_TWO,
    DIGIT_THREE,
    DIGIT_FOUR,
    DIGIT_FIVE,
    DIGIT_SIX,
    DIGIT_SEVEN,
    DIGIT_EIGHT,
    DIGIT_NINE,
    LETTER_I_LOWERCASE,
    LETTER_N_LOWERCASE,
    LETTER_T_LOWERCASE,
    LETTER_E_UPPERCASE,
    SYMBOL_SPACE,
    SYMBOL_PERIOD,
    INDEX_MAX
  } GlyphIndex;

  static const Glyph GLYPHS[];

  static const Glyph& get_glyph(const uint8_t index) {
    if (index >= INDEX_MAX) {
      return GLYPHS[SYMBOL_SPACE];
    }
    return GLYPHS[index];
  }
};

const Glyph GlyphLibrary::GLYPHS[INDEX_MAX] = {
  Glyph(Glyph::Bits::DIGIT_ZERO),
  Glyph(Glyph::Bits::DIGIT_ONE),
  Glyph(Glyph::Bits::DIGIT_TWO),
  Glyph(Glyph::Bits::DIGIT_THREE),
  Glyph(Glyph::Bits::DIGIT_FOUR),
  Glyph(Glyph::Bits::DIGIT_FIVE),
  Glyph(Glyph::Bits::DIGIT_SIX),
  Glyph(Glyph::Bits::DIGIT_SEVEN),
  Glyph(Glyph::Bits::DIGIT_EIGHT),
  Glyph(Glyph::Bits::DIGIT_NINE),
  Glyph(Glyph::Bits::LETTER_I_LOWERCASE),
  Glyph(Glyph::Bits::LETTER_N_LOWERCASE),
  Glyph(Glyph::Bits::LETTER_T_LOWERCASE),
  Glyph(Glyph::Bits::LETTER_E_UPPERCASE),
  Glyph(Glyph::Bits::SYMBOL_SPACE),
  Glyph(Glyph::Bits::SYMBOL_PERIOD),
};

typedef enum {
  COMMON_CATHODE = LOW,
  COMMON_ANODE = HIGH, /* TODO: untested */
} DisplayType;

class SevenSegmentPins {
public:
  typedef enum {
    SEGMENT_A = 0,
    SEGMENT_B,
    SEGMENT_C,
    SEGMENT_D,
    SEGMENT_E,
    SEGMENT_F,
    SEGMENT_G,
    SEGMENT_DP,
    SEGMENT_MAX
  } SegmentPinIndex;

  SevenSegmentPins(
    const DisplayType display_type,
    const uint8_t pin_a,
    const uint8_t pin_b,
    const uint8_t pin_c,
    const uint8_t pin_d,
    const uint8_t pin_e,
    const uint8_t pin_f,
    const uint8_t pin_g,
    const uint8_t pin_dp,
    const uint16_t time_on_micros)
    : display_type(display_type),
      time_on_micros(time_on_micros),
      pin_segments{
        pin_a,
        pin_b,
        pin_c,
        pin_d,
        pin_e,
        pin_f,
        pin_g,
        pin_dp
      } {}

  void setup() const {
    for (uint8_t i = 0; i < SEGMENT_MAX; i++) {
      const uint8_t segment_pin = get_segment_pin((SegmentPinIndex)i);
      pinMode(segment_pin, OUTPUT);
      digitalWrite(segment_pin, display_type);
    }
  }

  void render_glyph(const Glyph& glyph) const {
    uint8_t glyph_bits = glyph.bits;
    for (uint8_t i = 0; i < SEGMENT_MAX; i++) {
      const bool segment_is_on = glyph_bits & 1;
      render_segment((SegmentPinIndex)i, segment_is_on);
      glyph_bits >>= 1;
    }
  }

private:
  uint8_t get_segment_pin(const SegmentPinIndex index) const {
    if (index >= SEGMENT_MAX) {
      return pin_segments[SEGMENT_DP];
    }
    return pin_segments[index];
  }

  void render_segment(const SegmentPinIndex segment_index, const bool segment_is_on) const {
    const uint8_t segment_pin = get_segment_pin(segment_index);
    // to turn on, segment is the inverse of the digit --> !display_type
    // turn on only if state is true = invert (xor) the "on value", if state is false
    digitalWrite(segment_pin, !display_type ^ !segment_is_on);  // enable segment pin (if needed)
    delayMicroseconds(time_on_micros);
    digitalWrite(segment_pin, display_type);  // disable segment pin (always)
  }

public:
  const DisplayType display_type;

private:
  const uint16_t time_on_micros;
  const uint8_t pin_segments[SEGMENT_MAX];
};

class SegmentDisplay {
public:
  SegmentDisplay(const SevenSegmentPins& segment_pins, const uint8_t pin_digit)
    : segment_pins(segment_pins), pin_digit(pin_digit) {}

  void setup() const {
    pinMode(pin_digit, OUTPUT);
    digitalWrite(pin_digit, !segment_pins.display_type);
  }

  void render_glyph(const Glyph& glyph) const {
    digitalWrite(pin_digit, segment_pins.display_type);  // enable digit pin
    segment_pins.render_glyph(glyph);
    digitalWrite(pin_digit, !segment_pins.display_type);  // disable digit pin
  }

private:
  const SevenSegmentPins& segment_pins;
  const uint8_t pin_digit;
};

typedef enum {
  DISPLAY_1 = 0,
  DISPLAY_2,
  DISPLAY_3,
  DISPLAY_4,
  DISPLAY_MAX
} DisplayIndex;

class Glyphs {
public:
  static const Glyphs INIT;
  static const Glyphs TEST;
  static const Glyphs EMPTY;

  Glyphs(const Glyph& glyph1, const Glyph& glyph2, const Glyph& glyph3, const Glyph& glyph4)
    : glyphs{ glyph1, glyph2, glyph3, glyph4 } {}


  Glyph& get_glyph(const DisplayIndex index) {
    if (index >= DISPLAY_MAX) {
      return glyphs[0];  // protection
    }
    return glyphs[index];
  }

  const Glyph& get_glyph_ref(const DisplayIndex index) const {
    // return get_glyph(index);  // does not compile due to const discard
    if (index >= DISPLAY_MAX) {
      return glyphs[0];  // protection
    }
    return glyphs[index];
  }

  static Glyphs build_uint16(uint16_t value, uint8_t min_digit) {
    GlyphLibrary::GlyphIndex digits[DISPLAY_MAX] = {
      GlyphLibrary::SYMBOL_SPACE,
      GlyphLibrary::SYMBOL_SPACE,
      GlyphLibrary::SYMBOL_SPACE,
      GlyphLibrary::SYMBOL_SPACE
    };
    for (uint8_t i = 0; i < DISPLAY_MAX; i++) {
      if (value == 0 && i >= min_digit) {
        break;
      }
      uint8_t units = value % 10;
      digits[i] = (GlyphLibrary::GlyphIndex)units;
      value = (value - units) / 10;
    }
    return Glyphs(  // rendering de gauche à droite
      GlyphLibrary::get_glyph(digits[3]),
      GlyphLibrary::get_glyph(digits[2]),
      GlyphLibrary::get_glyph(digits[1]),
      GlyphLibrary::get_glyph(digits[0]));
  }

private:
  static const Glyphs build_test() {
    Glyph full = GlyphLibrary::get_glyph(GlyphLibrary::DIGIT_EIGHT);
    full.or_with(GlyphLibrary::get_glyph(GlyphLibrary::SYMBOL_PERIOD));
    return Glyphs(full, full, full, full);
  }

  static const Glyphs build_init() {
    Glyph i = GlyphLibrary::get_glyph(GlyphLibrary::LETTER_I_LOWERCASE);
    return Glyphs(
      i,
      GlyphLibrary::get_glyph(GlyphLibrary::LETTER_N_LOWERCASE),
      i,
      GlyphLibrary::get_glyph(GlyphLibrary::LETTER_T_LOWERCASE));
  }

  static const Glyphs build_empty() {
    Glyph empty = GlyphLibrary::get_glyph(GlyphLibrary::SYMBOL_SPACE);
    return Glyphs(empty, empty, empty, empty);
  }

  Glyph glyphs[DISPLAY_MAX];
};

const Glyphs Glyphs::INIT = Glyphs::build_init();
const Glyphs Glyphs::TEST = Glyphs::build_test();
const Glyphs Glyphs::EMPTY = Glyphs::build_empty();

class FourDigitDisplay {
public:
  FourDigitDisplay(
    const SevenSegmentPins& segment_pins,
    const uint8_t pin_digit1,
    const uint8_t pin_digit2,
    const uint8_t pin_digit3,
    const uint8_t pin_digit4)
    : segment_displays{
        SegmentDisplay(segment_pins, pin_digit1),
        SegmentDisplay(segment_pins, pin_digit2),
        SegmentDisplay(segment_pins, pin_digit3),
        SegmentDisplay(segment_pins, pin_digit4),
      } {}

  void setup() const {
    for (uint8_t i = 0; i < DISPLAY_MAX; i++) {
      segment_displays[(DisplayIndex)i].setup();
    }
  }

  void render_glyphs(const Glyphs& glyphs) const {
    for (uint8_t i = 0; i < DISPLAY_MAX; i++) {
      const Glyph& glyph = glyphs.get_glyph_ref((DisplayIndex)i);
      segment_displays[i].render_glyph(glyph);
    }
  }

private:
  const SegmentDisplay segment_displays[DISPLAY_MAX];
};

typedef void (*IsrFunc)(void);

class Encoder {
public:
  Encoder(
    const uint16_t pulse_per_revolution,
    const uint8_t pin_a,
    const uint8_t pin_b
#if defined(USE_Z_RESET)
    ,
    const uint8_t pin_z,
    const uint8_t pin_z_active_state
#endif  // USE_Z_RESET
    )
    : pulse_per_revolution(pulse_per_revolution),
      // Encoder count of A/B signal *changes* in a single turn
      // = max raw value before over/under flowing
      raw_value_range(pulse_per_revolution << 2),
      pin_a(pin_a),
      pin_b(pin_b)
#if defined(USE_Z_RESET)
      ,
      pin_z(pin_z),
      pin_z_active_state(pin_z_active_state)
#endif  // USE_Z_RESET
  {
  }

  void setup() const {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
#if defined(USE_Z_RESET)
    pinMode(pin_z, INPUT);
#endif  // USE_Z_RESET
  }

  uint8_t get_pin_a() const {
    return digitalRead(pin_a);
  }

  uint8_t get_pin_b() const {
    return digitalRead(pin_b);
  }

#if defined(USE_Z_RESET)
  bool reset_detected() const {
    return digitalRead(pin_z) == pin_z_active_state;
  }
#endif  // USE_Z_RESET

  void uninstall_all_isr() {
    detachInterrupt(digitalPinToInterrupt(pin_a));
    detachInterrupt(digitalPinToInterrupt(pin_b));
  }

  void install_isr_change_a_b(IsrFunc isr) {
    attachInterrupt(digitalPinToInterrupt(pin_a), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_b), isr, CHANGE);
  }

  void install_isr_rising_a(IsrFunc isr) {
    attachInterrupt(digitalPinToInterrupt(pin_a), isr, RISING);
  }

public:
  const uint16_t pulse_per_revolution;
  const uint16_t raw_value_range;

private:
  const uint8_t pin_a;
  const uint8_t pin_b;
#if defined(USE_Z_RESET)
  const uint8_t pin_z;
  const uint8_t pin_z_active_state;
#endif  // USE_Z_RESET
};

class Quadrature {
public:
  Quadrature(const Encoder& encoder)
    : encoder(encoder), state(0), counter(0), relative_zero(0), error_flag(0) {
#if defined(USE_Z_RESET)
    require_homing();
#endif  // USE_Z_RESET
  }

  void update_state_from_inputs() {
    state = ((state & 0b0011) << 2) | ((encoder.get_pin_b() & 1) << 1) | (encoder.get_pin_a() & 1);
  }

  void update_counter_from_quadrature() {
    // branchless counter update (increment, decrement, or keep)
    const uint16_t result = LOOKUP[state];
    const int8_t delta = (int8_t)result & 0xFF;
    counter += delta;
    // branchless error tracking for later handling
    const uint8_t error = (uint8_t)result >> 8;
    error_flag |= error;
#if defined(USE_Z_RESET)
    // branchless counter reset:
    // if reset is true (1) --> counter is and'ed with 0L (erased)
    // if reset is false (0) --> counter is and'ed with 0xFFFFFFFF (kept intact)
    const bool reset = encoder.reset_detected();
    const uint32_t mask = ((uint32_t)reset) - 1;
    counter &= mask;
    // branchless homing flag
    homing_canary = homing_canary || reset;
#endif  // USE_Z_RESET
  }

  void setup() {
    // build a valid state from two reads
    update_state_from_inputs();
    update_state_from_inputs();
  }

  void clear_counter() {
    counter = 0;
  }

  void increment_counter() {
    counter++;
  }

  uint32_t get_counter() const {
    return counter - relative_zero;
  }

  void set_relative_zero() {
    relative_zero = counter;
  }

  uint8_t get_error() const {
    return error_flag;
  }

  void clear_error() {
    error_flag = 0;
  }

#if defined(USE_Z_RESET)
  void require_homing() {
    homing_canary = false;
  }

  bool is_homed() const {
    return homing_canary;
  }
#endif  // USE_Z_RESET

  // 1 state bit for dual signal, and a before and after => 4 bits required
  static const uint8_t LOOKUP_SIZE = (1 << 4);

  typedef enum {
    CCW = 0x00FF,        // -1 without error
    NOOP = 0x0000,       // 0 without error
    CW = 0x0001,         // +1 without error
    INVALID_3 = 0x8300,  // 0 with error and state was 0b0011
    INVALID_6 = 0x8600,  // 0 with error and state was 0b0110
    INVALID_9 = 0x8900,  // 0 with error and state was 0b1001
    INVALID_C = 0x8C00,  // 0 with error and state was 0b1100
  } LookupResult;

  static const uint16_t LOOKUP[];

private:
  const Encoder& encoder;
  volatile uint8_t state;
  volatile uint32_t counter;
  uint32_t relative_zero;
  volatile uint8_t error_flag;
#if defined(USE_Z_RESET)
  volatile bool homing_canary;
#endif  // USE_Z_RESET
};

const uint16_t Quadrature::LOOKUP[LOOKUP_SIZE] = {
  /* i old new        */
  /*    ba BA         */
  /* 0  00 00 same    */ NOOP,
  /* 1  00 01 cw 1    */ CW,
  /* 2  00 10 ccw 1   */ CCW,
  /* 3  00 11 invalid */ INVALID_3,
  /* 4  01 00 ccw 4   */ CCW,
  /* 5  01 01 same    */ NOOP,
  /* 6  01 10 invalid */ INVALID_6,
  /* 7  01 11 cw 2 Z  */ CW,
  /* 8  10 00 cw 4    */ CW,
  /* 9  10 01 invalid */ INVALID_9,
  /* A  10 10 same    */ NOOP,
  /* B  10 11 ccw 2   */ CCW,
  /* C  11 00 invalid */ INVALID_C,
  /* D  11 01 ccw 3 Z */ CCW,
  /* E  11 10 cw 3 Z  */ CW,
  /* F  11 11 same    */ NOOP
};

#if defined(USE_ISR_WAVEFORM)
class IsrMonitor {
public:
  IsrMonitor(const uint8_t pin_waveform)
    : pin_waveform(pin_waveform) {}

  void setup() const {
    pinMode(pin_waveform, OUTPUT);
    digitalWrite(pin_waveform, LOW);
  }

  void start() const {
    digitalWrite(pin_waveform, HIGH);
  }

  void stop() const {
    digitalWrite(pin_waveform, LOW);
  }

private:
  uint8_t pin_waveform;
};
#endif  // USE_ISR_WAVEFORM

class DebouncedButton {
public:
  DebouncedButton(
    const uint8_t pin_button,
    const uint8_t active_state,
    const uint32_t debounce_millis,
    const uint32_t long_press_millis)
    : pin_button(pin_button),
      active_state(active_state),
      debounce_millis(debounce_millis),
      long_press_millis(long_press_millis),
      last_value(0),
      stored_state(0),
      last_change_millis(0L),
      long_press_has_been_triggered(false) {}

  void setup() {
    pinMode(pin_button, INPUT);
    last_value = stored_state = digitalRead(pin_button);
    last_change_millis = millis();
  }

  bool update() {
    // read and debounce
    uint8_t current_state = digitalRead(pin_button);
    uint32_t current_millis = millis();
    if (current_state != stored_state) {
      stored_state = current_state;
      last_change_millis = current_millis;
    }
    if (stored_state != last_value && current_millis - last_change_millis > debounce_millis) {
      last_value = stored_state;  // change has persisted for long enough
      if (long_press_has_been_triggered && stored_state != active_state) {
        return false;  // after a long press, do NOT notify of button RELEASE
      }
      long_press_has_been_triggered = false;  // clear the long press event masking lag
      return true;                            // notify on value change
    }
    return false;
  }

  bool detect_long_press() {
    const uint32_t current_millis = millis();
    const uint32_t duration_millis = current_millis - last_change_millis;
    // 1) triggers on continuous press, so do not trigger while released
    // 2) do not trigger before a long press could have happened
    // 3) do not trigger if the long press already happened
    if (stored_state != active_state || duration_millis < long_press_millis || long_press_has_been_triggered) {
      return false;
    }
    // Mark it as happened so no more event
    long_press_has_been_triggered = true;
    return true;
  }

  bool is_active() const {
    return stored_state == active_state;
  }

private:
  const uint8_t pin_button;
  const uint8_t active_state;
  const uint32_t debounce_millis;
  const uint32_t long_press_millis;
  uint8_t last_value;
  uint8_t stored_state;
  uint32_t last_change_millis;
  bool long_press_has_been_triggered;
};

/************************* Global variables (required for interrupts) *************************/

Encoder global_encoder(
  CONFIG_ENCODER_PULSES_PER_REVOLUTION,
  CONFIG_PIN_IN_QUAD_A,
  CONFIG_PIN_IN_QUAD_B
#if defined(USE_Z_RESET)
  ,
  CONFIG_PIN_IN_QUAD_Z,
  CONFIG_PIN_IN_QUAD_Z_ACTIVE_STATE
#endif  // USE_Z_RESET
);

Quadrature global_quadrature(global_encoder);

/************************* Interrup service routines (need global variables) *************************/

#if defined(USE_ISR_WAVEFORM)
IsrMonitor global_isr_monitor(CONFIG_PIN_OUT_ISR_WAVEFORM);
#endif  // USE_ISR_WAVEFORM

void isr_position() {
#if defined(USE_ISR_WAVEFORM)
  global_isr_monitor.start();
#endif  // USE_ISR_WAVEFORM
  global_quadrature.update_state_from_inputs();
  global_quadrature.update_counter_from_quadrature();
#if defined(USE_ISR_WAVEFORM)
  global_isr_monitor.stop();
#endif  // USE_ISR_WAVEFORM
}

void isr_speed() {
#if defined(USE_ISR_WAVEFORM)
  global_isr_monitor.start();
#endif  // USE_ISR_WAVEFORM
  global_quadrature.increment_counter();
#if defined(USE_ISR_WAVEFORM)
  global_isr_monitor.stop();
#endif  // USE_ISR_WAVEFORM
}

/************************* Application code (needs ISR routines) *************************/

class RpmCalculator {
public:
  RpmCalculator(const Encoder& encoder, const Quadrature& quadrature)
    : encoder(encoder), quadrature(quadrature) {
    reset();
  }

  void reset() {
    last_rpm_computation_millis = millis();
    last_counter = quadrature.get_counter();
    rpm_value = 0;
  }

  void update() {
    uint32_t current_millis = millis();
    uint32_t interval_millis = current_millis - last_rpm_computation_millis;
    // only update every once in a while
    if (interval_millis > CONFIG_UI_INTERVAL_MILLIS) {
      last_rpm_computation_millis = current_millis;
      uint32_t current_counter = quadrature.get_counter();
      uint16_t counter_diff = current_counter - last_counter;
      last_counter = current_counter;
      // convert counter difference to "per minute" and switch to "millis"
      // divide by elasped time, keep only full turns, clip to display, and store
      uint32_t result = 60000 * counter_diff;
      result /= interval_millis * encoder.pulse_per_revolution;
      if (result > 9999) {
        result = 9999;
      }
      rpm_value = result;
    }
  }

  uint16_t get_rpm() const {
    return rpm_value;
  }

private:
  const Encoder& encoder;
  const Quadrature& quadrature;
  uint32_t last_rpm_computation_millis;
  uint32_t last_counter;
  uint16_t rpm_value;
};

class Application {
public:
  typedef enum {
    TEST = 0,
    RPM,
    DEGREES,
#if defined(USE_RAW_DISPLAY)
    RAW,
#endif  // USE_RAW_DISPLAY
    ERROR,
#if defined(USE_Z_RESET)
    INIT,
#endif  // USE_Z_RESET
  } Mode;

  Application(
    Encoder& encoder,
    Quadrature& quadrature,
    FourDigitDisplay& display,
    DebouncedButton& button)
    : encoder(encoder),
      quadrature(quadrature),
      display(display),
      button(button),
      mode(TEST),
      self_test_done(false),
      rpm_calculator(encoder, quadrature) {}

  void setup() {
    set_mode_test();
  }

  void request_mode(Mode requested_mode) {
    if (requested_mode == mode) {
      return;
    }
    switch (requested_mode) {
      case TEST:
        set_mode_test();
        break;
      case RPM:
        set_mode_rpm();
        break;
      case DEGREES:
        set_mode_degrees();
        break;
#if defined(USE_RAW_DISPLAY)
      case RAW:
        set_mode_raw();
        break;
#endif  // USE_RAW_DISPLAY
      case ERROR:
        set_mode_error();
        break;
#if defined(USE_Z_RESET)
      case INIT:
        set_mode_init();
        break;
#endif  // USE_Z_RESET
    }
  }

  void loop_test() {
    if (!self_test_done && millis() > CONFIG_DISPLAY_MODE_TEST_DURATION_MILLIS) {
      self_test_done = true;
      request_mode(RPM);
    }
    display.render_glyphs(Glyphs::TEST);
  }


  void loop_rpm() {
    rpm_calculator.update();
    Glyphs glyphs = Glyphs::build_uint16(rpm_calculator.get_rpm(), 1);
    display.render_glyphs(glyphs);
  }

  void loop_degrees() {
    uint32_t counter = quadrature.get_counter() % encoder.raw_value_range;
    counter = 360 * 10 * counter / encoder.raw_value_range;
    Glyphs glyphs = Glyphs::build_uint16(counter, 2);
    const Glyph& period = GlyphLibrary::get_glyph(GlyphLibrary::SYMBOL_PERIOD);
    glyphs.get_glyph(DisplayIndex::DISPLAY_3).or_with(period);
    display.render_glyphs(glyphs);
  }

#if defined(USE_RAW_DISPLAY)
  void loop_raw() const {
    uint32_t counter = quadrature.get_counter() % encoder.raw_value_range;
    Glyphs glyphs = Glyphs::build_uint16(counter, 1);
    // overlay period on all digits to show we are in RAW mode
    const Glyph& period = GlyphLibrary::get_glyph(GlyphLibrary::SYMBOL_PERIOD);
    for (int i = 0; i < DISPLAY_MAX; i++) {
      Glyph& glyph = glyphs.get_glyph((DisplayIndex)i);
      glyph.or_with(period);
    }
    display.render_glyphs(glyphs);
  }
#endif  // USE_RAW_DISPLAY

  void loop_error(uint8_t error_code) {
    Glyphs glyphs = Glyphs::build_uint16(error_code, 3);
    const Glyph& letter_e = GlyphLibrary::get_glyph(GlyphLibrary::LETTER_E_UPPERCASE);
    glyphs.get_glyph(DisplayIndex::DISPLAY_1).or_with(letter_e);
    display.render_glyphs(glyphs);
  }

#if defined(USE_Z_RESET)
  void loop_init() {
    if (quadrature.is_homed()) {
      request_mode(DEGREES);
    }
    display.render_glyphs(Glyphs::INIT);
  }
#endif  // USE_Z_RESET

  void loop() {
    bool button_short_press = button.update() && !button.is_active();  // order is important
    bool button_long_press = button.detect_long_press();               // must be called after update

    // error management by priority
    uint8_t error = quadrature.get_error();
    if (error) {
      request_mode(ERROR);
    }

    switch (mode) {
      case TEST:
        loop_test();
        break;
      case RPM:
        loop_rpm();
        if (button_short_press) {
#if defined(USE_Z_RESET)
          request_mode(INIT);
#else
          request_mode(DEGREES);
#endif  // USE_Z_RESET
        }
        break;
      case DEGREES:
        if (button_long_press) {
          SERIAL_PRINTLN("Setting new zero");
          quadrature.set_relative_zero();
        }
        loop_degrees();
        if (button_short_press) {
#if defined(USE_RAW_DISPLAY)
          request_mode(RAW);
#else
          request_mode(RPM);
#endif  // USE_RAW_DISPLAY
        }
        break;
#if defined(USE_RAW_DISPLAY)
      case RAW:
        if (button_long_press) {
          SERIAL_PRINTLN("Setting new zero");
          quadrature.set_relative_zero();
        }
        loop_raw();
        if (button_short_press) {
          request_mode(RPM);
        }
        break;
#endif  // USE_RAW_DISPLAY
      case ERROR:
        loop_error(error);
        if (button_short_press) {
          quadrature.clear_error();
          request_mode(TEST);
        }
        break;
#if defined(USE_Z_RESET)
      case INIT:
        loop_init();
        if (button_short_press) {
#if defined(USE_RAW_DISPLAY)
          request_mode(RAW);
#else
          request_mode(RPM);
#endif  // USE_RAW_DISPLAY
        }
        break;
#endif  // USE_Z_RESET
    }
  }

private:
  void set_mode_test() {
    encoder.uninstall_all_isr();
    mode = TEST;
    self_test_done = false;
    SERIAL_PRINTLN("Mode TEST");
    quadrature.clear_counter();
  }

  void set_mode_rpm() {
    encoder.uninstall_all_isr();
    mode = RPM;
    SERIAL_PRINTLN("Mode RPM");
    quadrature.clear_counter();
    rpm_calculator.reset();
    encoder.install_isr_rising_a(isr_speed);
  }

  void set_mode_degrees() {
    encoder.uninstall_all_isr();
    mode = DEGREES;
    SERIAL_PRINTLN("Mode DEGREES");
    quadrature.clear_counter();
    encoder.install_isr_change_a_b(isr_position);
  }

#if defined(USE_RAW_DISPLAY)
  void set_mode_raw() {
    encoder.uninstall_all_isr();
    mode = RAW;
    SERIAL_PRINTLN("Mode RAW");
    quadrature.clear_counter();
    encoder.install_isr_change_a_b(isr_position);
  }
#endif  // USE_RAW_DISPLAY

  void set_mode_error() {
    encoder.uninstall_all_isr();
    mode = ERROR;
    SERIAL_PRINTLN("Mode ERROR");
  }

#if defined(USE_Z_RESET)
  void set_mode_init() {
    encoder.uninstall_all_isr();
    mode = INIT;
    SERIAL_PRINTLN("Mode INIT");
    quadrature.clear_counter();
    quadrature.require_homing();
    encoder.install_isr_change_a_b(isr_position);
  }
#endif  // USE_Z_RESET

  Encoder& encoder;
  Quadrature& quadrature;
  FourDigitDisplay& display;
  DebouncedButton& button;
  Mode mode;
  bool self_test_done;
  RpmCalculator rpm_calculator;
};

/************************* Other global variables *************************/

SevenSegmentPins global_segment_pins(
  DisplayType::COMMON_CATHODE,
  CONFIG_PIN_OUT_SEGMENT_A,
  CONFIG_PIN_OUT_SEGMENT_B,
  CONFIG_PIN_OUT_SEGMENT_C,
  CONFIG_PIN_OUT_SEGMENT_D,
  CONFIG_PIN_OUT_SEGMENT_E,
  CONFIG_PIN_OUT_SEGMENT_F,
  CONFIG_PIN_OUT_SEGMENT_G,
  CONFIG_PIN_OUT_SEGMENT_DP,
  CONFIG_SEGMENT_ON_TIME_MICROS);

FourDigitDisplay global_display(
  global_segment_pins,
  /* FIXME : might be inverted */
  CONFIG_PIN_OUT_DIGIT_1,
  CONFIG_PIN_OUT_DIGIT_2,
  CONFIG_PIN_OUT_DIGIT_3,
  CONFIG_PIN_OUT_DIGIT_4);

DebouncedButton global_button(
  CONFIG_PIN_IN_BUTTON,
  CONFIG_PIN_IN_BUTTON_ACTIVE_STATE,
  CONFIG_BUTTON_DEBOUNCE_MILLIS,
  CONFIG_BUTTON_LONG_PRESS_MILLIS);

/************************* Arduino framework *************************/

Application global_application(
  global_encoder,
  global_quadrature,
  global_display,
  global_button);

void setup() {
  SERIAL_BEGIN(9600);
  SERIAL_PRINTLN("Starting...");
#if defined(USE_ISR_WAVEFORM)
  global_isr_monitor.setup();
#endif  // USE_ISR_WAVEFORM
  global_encoder.setup();
  global_quadrature.setup();
  global_segment_pins.setup();
  global_display.setup();
  global_button.setup();
  global_application.setup();
}

void loop() {
  global_application.loop();
}
