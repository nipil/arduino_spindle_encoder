#define ENCODER_PULSE_PER_REV 1024

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

volatile int16_t position = 0;

int16_t last_position = 0;

volatile uint8_t quadrature = 0;

volatile uint8_t last_invalid_quadrature = 0;

int8_t delta[16] = {
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
    } else if (quadrature == 0xD) {
      // CCW reset
      position = -1;
    }
  }
}

const float DEGREE_PER_CHANGE = 360.0f / ((float)(ENCODER_PULSE_PER_REV << 2));  // times 4 for number of *changes* per revolution

inline float angle_from_position(float pos) {
  return DEGREE_PER_CHANGE * pos;
}

uint8_t current_digit = 0;

uint8_t digit_pins[NUM_DIGITS] = {
  OUT_DIGIT_1,
  OUT_DIGIT_2,
  OUT_DIGIT_3,
  OUT_DIGIT_4,
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

void setup() {
  Serial.begin(9600);
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
  for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
    pinMode(segment_pins[i], OUTPUT);
    digitalWrite(segment_pins[i], LOW);
  }
  for (uint8_t i = 0; i < NUM_DIGITS; i++) {
    pinMode(digit_pins[i], OUTPUT);
    digitalWrite(digit_pins[i], HIGH);
  }
  Serial.print("Angular resolution: ");
  Serial.print(DEGREE_PER_CHANGE);
  Serial.println(" degrees per quadrature edge");
  pinMode(IN_QUAD_Z, INPUT);
  pinMode(IN_QUAD_A, INPUT);
  pinMode(IN_QUAD_B, INPUT);
  quadrature = (((digitalRead(IN_QUAD_B) << 1) + digitalRead(IN_QUAD_A)) << 2) & 0xF;
  attachInterrupt(digitalPinToInterrupt(IN_QUAD_A), quad_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IN_QUAD_B), quad_changed, CHANGE);
}

void loop() {

  if (last_invalid_quadrature != 0) {
    Serial.print("Invalid quadrature: ");
    Serial.println(last_invalid_quadrature);
    last_invalid_quadrature = 0;
  }

  if (position != last_position) {
    last_position = position;
    float angle = angle_from_position(last_position);
    Serial.print(angle);
    Serial.println("°");
  }

  digitalWrite(digit_pins[current_digit], LOW);
  digitalWrite(segment_pins[current_segment], HIGH);

  delay(100);

  digitalWrite(segment_pins[current_segment], LOW);
  digitalWrite(digit_pins[current_digit], HIGH);

  current_segment++;

  if (current_segment == NUM_SEGMENTS) {
    current_segment = 0;
    current_digit++;
  }

  if (current_digit == NUM_DIGITS) {
    current_digit = 0;
  }
}
