#define ENCODER_PULSE_PER_REV 1024

const int PIN_A = 2;
const int PIN_B = 3;
const int PIN_Z = 4;

volatile short position = 0;

short last_position = 0;

volatile int quadrature = 0;

volatile int last_invalid_quadrature = 0;

char delta[16] = {
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
  quadrature = ((quadrature << 2) + (digitalRead(PIN_B) << 1) + digitalRead(PIN_A)) & 0xF;

  int change = delta[quadrature];
  if (change >= 2) {
    last_invalid_quadrature = quadrature;
    return;
  }
  position += change;

  // reset using Z pin
  if (digitalRead(PIN_Z) == 1) {
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

void setup() {
  Serial.begin(9600);
  Serial.print("Angular resolution: ");
  Serial.print(DEGREE_PER_CHANGE);
  Serial.println(" degrees per quadrature edge");
  pinMode(PIN_Z, INPUT);
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  quadrature = (((digitalRead(PIN_B) << 1) + digitalRead(PIN_A)) << 2) & 0xF;
  attachInterrupt(digitalPinToInterrupt(PIN_A), quad_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), quad_changed, CHANGE);
}

void loop() {
  delay(100);

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
}
