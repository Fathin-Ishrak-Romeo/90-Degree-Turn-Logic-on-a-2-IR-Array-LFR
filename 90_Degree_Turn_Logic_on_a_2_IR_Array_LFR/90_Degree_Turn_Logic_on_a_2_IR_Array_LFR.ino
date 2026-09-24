// ===================== PINS =====================
#define enA 10   // Enable1 L298 Pin enA  -> RIGHT motors speed
#define in1 9    // Motor1 (RIGHT) forward pin
#define in2 8    // Motor1 (RIGHT) backward pin
#define in3 7    // Motor2 (LEFT)  backward pin
#define in4 6    // Motor2 (LEFT)  forward pin
#define enB 5    // Enable2 L298 Pin enB  -> LEFT motors speed
#define R_S A0   // IR sensor Right
#define L_S A1   // IR sensor Left

// ===================== CONFIG =====================
// Sensor output when it is over the BLACK line (reference sketch uses 1 = black)
#define LINE_LEVEL HIGH

// Both sensors on black: 1 = stop (like the reference sketch), 0 = keep going
#define STOP_ON_BOTH_SENSORS 1

// Set to 1 to print sensor/state info in Serial Monitor (9600 baud) while testing
#define DEBUG 0
const unsigned long DEBUG_PRINT_INTERVAL_MS = 100; // throttle so prints don't flood/slow the loop

// ---------- Speeds (0-255) - tune these ----------
// -- Speed scheduling (#3) --
const int CRUISE_SPEED       = 70;  // steady speed once centered & stable
const int RECOVERY_SPEED     = 60;   // speed right as a turn finishes
const unsigned long ACCEL_RAMP_MS = 250; // ms to ramp RECOVERY_SPEED -> CRUISE_SPEED after re-centering

// -- Turn entry (#2) --
const int SOFT_OUTER_SPEED   = 190;  // outer side during the brief entry kick
const int SOFT_INNER_SPEED   = -50;  // inner side during entry kick: light REVERSE (was 0/braked)
const unsigned long SOFT_TURN_MS   = 10;  // entry-kick duration - just enough to start the yaw
const int PIVOT_START_SPEED  = 110;  // pivot begins here - already strong from the start
const int PIVOT_MAX_SPEED    = 200;  // ...and ramps up to this
const int PIVOT_RAMP_MS_PER_PWM = 6; // +1 PWM every 2ms while pivoting

// -- Anti-overshoot counter-pulse after a pivot --
const int COUNTER_PULSE_SPEED = 200; // reverse-spin strength after a pivot
const int COUNTER_PULSE_MS    = 35;  // 0 = disable counter-pulse

// -- Line-loss hysteresis (#1) --
const unsigned long LOST_GRACE_MS = 60; // keep steering toward last-seen side for up to this long
                                         // when both sensors go white, before accepting "straight"

// ===================== STATE =====================
enum TurnDir { NONE, TURN_LEFT, TURN_RIGHT };

// Active-maneuver state (drives the pivot ramp + counter-pulse)
TurnDir currentTurn = NONE;
unsigned long turnStart = 0;
bool wasPivoting = false;

// line-loss / speed-scheduling bookkeeping
TurnDir lastDir = NONE;              // which side a sensor last actually saw black on
unsigned long lastBlackTime = 0;     // millis() timestamp of that last black reading
unsigned long straightEnteredAt = 0; // millis() we started cruising straight (for the accel ramp)

#if DEBUG
unsigned long lastDebugPrint = 0;
#endif

// ===================== MOTOR CONTROL =====================
// speed: -255..255  (positive = forward, negative = reverse, 0 = active brake)

// RIGHT side: forward = in1 HIGH, in2 LOW
void rightMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    analogWrite(enA, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    analogWrite(enA, -speed);
  } else {
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);  // brake
    analogWrite(enA, 255);
  }
}

// LEFT side: forward = in3 LOW, in4 HIGH
void leftMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    analogWrite(enB, speed);
  } else if (speed < 0) {
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enB, -speed);
  } else {
    digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);  // brake
    analogWrite(enB, 255);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  leftMotors(leftSpeed);
  rightMotors(rightSpeed);
}

void stopMotors() {
  // Free stop, exactly like the reference Stop() function
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// ===================== SPEED SCHEDULING (#3) =====================
// Ramps from RECOVERY_SPEED up to CRUISE_SPEED over ACCEL_RAMP_MS,
// measured from the moment we most recently finished a turn.
int cruiseSpeedNow() {
  unsigned long dt = millis() - straightEnteredAt;
  if (dt >= ACCEL_RAMP_MS) return CRUISE_SPEED;
  long ramped = RECOVERY_SPEED +
                (long)(CRUISE_SPEED - RECOVERY_SPEED) * (long)dt / (long)ACCEL_RAMP_MS;
  return (int)ramped;
}

// ===================== TURN LOGIC (#2) =====================
void handleTurn(TurnDir dir) {
  if (currentTurn != dir) {     // new turn -> restart the timer
    currentTurn = dir;
    turnStart = millis();
    wasPivoting = false;
  }

  unsigned long t = millis() - turnStart;
  int outer, inner;

  if (t < SOFT_TURN_MS) {
    // Stage 1: brief entry kick - outer fast, inner light reverse
    // (starts the yaw immediately instead of coasting straight)
    outer = SOFT_OUTER_SPEED;
    inner = SOFT_INNER_SPEED;
  } else {
    // Stage 2: pivot, power ramping up from an already-strong start
    wasPivoting = true;
    int p = PIVOT_START_SPEED + (int)((t - SOFT_TURN_MS) / PIVOT_RAMP_MS_PER_PWM);
    p = constrain(p, 0, PIVOT_MAX_SPEED);
    outer = p;
    inner = -p;
  }

  if (dir == TURN_LEFT) drive(inner, outer);   // left sensor on line  -> turn left
  else                  drive(outer, inner);   // right sensor on line -> turn right
}

// Brief reverse pulse after a pivot to cancel rotation momentum
void finishTurn() {
  if (wasPivoting && COUNTER_PULSE_MS > 0) {
    if (currentTurn == TURN_LEFT)  drive(COUNTER_PULSE_SPEED, -COUNTER_PULSE_SPEED);
    else                           drive(-COUNTER_PULSE_SPEED, COUNTER_PULSE_SPEED);
    delay(COUNTER_PULSE_MS);
  }
  currentTurn = NONE;
  wasPivoting = false;
}

// ===================== SETUP / LOOP =====================
void setup() {
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

#if DEBUG
  Serial.begin(9600);
#endif

  stopMotors();
  delay(2000);   // time to place the robot on the track
  straightEnteredAt = millis(); // start the accel ramp from a gentle launch
}

void loop() {
  bool L = (digitalRead(L_S) == LINE_LEVEL);   // true = left sensor on black
  bool R = (digitalRead(R_S) == LINE_LEVEL);   // true = right sensor on black
  unsigned long now = millis();

#if DEBUG
  char stateChar = 'S';
#endif

  if (L && R) {
    // Both on black: finish line / intersection
    if (currentTurn != NONE) finishTurn();
    lastDir = NONE;
    straightEnteredAt = now;
#if STOP_ON_BOTH_SENSORS
    stopMotors();
#else
    { int s = cruiseSpeedNow(); drive(s, s); }
#endif
#if DEBUG
    stateChar = 'X'; // both black
#endif
  }
  else if (L || R) {
    // One sensor sees the line -> actively turning
    TurnDir dir = L ? TURN_LEFT : TURN_RIGHT;
    lastDir = dir;
    lastBlackTime = now;
    handleTurn(dir);
#if DEBUG
    stateChar = (dir == TURN_LEFT) ? 'L' : 'R';
#endif
  }
  else {
    // Both white: either genuinely centered, or mid-turn with a brief gap
    if (lastDir != NONE && (now - lastBlackTime) <= LOST_GRACE_MS) {
      // Grace window: keep steering toward the last-seen side.
      // handleTurn() sees currentTurn == lastDir already, so this
      // simply continues the same ramp rather than restarting it.
      handleTurn(lastDir);
#if DEBUG
      stateChar = 'g'; // grace hold
#endif
    } else {
      // Genuinely straight: finish any pending turn (counter-pulse once),
      // start the speed-scheduling ramp, and cruise.
      if (currentTurn != NONE) {
        finishTurn();
        straightEnteredAt = now;
      }
      lastDir = NONE;
      int s = cruiseSpeedNow();
      drive(s, s);
#if DEBUG
      stateChar = 'S';
#endif
    }
  }

#if DEBUG
  if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrint = now;
    Serial.print("L:"); Serial.print(L);
    Serial.print(" R:"); Serial.print(R);
    Serial.print(" state:"); Serial.print(stateChar);
    Serial.print(" spd:"); Serial.println(cruiseSpeedNow());
  }
#endif
}
