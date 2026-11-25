#include <Wire.h>
#include <DFPongController.h>
#include <Adafruit_APDS9999.h>
#include <Adafruit_LSM6DS3TRC.h>

const int   CONTROLLER_NUMBER = 17;

const int LED_PINS[3] = {12, 27, 33};
const unsigned long TARGET_ON_MS   = 2000;
const unsigned long TARGET_GAP_MS  = 5000;
const unsigned long TARGET_BLINK_MS = 250;
const unsigned long WRONG_FLASH_MS = 150;

const int   PROX_FAR_RAW  = 6;
const int   PROX_NEAR_RAW = 255;
const bool  INVERT_PROXIMITY = false;

const float ZONE_UP_MAX   = 33.3;
const float ZONE_DOWN_MIN = 66.7;
const float HYSTERESIS    = 3.0;
const float SMOOTHING     = 0.3;

const float FLICK_THRESHOLD = 7.0;
const float FLICK_RESET     = 18.0;

const int SIZE_FOR_HITS[4] = {100, 125, 150, 200};

const char* HIT_SOUND   = "laser";
const char* SCORE_SOUND = "https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/onze%20death%20sound.mp3";
const char* WIN_SOUND   = "https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/douze%20victory.mp3";
const char* MUSIC       = "arcade";

const int COLOR_FOR_HITS[4][3] = { {0, 200, 255}, {80, 255, 80}, {255, 220, 0}, {255, 40, 40} };

const unsigned long POINT_DEBOUNCE_MS      = 2000;
const unsigned long BUFF_RESET_FALLBACK_MS = 8000;

DFPongController    controller;
Adafruit_APDS9999   apds;
Adafruit_LSM6DS3TRC imu;

bool hasProx = false;
bool hasImu  = false;

float proxPercent = 50.0;
int   proxZone    = NEUTRAL;
uint16_t lastHandRaw = 0;
int   handRaw = 0;

const int LED_ZONE[3] = {DOWN, NEUTRAL, UP};
bool          zoneHit[3]      = {false, false, false};
int           hitsThisRound   = 0;
int           targetLed       = -1;
unsigned long targetShownAt   = 0;
unsigned long nextTargetAt    = 0;
unsigned long wrongFlashUntil = 0;

bool flickArmed = true;

bool buffRound = false;
bool resetSent = false;
unsigned long buffRoundStartedAt = 0;
unsigned long lastPointAt = 0;

volatile bool pointEnded  = false;
volatile bool matchStart  = false;
volatile bool matchEnded  = false;
volatile bool playStarted = false;

// Called by the DF Pong library whenever the game sends me an event.
// It only raises flags (point ended, match started or ended, rally started)
// so that loop() can react to them safely.
void gameEvent(int event, int value) {
  Serial.print("[event ");
  Serial.print(event);
  Serial.print(" value ");
  Serial.print(value);
  Serial.println("]");
  if (event == EVENT_COUNTDOWN && value == 0) playStarted = true;
  if (event == EVENT_HIT)                      playStarted = true;
  if (event == EVENT_SCORED || event == EVENT_CONCEDED) pointEnded = true;
  if (event == EVENT_MATCH_START)                        matchStart = true;
  if (event == EVENT_WON || event == EVENT_LOST)         matchEnded = true;
}

// Turns a light number (0, 1 or 2) into the name of its zone (DOWN, STOP
// or UP). I use it for the Serial Monitor messages.
const char* zoneName(int led) {
  return led == 0 ? "DOWN" : led == 1 ? "STOP" : "UP";
}

// Sets the paddle color to match how many lights I have shot this round,
// so the color shows my progress in the game.
void showHitsColor(int hits) {
  controller.setPlayerColor(COLOR_FOR_HITS[hits][0], COLOR_FOR_HITS[hits][1], COLOR_FOR_HITS[hits][2]);
}

// Starts a fresh shooting round: clears all hits, turns the lights off,
// puts the paddle color back to the base color, and schedules the first
// target after a short pause.
void startRound(unsigned long now) {
  buffRound = false;
  for (int i = 0; i < 3; i++) zoneHit[i] = false;
  hitsThisRound = 0;
  showHitsColor(0);
  targetLed     = -1;
  nextTargetAt  = now + TARGET_GAP_MS;
}

// Picks a random light to be the next target, only among the zones I have
// not shot yet. If all three are already shot, there is no target.
void pickNewTarget(unsigned long now) {
  int candidates[3];
  int n = 0;
  for (int i = 0; i < 3; i++) {
    if (!zoneHit[i]) candidates[n++] = i;
  }
  if (n == 0) {
    targetLed = -1;
    return;
  }
  targetLed     = candidates[random(n)];
  targetShownAt = now;
  Serial.print("New target: ");
  Serial.println(zoneName(targetLed));
}

// Reads the raw proximity value from the APDS9999.
// If a reading fails, it returns the last good value instead.
int readProximityRaw() {
  uint16_t prox;
  if (apds.readProximity(&prox)) {
    lastHandRaw = prox;
  }
  return (int)lastHandRaw;
}

// Converts the raw proximity into a position from 0% (far) to 100% (near).
// The curve follows how reflected light fades with distance, so equal steps
// in percent are roughly equal steps along the base.
float proximityToPercent(int raw) {
  float r     = (float)constrain(raw, PROX_FAR_RAW, PROX_NEAR_RAW);
  float cm    = sqrtf((float)PROX_NEAR_RAW / r) - 1.0f;
  float cmMax = sqrtf((float)PROX_NEAR_RAW / (float)PROX_FAR_RAW) - 1.0f;
  cm = constrain(cm, 0.0f, cmMax);
  float p = 100.0f - (cm * 100.0f / cmMax);
  return INVERT_PROXIMITY ? 100.0f - p : p;
}

// Turns the position percentage into one of the three base zones
// (UP, STOP or DOWN). A small dead band around each border stops the paddle
// from flickering between two zones.
int zoneFromPercent(float p, int currentZone) {
  switch (currentZone) {
    case UP:
      return (p > ZONE_UP_MAX + HYSTERESIS) ? ((p >= ZONE_DOWN_MIN) ? DOWN : NEUTRAL) : UP;
    case DOWN:
      return (p < ZONE_DOWN_MIN - HYSTERESIS) ? ((p <= ZONE_UP_MAX) ? UP : NEUTRAL) : DOWN;
    default:
      if (p < ZONE_UP_MAX)   return UP;
      if (p > ZONE_DOWN_MIN) return DOWN;
      return NEUTRAL;
  }
}

// Returns true once per trigger flick.
// It measures the total rotation speed of the gyro on all three axes and
// counts a flick when it passes FLICK_THRESHOLD. It only counts again after
// the rotation drops below FLICK_RESET.
bool detectFlick() {
  if (!hasImu) return false;
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  float mag = sqrtf(gyro.gyro.x * gyro.gyro.x +
                    gyro.gyro.y * gyro.gyro.y +
                    gyro.gyro.z * gyro.gyro.z);

  if (flickArmed && mag > FLICK_THRESHOLD) {
    flickArmed = false;
    return true;
  }
  if (!flickArmed && mag < FLICK_RESET) {
    flickArmed = true;
  }
  return false;
}

// Reacts to the start of a match and to the end of each point.
// After a shooting round with at least one hit, the next round becomes a
// buff round with a bigger paddle. After a round with no hits, a new
// shooting round starts. After a buff round, everything resets. A second
// point event right after the first one is ignored.
void handleMatchStartAndPointEnd(unsigned long now) {
  if (matchStart) {
    matchStart = false;
    controller.setPaddleSize(SIZE_FOR_HITS[0]);
    startRound(now);
    Serial.println("=== MATCH START: paddle normal, new round");
  }

  if (pointEnded) {
    pointEnded = false;
    if (now - lastPointAt < POINT_DEBOUNCE_MS) {
      Serial.println("(second point event ignored)");
      return;
    }
    lastPointAt = now;

    if (!buffRound) {
      if (hitsThisRound > 0) {
        int size = SIZE_FOR_HITS[hitsThisRound];
        controller.setPaddleSize(size);
        buffRound = true;
        resetSent = false;
        playStarted = false;
        buffRoundStartedAt = now;
        targetLed = -1;
        Serial.print("=== SHOOTING ROUND END: ");
        Serial.print(hitsThisRound);
        Serial.print(" hit(s) -> BUFF round, paddle ");
        Serial.print(size);
        Serial.println("%");
      } else {
        startRound(now);
        Serial.println("=== SHOOTING ROUND END: 0 hits -> new shooting round");
      }
    } else {
      controller.setPaddleSize(SIZE_FOR_HITS[0]);
      startRound(now);
      Serial.println("=== BUFF ROUND END: reset, new shooting round");
    }
  }
}

// During the buff round, once the rally has really started, sends the
// normal paddle size so it applies from the next serve.
void handleBuffResetQueue(unsigned long now) {
  if (buffRound && !resetSent &&
      (playStarted || now - buffRoundStartedAt > BUFF_RESET_FALLBACK_MS)) {
    controller.setPaddleSize(SIZE_FOR_HITS[0]);
    resetSent = true;
    Serial.println("Buff round under way: next serve goes back to normal size");
  }
}

// When the whole match is over, resets the paddle size and starts again
// from a fresh shooting round.
void handleMatchEnd(unsigned long now) {
  if (matchEnded) {
    matchEnded = false;
    controller.setPaddleSize(SIZE_FOR_HITS[0]);
    startRound(now);
    Serial.println("=== MATCH END: paddle back to normal");
  }
}

// Reads the proximity sensor and works out which base zone the gun is in
// (UP, STOP or DOWN). The position is smoothed so small jitters do not move
// the paddle.
void updateHandZone() {
  if (hasProx) {
    handRaw = readProximityRaw();
    proxPercent += SMOOTHING * (proximityToPercent(handRaw) - proxPercent);
    proxZone = zoneFromPercent(proxPercent, proxZone);
  }
}

// Controls the target light timing. A target stays on for TARGET_ON_MS,
// then goes dark. After a pause a new random target is picked, as long as
// it is a shooting round and not all three lights are shot yet.
void handleTargetTiming(unsigned long now) {
  if (!buffRound && targetLed >= 0 && (now - targetShownAt >= TARGET_ON_MS)) {
    targetLed    = -1;
    nextTargetAt = now + TARGET_GAP_MS;
    Serial.println("Target expired");
  }
  if (!buffRound && targetLed < 0 && hitsThisRound < 3 && now >= nextTargetAt) {
    pickNewTarget(now);
  }
}

// Handles the trigger. A flick only counts as a hit when there is a target
// light and the gun is in the matching zone. A hit keeps that light on,
// adds one to the count, and sends the new paddle size and color. A flick
// in the wrong zone makes the target flash off briefly. Flicks during the
// buff round are ignored.
void handleFlick(unsigned long now) {
  if (detectFlick()) {
    if (buffRound) {
      Serial.println("Flick ignored: this is the BUFF round");
    } else if (targetLed < 0) {
      Serial.println("Flick but no target LED - nothing");
    } else if (proxZone == LED_ZONE[targetLed]) {
      zoneHit[targetLed] = true;
      hitsThisRound++;
      controller.setPaddleSize(SIZE_FOR_HITS[hitsThisRound]);
      showHitsColor(hitsThisRound);
      Serial.print("HIT ");
      Serial.print(zoneName(targetLed));
      Serial.print("  (hits this round: ");
      Serial.print(hitsThisRound);
      Serial.println(")");
      targetLed    = -1;
      nextTargetAt = now + TARGET_GAP_MS;
    } else {
      wrongFlashUntil = now + WRONG_FLASH_MS;
      Serial.println("Flick in the WRONG zone");
    }
  }
}

// Shows the three lights. Shot zones stay on, the current target blinks,
// and the target goes off for a moment after a wrong flick.
void updateLeds(unsigned long now) {
  for (int i = 0; i < 3; i++) {
    bool on = false;
    if (zoneHit[i]) {
      on = true;
    } else if (i == targetLed) {
      bool blinkOn = (TARGET_BLINK_MS == 0) || (((now / TARGET_BLINK_MS) % 2) == 0);
      on = blinkOn && (now >= wrongFlashUntil);
    }
    digitalWrite(LED_PINS[i], on ? HIGH : LOW);
  }
}

// Sends the paddle command to the game (UP, STOP or DOWN) from the zone
// the gun is in. Returns the command that was sent.
int sendMovement() {
  int command = proxZone;
  controller.sendControl(command);
  return command;
}

void setup() {
  Serial.begin(115200);

#ifdef PIN_I2C_POWER
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif
  delay(50);

  for (int i = 0; i < 3; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  randomSeed(esp_random());
  startRound(millis());

  Wire.begin();

  hasProx = apds.begin();
  if (hasProx) {
    apds.enableProximitySensor(true);
  } else {
    Serial.println("APDS9999 not found - check wiring");
  }

  hasImu = imu.begin_I2C();
  if (!hasImu) {
    Serial.println("LSM6DS3TR-C not found - check wiring");
  } else {
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  }

  controller.setControllerNumber(CONTROLLER_NUMBER);
  controller.setControllerName("Douze");
  controller.setPaddleSize(SIZE_FOR_HITS[0]);
  controller.setHitSound(HIT_SOUND);
  controller.setScoreSound(SCORE_SOUND);
  controller.setWinSound(WIN_SOUND);
  controller.setMusic(MUSIC);
  controller.setStatusLED(LED_BUILTIN);
  controller.onGameEvent(gameEvent);
  controller.begin();
}

void loop() {
  controller.update();
  unsigned long now = millis();

  handleMatchStartAndPointEnd(now);
  handleBuffResetQueue(now);
  handleMatchEnd(now);
  updateHandZone();
  handleTargetTiming(now);
  handleFlick(now);
  updateLeds(now);
  int command = sendMovement();
  printDebug(now, command);

  delay(5);
}

// Prints the live sensor values and state every 200 ms to the Serial
// Monitor, which I use for tuning.
void printDebug(unsigned long now, int command) {
  static unsigned long lastPrint = 0;
  if (now - lastPrint <= 200) return;
  lastPrint = now;
  Serial.print("hand raw: ");
  Serial.print(handRaw);
  Serial.print("  percent: ");
  Serial.print(proxPercent, 1);
  Serial.print("  | target: ");
  Serial.print(targetLed < 0 ? "none" : zoneName(targetLed));
  Serial.print("  | hits: ");
  Serial.print(hitsThisRound);
  Serial.print(buffRound ? " [BUFF ROUND]" : " [shooting]");
  Serial.print("  | cmd: ");
  Serial.println(command == UP ? "UP" : command == DOWN ? "DOWN" : "STOP");