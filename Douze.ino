#include <Wire.h>
#include <DFPongController.h>
#include <Adafruit_APDS9999.h>
#include <Adafruit_LSM6DS3TRC.h>

const int   CONTROLLER_NUMBER = 16;
- const charCONTROLLER_NAME   = 
"Onze";

const int GYRO_LED_PIN = 27;
const int PROX_LED_PIN = 33;

const unsigned long STEP_MS     = 150;
const unsigned long COOLDOWN_MS = 300;

const int   PROX_DELTA        = 6;
const unsigned long PROX_SAMPLE_MS = 30;
const int   PROX_CONFIRM      = 2;
const int   PROX_AVG          = 3;
const float PROX_BASE_SPEED   = 0.01;

const apds9999_prox_resolution_t PROX_RES         = APDS9999_PROX_RES_11BIT;
const apds9999_prox_meas_rate_t  PROX_RATE        = APDS9999_PROX_RATE_6MS;
const apds9999_led_current_t     PROX_LED_CURRENT = APDS9999_LED_CURRENT_25MA;
const uint8_t                    PROX_LED_PULSES  = 16;

const float JOLT_THRESHOLD = 25.0;
const float JOLT_RESET     = 3.0;

const int SIZE_FOR_LEDS[3] = {100, 150, 200};

const unsigned long SIZE_SETTLE_MS = 1500;

const unsigned long POINT_DEBOUNCE_MS      = 2000;
const unsigned long BUFF_RESET_FALLBACK_MS = 8000;

- const char
COLOR_FOR_LEDS[3] = {
  "#39ff14",
  "#ff4500",
  "#ffd700"
};

- const charHIT_SOUND   = 
"wood";
- const charSCORE_SOUND = 
"https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/douze%20death%20sound.mp3";
- const charWIN_SOUND   = 
"https://github.com/lamnpfs2232/Sound/raw/refs/heads/main/onze%20victory.mp3";

DFPongController    controller;
Adafruit_APDS9999   apds;
Adafruit_LSM6DS3TRC imu;

bool hasProx = false;
bool hasImu  = false;

uint16_t lastHandRaw = 0;
int   handRaw      = 0;
float proxBase     = -1;
float proxDiff     = 0;
bool  proxArmed    = true;
int   proxAbove    = 0;
int   proxHist[8];
int   proxHistN    = 0;
unsigned long proxSampleAt = 0;
unsigned long proxLastAt   = 0;
bool  proxLedOn    = false;

float joltValue    = 0;
bool  joltArmed    = true;
unsigned long joltLastAt = 0;
bool  gyroLedOn    = false;

int   stepDirection = NEUTRAL;
unsigned long stepEnd = 0;

bool  buffRound = false;
bool  resetSent = false;
unsigned long buffRoundStartedAt = 0;
unsigned long lastPointAt = 0;

int   sentSize  = -1;
int   sentColor = -1;
bool  sizeDirty = false;
unsigned long ledsChangedAt = 0;

volatile bool pointEnded  = false;
volatile bool matchStart  = false;
volatile bool matchEnded  = false;
volatile bool playStarted = false;

// Được gọi bởi thư viện DF Pong mỗi khi game gửi sự kiện về.
// Hàm này chỉ bật các cờ báo hiệu khi kết thúc điểm,
// để hàm loop() có thể xử lý chúng một cách an toàn.
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

// Đếm số cặp mắt chuột chũi đang sáng hiện tại (0, 1 hoặc 2).
// Con số này quyết định mức buff cho thanh đỡ (paddle).
int ledsLit() {
  return (gyroLedOn ? 1 : 0) + (proxLedOn ? 1 : 0);
}

// Gửi kích thước thanh đỡ lên game, nhưng chỉ gửi khi khác với
// kích thước lần trước để tránh spam cùng một giá trị lên game.
void sendSize(int size) {
  if (size == sentSize) return;
  controller.setPaddleSize(size);
  sentSize = size;
}

// Gửi màu sắc thanh đỡ tương ứng với cấp độ buff (0, 1 hoặc 2) lên game,
// cũng chỉ gửi khi có sự thay đổi.
void sendColor(int level) {
  if (level == sentColor) return;
  controller.setPlayerColor(COLOR_FOR_LEDS[level]);
  sentColor = level;
}

// Bắt đầu một hiệp tích điểm (earn round) mới: tắt cả hai mắt chuột chũi,
// đưa thanh đỡ về kích thước bình thường và màu mặc định.
void startRound() {
  buffRound = false;
  gyroLedOn = false;
  proxLedOn = false;
  sizeDirty = false;
  sendSize(SIZE_FOR_LEDS[0]);
  sendColor(0);
}

// Bắt đầu một bước di chuyển thanh đỡ (UP hoặc DOWN) kéo dài trong STEP_MS.
void startStep(int direction, unsigned long now) {
  stepDirection = direction;
  stepEnd = now + STEP_MS;
}

// Đọc giá trị tiệm cận thô từ cảm biến APDS9999.
// Nếu đọc lỗi, hàm sẽ trả về giá trị hợp lệ gần nhất trước đó.
int readProximityRaw() {
  uint16_t prox;
  if (apds.readProximity(&prox)) {
    lastHandRaw = prox;
  }
  return (int)lastHandRaw;
}

// Nút bên phải: trả về true một lần cho mỗi lần nhấn.
// Hàm tính trung bình vài mẫu tiệm cận gần nhất và so sánh với mức nền (baseline)
// thay đổi chậm. Một lần nhấn được tính khi giá trị trung bình vượt mức nền
// trong PROX_CONFIRM mẫu liên tiếp, và giá trị phải giảm xuống lại trước khi
// có thể tính lần nhấn tiếp theo.
bool detectPress(int raw, unsigned long now) {
  if (now - proxSampleAt < PROX_SAMPLE_MS) return false;
  proxSampleAt = now;

  int n = constrain(PROX_AVG, 1, 8);
  for (int i = min(proxHistN, n - 1); i > 0; i--) proxHist[i] = proxHist[i - 1];
  proxHist[0] = raw;
  if (proxHistN < n) proxHistN++;
  float sum = 0;
  for (int i = 0; i < proxHistN; i++) sum += proxHist[i];
  float avg = sum / proxHistN;

  if (proxBase < 0) { proxBase = avg; return false; }

  proxDiff = avg - proxBase;
  bool fired = false;

  if (proxDiff < PROX_DELTA * 0.5f) proxArmed = true;
  proxAbove = (proxDiff >= PROX_DELTA) ? proxAbove + 1 : 0;

  if (proxArmed && proxAbove >= PROX_CONFIRM && now - proxLastAt > COOLDOWN_MS) {
    proxArmed  = false;
    proxLastAt = now;
    fired = true;
  }

  if (avg < proxBase) proxBase = avg;
  else proxBase += PROX_BASE_SPEED * proxDiff;
  return fired;
}

// Nút bên trái: trả về true một lần cho mỗi cú rung/đập (jolt).
// Hàm lấy tổng gia tốc trừ đi trọng lực, nhờ đó khi đứng yên nút đọc được
// xấp xỉ 0 và khi nhấn sẽ tạo ra một đỉnh xung. Giá trị phải ổn định
// trở lại trước khi tính cú đập tiếp theo.
bool detectJolt(unsigned long now) {
  if (!hasImu) return false;
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
-   float mag = sqrtf(accel.acceleration.x
accel.acceleration.x +
-                     accel.acceleration.y
accel.acceleration.y +
-                     accel.acceleration.z
accel.acceleration.z);
  joltValue = fabsf(mag - 9.81f);

  if (joltValue < JOLT_RESET) joltArmed = true;
  if (joltArmed && joltValue > JOLT_THRESHOLD && now - joltLastAt > COOLDOWN_MS) {
    joltArmed = false;
    joltLastAt = now;
    return true;
  }
  return false;
}

// Phản hồi khi bắt đầu trận đấu và khi kết thúc mỗi điểm.
// Sau một hiệp tích điểm, nếu có đèn mắt sáng sẽ bắt đầu hiệp buff (kích thước
// và màu theo số đèn sáng); nếu không có đèn nào sáng sẽ bắt đầu hiệp tích điểm mới.
// Sau hiệp buff, mọi thứ được đặt lại. Sự kiện ghi điểm thứ hai xuất hiện ngay
// sau sự kiện đầu tiên sẽ bị bỏ qua.
void handleMatchStartAndPointEnd(unsigned long now) {
  if (matchStart) {
    matchStart = false;
    startRound();
    Serial.println("=== MATCH START: paddle normal, new earn round");
  }

  if (pointEnded) {
    pointEnded = false;

    if (now - lastPointAt < POINT_DEBOUNCE_MS) {
      Serial.println("(second point event ignored)");
      return;
    }
    lastPointAt = now;

    if (!buffRound) {

      int n = ledsLit();
      if (n > 0) {
        sendSize(SIZE_FOR_LEDS[n]);
        sizeDirty = false;
        sendColor(n);
        buffRound = true;
        resetSent = false;
        playStarted = false;
        buffRoundStartedAt = now;
        Serial.print("=== EARN ROUND END: ");
        Serial.print(n);
        Serial.print(" pair(s) lit -> BUFF round, paddle ");
        Serial.print(SIZE_FOR_LEDS[n]);
        Serial.println("%");
      } else {
        startRound();
        Serial.println("=== EARN ROUND END: 0 lit -> new earn round");
      }
    } else {

      startRound();
      Serial.println("=== BUFF ROUND END: reset, new earn round");
    }
  }
}

// Trong hiệp buff, ngay khi pha bóng thực sự bắt đầu, gửi lại kích thước
// thanh đỡ bình thường để áp dụng từ lượt giao bóng tiếp theo.
void handleBuffResetQueue(unsigned long now) {
  if (buffRound && !resetSent &&
      (playStarted || now - buffRoundStartedAt > BUFF_RESET_FALLBACK_MS)) {
    sendSize(SIZE_FOR_LEDS[0]);
    resetSent = true;
    Serial.println("Buff round under way: next serve goes back to normal size");
  }
}

// Khi cả trận đấu kết thúc, đặt lại mọi thứ về trạng thái ban đầu.
void handleMatchEnd() {
  if (matchEnded) {
    matchEnded = false;
    startRound();
    Serial.println("=== MATCH END: paddle back to normal");
  }
}

// Nút bên trái: một cú đập (jolt) sẽ di chuyển thanh đỡ LÊN (UP) một bước.
// Trong hiệp tích điểm, nó cũng bật/tắt đèn mắt của chuột chũi bên trái.
void handleLeftButton(unsigned long now) {
  if (detectJolt(now)) {
    startStep(UP, now);
    if (!buffRound) {
      gyroLedOn = !gyroLedOn;
      sizeDirty = true;  ledsChangedAt = now;
      Serial.println("JOLT -> UP, LEDs 27 toggled");
    } else {
      Serial.println("JOLT -> UP (buff round: LEDs locked)");
    }
  }
}

// Nút bên phải: một lần nhấn sẽ di chuyển thanh đỡ XUỐNG (DOWN) một bước.
// Trong hiệp tích điểm, nó cũng bật/tắt đèn mắt của chuột chũi bên phải.
void handleRightButton(unsigned long now) {
  if (!hasProx) return;
  handRaw = readProximityRaw();
  if (detectPress(handRaw, now)) {
    startStep(DOWN, now);
    if (!buffRound) {
      proxLedOn = !proxLedOn;
      sizeDirty = true;  ledsChangedAt = now;
      Serial.println("PRESS -> DOWN, LEDs 33 toggled");
    } else {
      Serial.println("PRESS -> DOWN (buff round: LEDs locked)");
    }
  }
}

// Gửi kích thước thanh đỡ mới sau khi các đèn mắt ngừng thay đổi trong khoảng
// SIZE_SETTLE_MS, giúp các thao tác bật/tắt liên tục chỉ gửi lên game một lần.
void handleSizeSend(unsigned long now) {
  if (sizeDirty && !buffRound && now - ledsChangedAt >= SIZE_SETTLE_MS) {
    sendSize(SIZE_FOR_LEDS[ledsLit()]);
    sizeDirty = false;
  }
}

// Hiển thị trạng thái hai cặp mắt chuột chũi ra các chân LED.
void updateLeds() {
  digitalWrite(GYRO_LED_PIN, gyroLedOn ? HIGH : LOW);
  digitalWrite(PROX_LED_PIN, proxLedOn ? HIGH : LOW);
}

// Gửi lệnh điều khiển thanh đỡ lên game: UP hoặc DOWN khi bước di chuyển
// đang chạy, ngược lại là STOP. Trả về lệnh đã được gửi.
int sendMovement(unsigned long now) {
  int command = (now < stepEnd) ? stepDirection : NEUTRAL;
  controller.sendControl(command);
  return command;
}

// Chạy một lần khi khởi động: cấp nguồn cổng I2C, thiết lập các đèn LED và
// cả hai cảm biến, sau đó cài đặt số hiệu, tên tay cầm, âm thanh, sự kiện
// và khởi động Bluetooth.
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

  pinMode(GYRO_LED_PIN, OUTPUT);
  pinMode(PROX_LED_PIN, OUTPUT);
  digitalWrite(GYRO_LED_PIN, LOW);
  digitalWrite(PROX_LED_PIN, LOW);

  Wire.begin();

  hasProx = apds.begin();
  if (hasProx) {
    apds.enableProximitySensor(true);
    apds.setProxResolution(PROX_RES);
    apds.setProxMeasRate(PROX_RATE);
    apds.setLEDCurrent(PROX_LED_CURRENT);
    apds.setLEDPulses(PROX_LED_PULSES);
  } else {
    Serial.println("APDS9999 not found - check wiring");
  }

  hasImu = imu.begin_I2C();
  if (!hasImu) {
    Serial.println("LSM6DS3TR-C not found - check wiring");
  } else {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
  }

  controller.setControllerNumber(CONTROLLER_NUMBER);
  controller.setControllerName(CONTROLLER_NAME);
  controller.setHitSound(HIT_SOUND);
  controller.setScoreSound(SCORE_SOUND);
  controller.setWinSound(WIN_SOUND);
  startRound();
  controller.setStatusLED(LED_BUILTIN);
  controller.onGameEvent(gameEvent);
  controller.begin();
}

// Chạy lặp lại liên tục: xử lý sự kiện game, đọc cả hai nút, gửi kích thước
// và hướng di chuyển thanh đỡ, cập nhật đèn LED và in thông tin gỡ lỗi (debug).
void loop() {
  controller.update();
  unsigned long now = millis();

  handleMatchStartAndPointEnd(now);
  handleBuffResetQueue(now);
  handleMatchEnd();
  handleLeftButton(now);
  handleRightButton(now);
  handleSizeSend(now);
  updateLeds();
  int command = sendMovement(now);
  printDebug(now, command);

  delay(5);
}

// In giá trị cảm biến trực tiếp và trạng thái mỗi 200 ms ra Serial Monitor,
// dùng để tinh chỉnh các ngưỡng (threshold).
void printDebug(unsigned long now, int command) {
  static unsigned long lastPrint = 0;
  if (now - lastPrint <= 200) return;
  lastPrint = now;
  Serial.print("jolt: ");
  Serial.print(joltValue, 2);
  Serial.print("  | hand raw: ");
  Serial.print(handRaw);
  Serial.print(" (base ");
  Serial.print(proxBase, 1);
  Serial.print(" diff ");
  Serial.print(proxDiff, 1);
  Serial.print(")");
  Serial.print("  | LEDs 27: ");
  Serial.print(gyroLedOn ? "ON" : "off");
  Serial.print("  33: ");
  Serial.print(proxLedOn ? "ON" : "off");
  Serial.print(buffRound ? "  [BUFF ROUND]" : "  [earn]");
  Serial.print("  | cmd: ");
  Serial.println(command == UP ? "UP" : command == DOWN ? "DOWN" : "STOP");
}