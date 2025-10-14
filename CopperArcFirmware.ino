#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Encoder.h>

//============================================================
// Configuration & Constants
//============================================================

#define ENABLE_DOUBLE_PULSE 1
#define ENABLE_SPLASH 1

static const uint8_t PIN_OLED_SDA     = 2;  // D2 SDA
static const uint8_t PIN_OLED_SCL     = 3;  // D3 SCL
static const uint8_t PIN_ENCODER_A    = 7;  // D7
static const uint8_t PIN_ENCODER_B    = 8;  // D8
static const uint8_t PIN_ENCODER_PUSH = 4;  // D4 active low
static const uint8_t PIN_CONFIRM      = 5;  // D5 active low
static const uint8_t PIN_BACK         = 6;  // D6 active low
static const uint8_t PIN_WELD_TRIGGER = 10; // D10 gate driver

static const uint16_t DEFAULT_PULSE_US = 800;
static const uint16_t DEFAULT_P1_US    = 400;
static const uint16_t DEFAULT_GAP_US   = 600;
static const uint16_t DEFAULT_P2_US    = 400;
static const uint8_t DEFAULT_STEP_IDX  = 1; // 10us
static const uint8_t DEFAULT_CONTRAST  = 200;

static const uint16_t PULSE_MIN_US = 10;
static const uint16_t PULSE_MAX_US = 5000;
static const uint16_t GAP_MAX_US   = 5000;

static const uint8_t STEP_COUNT = 3;
static const uint16_t STEP_VALUES[STEP_COUNT] = {1, 10, 100};

static const uint32_t UI_FRAME_MS = 50; // ~20 Hz
static const uint16_t TOAST_MAX_MS = 1200;

static const uint8_t EEPROM_VERSION = 0x02;
static const uint16_t EEPROM_ADDR_SETTINGS = 0;
static const uint16_t EEPROM_ADDR_PROFILES = 64; // reserve space after settings

static const uint8_t PROFILE_COUNT = 5;
static const char PROFILE_NAMES[PROFILE_COUNT][11] PROGMEM = {
  "18650-Cu",
  "Pouch-Cu",
  "Bus-Bar",
  "Test-1",
  "Thin-Ni"
};

//============================================================
// Data Structures
//============================================================

struct Settings {
  bool     doublePulse;   // false = single, true = double
  uint16_t pulse_us;      // single pulse
  uint16_t p1_us;         // double pulse P1
  uint16_t gap_us;        // double pulse Gap
  uint16_t p2_us;         // double pulse P2
  uint8_t  stepIdx;       // 0=1us, 1=10us, 2=100us
  uint8_t  contrast;      // 0..255
  bool     invert;        // OLED invert
  bool     splashOn;      // show splash at boot
  bool     spinnerOn;     // show spinner during fire
  uint8_t  activeProfile; // 0..4
  uint16_t crc;           // checksum
};

//============================================================
// Globals
//============================================================

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_OLED_SCL, PIN_OLED_SDA);
Encoder encoder(PIN_ENCODER_A, PIN_ENCODER_B);

Settings gSettings;

static const uint8_t ICON_SPINNER_FRAME_COUNT = 4;
static const uint8_t ICON_SPINNER[ICON_SPINNER_FRAME_COUNT][8] PROGMEM = {
  {0x18,0x3C,0x7E,0xFF,0xFF,0x7E,0x3C,0x18},
  {0x18,0x3C,0x5A,0x99,0x99,0x5A,0x3C,0x18},
  {0x10,0x38,0x7C,0xFE,0x7C,0x38,0x10,0x00},
  {0x00,0x18,0x3C,0x7E,0x3C,0x18,0x00,0x00}
};

static const uint8_t ICON_EEPROM[8] PROGMEM = {0x7E,0x42,0x42,0x42,0x42,0x42,0x42,0x7E};

static const uint8_t LOGO_WIDTH = 64;
static const uint8_t LOGO_HEIGHT = 16;
static const uint8_t LOGO_BITMAP[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x1F,0xC0,0x3F,0xE0,0x78,0xF0,0x70,0x70,
  0x60,0x30,0x60,0x30,0x60,0x30,0x60,0x30,
  0x70,0x70,0x78,0xF0,0x3F,0xE0,0x1F,0xC0,
  0x00,0x00,0x7F,0xFC,0xFF,0xFE,0xE0,0x06,
  0xC0,0x03,0xC0,0x03,0xC0,0x03,0xE0,0x07,
  0xFF,0xFE,0x7F,0xFC,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

struct ButtonState {
  bool lastStable;
  bool currentStable;
  bool lastReading;
  uint32_t lastChangeMs;
  bool pressedEdge;
};

ButtonState btnConfirm;
ButtonState btnBack;
ButtonState btnEncoder;

long encoderLast = 0;
int16_t encoderFrameDelta = 0;
uint32_t encoderLastFrameMs = 0;

uint32_t lastUiFrameMs = 0;
uint8_t spinnerFrame = 0;
uint32_t spinnerLastMs = 0;
bool firingActive = false;

uint32_t toastExpireMs = 0;
char toastBuffer[22];

bool eepromFlash = false;
uint32_t eepromFlashExpireMs = 0;

bool defaultsLoaded = false;
bool i2cOk = false;

// Starfield
static const uint8_t STAR_COUNT = 18;
struct Star { int8_t x; int8_t y; int8_t speed; };
Star stars[STAR_COUNT];

// Menu state

enum ScreenId {
  SCREEN_HOME = 0,
  SCREEN_MENU,
  SCREEN_PULSE_SETUP,
  SCREEN_PROFILE_LOAD,
  SCREEN_PROFILE_SAVE,
  SCREEN_DISPLAY,
  SCREEN_TIMING,
  SCREEN_DIAGNOSTICS,
  SCREEN_ABOUT,
  SCREEN_DEMO_FIRE
};

ScreenId currentScreen = SCREEN_HOME;
uint8_t menuIndex = 0;
uint8_t menuScroll = 0;
uint8_t editIndex = 0;
bool editingValue = false;

//============================================================
// Utility Functions
//============================================================

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

uint16_t bounded(uint16_t value, uint16_t minVal, uint16_t maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

//============================================================
// Storage Functions
//============================================================

void fillDefaultSettings(Settings &s) {
  s.doublePulse = ENABLE_DOUBLE_PULSE ? true : false;
  s.pulse_us = DEFAULT_PULSE_US;
  s.p1_us = DEFAULT_P1_US;
  s.gap_us = DEFAULT_GAP_US;
  s.p2_us = DEFAULT_P2_US;
  s.stepIdx = DEFAULT_STEP_IDX;
  s.contrast = DEFAULT_CONTRAST;
  s.invert = false;
  s.splashOn = true;
  s.spinnerOn = true;
  s.activeProfile = 0;
  s.crc = 0;
}

void computeAndStoreCrc(Settings &s) {
  s.crc = 0;
  s.crc = crc16(reinterpret_cast<const uint8_t*>(&s), sizeof(Settings));
}

bool readSettingsFromEeprom(uint16_t addr, Settings &dest) {
  EEPROM.get(addr, dest);
  uint16_t savedCrc = dest.crc;
  dest.crc = 0;
  uint16_t calc = crc16(reinterpret_cast<const uint8_t*>(&dest), sizeof(Settings));
  dest.crc = savedCrc;
  return calc == savedCrc;
}

bool saveSettingsToEeprom(uint16_t addr, Settings &src) {
  computeAndStoreCrc(src);
  EEPROM.put(addr, src);
  return true;
}

bool loadSettings(Settings &s) {
  Settings temp;
  EEPROM.get(EEPROM_ADDR_SETTINGS, temp);
  if (temp.activeProfile > PROFILE_COUNT - 1) {
    temp.activeProfile = 0;
  }
  uint8_t version = EEPROM.read(EEPROM_ADDR_SETTINGS + sizeof(Settings));
  if (version != EEPROM_VERSION) {
    fillDefaultSettings(s);
    defaultsLoaded = true;
    return false;
  }
  uint16_t savedCrc = temp.crc;
  temp.crc = 0;
  uint16_t calc = crc16(reinterpret_cast<const uint8_t*>(&temp), sizeof(Settings));
  if (calc != savedCrc) {
    fillDefaultSettings(s);
    defaultsLoaded = true;
    return false;
  }
  temp.crc = savedCrc;
  if (temp.stepIdx >= STEP_COUNT) temp.stepIdx = DEFAULT_STEP_IDX;
  s = temp;
  return true;
}

bool saveSettings(const Settings &s) {
  Settings temp = s;
  computeAndStoreCrc(temp);
  EEPROM.put(EEPROM_ADDR_SETTINGS, temp);
  EEPROM.write(EEPROM_ADDR_SETTINGS + sizeof(Settings), EEPROM_VERSION);
  eepromFlash = true;
  eepromFlashExpireMs = millis() + 1200;
  return true;
}

bool loadProfile(uint8_t idx, Settings &s) {
  if (idx >= PROFILE_COUNT) return false;
  Settings temp;
  uint16_t addr = EEPROM_ADDR_PROFILES + idx * sizeof(Settings);
  EEPROM.get(addr, temp);
  uint16_t savedCrc = temp.crc;
  temp.crc = 0;
  uint16_t calc = crc16(reinterpret_cast<const uint8_t*>(&temp), sizeof(Settings));
  if (calc != savedCrc) {
    return false;
  }
  temp.crc = savedCrc;
  if (temp.stepIdx >= STEP_COUNT) temp.stepIdx = DEFAULT_STEP_IDX;
  s = temp;
  return true;
}

bool saveProfile(uint8_t idx, const Settings &s) {
  if (idx >= PROFILE_COUNT) return false;
  Settings temp = s;
  computeAndStoreCrc(temp);
  uint16_t addr = EEPROM_ADDR_PROFILES + idx * sizeof(Settings);
  EEPROM.put(addr, temp);
  eepromFlash = true;
  eepromFlashExpireMs = millis() + 1200;
  return true;
}

//============================================================
// Input Handling
//============================================================

void updateButton(ButtonState &btn, uint8_t pin) {
  bool reading = digitalRead(pin) == LOW;
  uint32_t now = millis();
  if (reading != btn.lastReading) {
    btn.lastChangeMs = now;
    btn.lastReading = reading;
  }
  if ((now - btn.lastChangeMs) > 20) {
    if (reading != btn.currentStable) {
      btn.lastStable = btn.currentStable;
      btn.currentStable = reading;
      if (btn.currentStable && !btn.lastStable) {
        btn.pressedEdge = true;
      }
    }
  }
}

void inputInit() {
  pinMode(PIN_ENCODER_PUSH, INPUT_PULLUP);
  pinMode(PIN_CONFIRM, INPUT_PULLUP);
  pinMode(PIN_BACK, INPUT_PULLUP);
  bool confirmState = digitalRead(PIN_CONFIRM) == LOW;
  bool backState = digitalRead(PIN_BACK) == LOW;
  bool pushState = digitalRead(PIN_ENCODER_PUSH) == LOW;
  btnConfirm = {confirmState, confirmState, confirmState, millis(), false};
  btnBack = {backState, backState, backState, millis(), false};
  btnEncoder = {pushState, pushState, pushState, millis(), false};
  encoder.write(0);
  encoderLast = 0;
  encoderFrameDelta = 0;
  encoderLastFrameMs = millis();
}

bool buttonPressed(uint8_t pin) {
  ButtonState *bs = nullptr;
  if (pin == PIN_CONFIRM) bs = &btnConfirm;
  else if (pin == PIN_BACK) bs = &btnBack;
  else if (pin == PIN_ENCODER_PUSH) bs = &btnEncoder;
  if (!bs) return false;
  if (bs->pressedEdge) {
    bs->pressedEdge = false;
    return true;
  }
  return false;
}

bool buttonHeld(uint8_t pin, uint16_t ms) {
  ButtonState *bs = nullptr;
  if (pin == PIN_CONFIRM) bs = &btnConfirm;
  else if (pin == PIN_BACK) bs = &btnBack;
  else if (pin == PIN_ENCODER_PUSH) bs = &btnEncoder;
  if (!bs) return false;
  if (bs->currentStable && (millis() - bs->lastChangeMs) > ms) {
    return true;
  }
  return false;
}

int16_t encoderDelta() {
  long now = encoder.read();
  int16_t delta = (int16_t)(now - encoderLast);
  if (delta != 0) {
    encoderLast = now;
  }
  encoderFrameDelta += delta;
  uint32_t nowMs = millis();
  if (nowMs - encoderLastFrameMs >= UI_FRAME_MS) {
    int16_t frame = encoderFrameDelta;
    encoderFrameDelta = 0;
    encoderLastFrameMs = nowMs;
    if (frame == 0) return 0;
    int16_t accel = abs(frame) >= 4 ? 2 : 1;
    return frame * accel;
  }
  return 0;
}

//============================================================
// UI Helpers
//============================================================

void uiSetTheme(const Settings &s) {
  u8g2.setContrast(s.contrast);
  u8g2.setFlipMode(0);
  u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.sendF("c", s.invert ? 0xA7 : 0xA6);
}

void uiToast(const __FlashStringHelper* s, uint16_t ms) {
  strncpy_P(toastBuffer, (PGM_P)s, sizeof(toastBuffer)-1);
  toastBuffer[sizeof(toastBuffer)-1] = '\0';
  toastExpireMs = millis() + ms;
}

void uiToast(const char *s, uint16_t ms) {
  strncpy(toastBuffer, s, sizeof(toastBuffer)-1);
  toastBuffer[sizeof(toastBuffer)-1] = '\0';
  toastExpireMs = millis() + ms;
}

void drawStatusIcons() {
  if (firingActive && gSettings.spinnerOn) {
    uint8_t frame = spinnerFrame % ICON_SPINNER_FRAME_COUNT;
    u8g2.drawXBMP(112, 0, 8, 8, ICON_SPINNER[frame]);
  }
  if (eepromFlash && millis() < eepromFlashExpireMs) {
    u8g2.drawXBMP(102, 0, 8, 8, ICON_EEPROM);
  }
}

void drawHintBar() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 63, "ENC=adj PUSH=step CONFIRM=fire BACK=menu");
}

void drawToast() {
  if (toastExpireMs && millis() < toastExpireMs) {
    u8g2.setFont(u8g2_font_5x8_tf);
    uint8_t w = u8g2.getStrWidth(toastBuffer);
    int16_t x = (128 - w) / 2;
    u8g2.drawRBox(x - 2, 48, w + 4, 12, 2);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x, 57, toastBuffer);
    u8g2.setDrawColor(1);
  }
}

void resetToast() {
  toastExpireMs = 0;
}

void drawStarfieldFrame(uint16_t frame) {
  for (uint8_t i = 0; i < STAR_COUNT; ++i) {
    Star &s = stars[i];
    s.x -= s.speed;
    if (s.x < 0) {
      s.x = 120 + random(8);
      s.y = random(64);
      s.speed = 1 + (random(3));
    }
    u8g2.drawPixel(s.x, s.y);
  }
  u8g2.drawHLine(0, 12, 128);
  if ((frame / 3) % 2 == 0) {
    u8g2.drawHLine(0, 44, 128);
  }
}

void uiSplashAnimated() {
#if ENABLE_SPLASH
  if (!gSettings.splashOn) return;
  uint32_t start = millis();
  for (uint8_t i = 0; i < STAR_COUNT; ++i) {
    stars[i].x = random(128);
    stars[i].y = random(64);
    stars[i].speed = 1 + random(3);
  }
  while (millis() - start < 1800) {
    u8g2.firstPage();
    do {
      drawStarfieldFrame((millis() - start) / 50);
      u8g2.drawXBMP(32, 18, LOGO_WIDTH, LOGO_HEIGHT, LOGO_BITMAP);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(34, 38, "Spot Welder v1.0");
      uint16_t elapsed = millis() - start;
      uint8_t progress = (elapsed * 100) / 1800;
      uint8_t barWidth = map(progress, 0, 100, 10, 108);
      u8g2.drawFrame(10, 46, 108, 10);
      u8g2.drawBox(12, 48, barWidth - 4, 6);
    } while (u8g2.nextPage());
    delay(30);
  }
#endif
}

void uiInit() {
  u8g2.begin();
  u8g2.setI2CAddress(0x3C<<1);
  uiSetTheme(gSettings);
  if (gSettings.invert) {
    u8g2.setDrawColor(1);
  }
}

void uiStepTag() {
  char buf[16];
  snprintf_P(buf, sizeof(buf), PSTR("STEP %dus"), STEP_VALUES[gSettings.stepIdx]);
  uiToast(buf, 800);
}

//============================================================
// Rendering Screens
//============================================================

void renderHomePulseSingle() {
  char buf[16];
  snprintf_P(buf, sizeof(buf), PSTR("%4u us"), gSettings.pulse_us);
  u8g2.setFont(u8g2_font_logisoso18_tf);
  uint8_t w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w)/2, 40, buf);
}

void renderHomePulseDouble() {
  char buf[24];
  u8g2.setFont(u8g2_font_6x12_tf);
  snprintf_P(buf, sizeof(buf), PSTR("P1 %u | G %u"), gSettings.p1_us, gSettings.gap_us);
  uint8_t w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w)/2, 34, buf);
  snprintf_P(buf, sizeof(buf), PSTR("P2 %u"), gSettings.p2_us);
  w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w)/2, 50, buf);
}

void uiDrawHome() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "COPPER ARC");
  drawStatusIcons();
  if (gSettings.doublePulse) {
    renderHomePulseDouble();
  } else {
    renderHomePulseSingle();
  }
  drawHintBar();
  drawToast();
}

const __FlashStringHelper* getProfileName(uint8_t idx) {
  static char name[11];
  strncpy_P(name, PROFILE_NAMES[idx], sizeof(name));
  name[sizeof(name)-1] = '\0';
  return (const __FlashStringHelper*)name;
}

void drawMenuList(const __FlashStringHelper* title, const __FlashStringHelper* const *items, uint8_t count) {
  u8g2.setFont(u8g2_font_6x12_tf);
  char titleBuf[20];
  strncpy_P(titleBuf, (PGM_P)title, sizeof(titleBuf) - 1);
  titleBuf[sizeof(titleBuf) - 1] = '\0';
  u8g2.drawStr(0, 10, titleBuf);
  drawStatusIcons();
  uint8_t visible = 4;
  if (menuIndex < menuScroll) menuScroll = menuIndex;
  if (menuIndex >= menuScroll + visible) menuScroll = menuIndex - visible + 1;
  for (uint8_t i = 0; i < visible && (i + menuScroll) < count; ++i) {
    uint8_t idx = i + menuScroll;
    int y = 22 + i * 12;
    if (idx == menuIndex) {
      u8g2.drawStr(0, y, ">" );
    }
    char label[16];
    strncpy_P(label, (PGM_P)items[idx], sizeof(label)-1);
    label[sizeof(label)-1] = '\0';
    u8g2.drawStr(12, y, label);
  }
  drawHintBar();
  drawToast();
}

void drawPulseSetup() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Pulse Setup");
  drawStatusIcons();
  const char* labels[4] = {"Mode", "Pulse", "Gap", "P2"};
  for (uint8_t i = 0; i < 4; ++i) {
    int y = 22 + i * 12;
    if (editIndex == i) {
      u8g2.drawStr(0, y, editingValue ? "*" : ">");
    }
    u8g2.drawStr(10, y, labels[i]);
    char buf[16];
    switch (i) {
      case 0:
        strcpy(buf, gSettings.doublePulse ? "Double" : "Single");
        break;
      case 1:
        if (gSettings.doublePulse) {
          snprintf(buf, sizeof(buf), "%u", gSettings.p1_us);
        } else {
          snprintf(buf, sizeof(buf), "%u", gSettings.pulse_us);
        }
        break;
      case 2:
        snprintf(buf, sizeof(buf), "%u", gSettings.gap_us);
        break;
      case 3:
        snprintf(buf, sizeof(buf), "%u", gSettings.p2_us);
        break;
    }
    u8g2.drawStr(80, y, buf);
  }
  drawHintBar();
  drawToast();
}

void drawDisplayMenu() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Display & UI");
  drawStatusIcons();
  const char* labels[4] = {"Invert", "Contrast", "Splash", "Spinner"};
  for (uint8_t i = 0; i < 4; ++i) {
    int y = 22 + i * 12;
    if (editIndex == i) {
      u8g2.drawStr(0, y, editingValue ? "*" : ">");
    }
    u8g2.drawStr(10, y, labels[i]);
    char buf[16];
    switch (i) {
      case 0: strcpy(buf, gSettings.invert ? "On" : "Off"); break;
      case 1: snprintf(buf, sizeof(buf), "%u", gSettings.contrast); break;
      case 2: strcpy(buf, gSettings.splashOn ? "On" : "Off"); break;
      case 3: strcpy(buf, gSettings.spinnerOn ? "On" : "Off"); break;
    }
    u8g2.drawStr(84, y, buf);
  }
  drawHintBar();
  drawToast();
}

void drawTimingMenu() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Timing & I/O");
  drawStatusIcons();
  const char* labels[4] = {"Trigger Pin", "Pre-arm", "Demo ms", "Blink D10"};
  for (uint8_t i = 0; i < 4; ++i) {
    int y = 22 + i * 12;
    if (editIndex == i) {
      u8g2.drawStr(0, y, editingValue ? "*" : ">");
    }
    u8g2.drawStr(10, y, labels[i]);
    char buf[16];
    switch (i) {
      case 0:
        snprintf(buf, sizeof(buf), "D%u", PIN_WELD_TRIGGER);
        break;
      case 1:
        snprintf(buf, sizeof(buf), "%u ms", preArmDelayMs);
        break;
      case 2:
        snprintf(buf, sizeof(buf), "%u ms", demoPulseMs);
        break;
      case 3:
        strcpy(buf, "Run");
        break;
    }
    u8g2.drawStr(88, y, buf);
  }
  drawHintBar();
  drawToast();
}

void drawProfilesMenu(const __FlashStringHelper* title) {
  u8g2.setFont(u8g2_font_6x12_tf);
  char titleBuf[20];
  strncpy_P(titleBuf, (PGM_P)title, sizeof(titleBuf) - 1);
  titleBuf[sizeof(titleBuf) - 1] = '\0';
  u8g2.drawStr(0, 10, titleBuf);
  drawStatusIcons();
  const uint8_t visible = 4;
  if (menuIndex < menuScroll) menuScroll = menuIndex;
  if (menuIndex >= menuScroll + visible) menuScroll = menuIndex - visible + 1;
  for (uint8_t i = 0; i < visible && (i + menuScroll) < PROFILE_COUNT; ++i) {
    uint8_t idx = i + menuScroll;
    int y = 22 + i * 12;
    if (menuIndex == idx) {
      u8g2.drawStr(0, y, ">");
    }
    char name[11];
    strncpy_P(name, PROFILE_NAMES[idx], sizeof(name));
    name[sizeof(name)-1] = '\0';
    u8g2.drawStr(12, y, name);
    if (gSettings.activeProfile == idx) {
      u8g2.drawStr(100, y, "*");
    }
  }
  drawHintBar();
  drawToast();
}

void drawDiagnostics() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 10, "Diagnostics");
  drawStatusIcons();
  char buf[24];
  snprintf_P(buf, sizeof(buf), PSTR("Enc: %ld"), encoder.read());
  u8g2.drawStr(0, 24, buf);
  snprintf_P(buf, sizeof(buf), PSTR("BTN C:%d B:%d P:%d"), btnConfirm.currentStable, btnBack.currentStable, btnEncoder.currentStable);
  u8g2.drawStr(0, 36, buf);
  snprintf_P(buf, sizeof(buf), PSTR("freeRAM: %d"), freeRam());
  u8g2.drawStr(0, 48, buf);
  snprintf_P(buf, sizeof(buf), PSTR("I2C 0x3C %s"), i2cOk ? "OK" : "FAIL");
  u8g2.drawStr(0, 60, buf);
  drawToast();
}

void drawAbout() {
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(10, 12, "COPPER ARC");
  u8g2.drawStr(10, 24, "Spot Welder v1.0");
  u8g2.drawStr(10, 36, "Build: " __DATE__);
  u8g2.drawStr(10, 48, "Author: Firmware Bot");
  u8g2.drawStr(10, 60, "Thanks for welding!");
  drawStatusIcons();
  drawToast();
}

void uiDrawMenu() {
  static const __FlashStringHelper* const mainItems[] = {
    F("Pulse Setup"),
    F("Load Profile"),
    F("Save Profile"),
    F("Display & UI"),
    F("Timing & I/O"),
    F("Diagnostics"),
    F("About")
  };
  drawMenuList(F("Main Menu"), mainItems, sizeof(mainItems)/sizeof(mainItems[0]));
}

void uiDrawScreen() {
  u8g2.firstPage();
  do {
    switch (currentScreen) {
      case SCREEN_HOME: uiDrawHome(); break;
      case SCREEN_MENU: uiDrawMenu(); break;
      case SCREEN_PULSE_SETUP: drawPulseSetup(); break;
      case SCREEN_PROFILE_LOAD: drawProfilesMenu(F("Load Profile")); break;
      case SCREEN_PROFILE_SAVE: drawProfilesMenu(F("Save Profile")); break;
      case SCREEN_DISPLAY: drawDisplayMenu(); break;
      case SCREEN_TIMING: drawTimingMenu(); break;
      case SCREEN_DIAGNOSTICS: drawDiagnostics(); break;
      case SCREEN_ABOUT: drawAbout(); break;
      case SCREEN_DEMO_FIRE:
        u8g2.drawStr(10, 32, "Blink demo in progress...");
        drawStatusIcons();
        break;
    }
  } while (u8g2.nextPage());
}

//============================================================
// Firing Control
//============================================================

uint16_t preArmDelayMs = 0;
uint16_t demoPulseMs = 10;

void triggerPulseMicroseconds(uint16_t duration) {
  digitalWrite(PIN_WELD_TRIGGER, HIGH);
  delayMicroseconds(duration);
  digitalWrite(PIN_WELD_TRIGGER, LOW);
}

void fireSingle(uint16_t us) {
  if (preArmDelayMs) delay(preArmDelayMs);
  triggerPulseMicroseconds(us);
}

void fireDouble(uint16_t p1, uint16_t gap, uint16_t p2) {
  if (preArmDelayMs) delay(preArmDelayMs);
  triggerPulseMicroseconds(p1);
  if (gap) delayMicroseconds(gap);
  if (p2) triggerPulseMicroseconds(p2);
}

void fireCurrent() {
  firingActive = true;
  spinnerLastMs = millis();
  uiDrawScreen();
  if (!gSettings.doublePulse) {
    fireSingle(gSettings.pulse_us);
  } else {
    fireDouble(gSettings.p1_us, gSettings.gap_us, gSettings.p2_us);
  }
  firingActive = false;
}

void runDemoFire() {
  firingActive = true;
  uiDrawScreen();
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(PIN_WELD_TRIGGER, HIGH);
    delay(demoPulseMs);
    digitalWrite(PIN_WELD_TRIGGER, LOW);
    delay(120);
  }
  firingActive = false;
}

//============================================================
// UI State Transitions
//============================================================

void changeStep() {
  gSettings.stepIdx = (gSettings.stepIdx + 1) % STEP_COUNT;
  uiStepTag();
  saveSettings(gSettings);
}

void applyEncoderToValue(uint16_t &value, uint16_t minv, uint16_t maxv, int16_t delta) {
  if (delta == 0) return;
  int32_t step = STEP_VALUES[gSettings.stepIdx];
  int32_t newValue = (int32_t)value + (int32_t)delta * step;
  if (newValue < (int32_t)minv) newValue = minv;
  if (newValue > (int32_t)maxv) newValue = maxv;
  value = (uint16_t)newValue;
}

void handlePulseSetupEncoder(int16_t delta) {
  if (!editingValue) {
    editIndex = (editIndex + (delta > 0 ? 1 : -1) + 4) % 4;
    return;
  }
  switch (editIndex) {
    case 0:
      if (delta != 0) {
#if ENABLE_DOUBLE_PULSE
        gSettings.doublePulse = delta > 0;
#else
        gSettings.doublePulse = false;
#endif
      }
      break;
    case 1:
      if (gSettings.doublePulse) {
        applyEncoderToValue(gSettings.p1_us, PULSE_MIN_US, PULSE_MAX_US, delta);
      } else {
        applyEncoderToValue(gSettings.pulse_us, PULSE_MIN_US, PULSE_MAX_US, delta);
      }
      break;
    case 2:
      applyEncoderToValue(gSettings.gap_us, 0, GAP_MAX_US, delta);
      break;
    case 3:
      applyEncoderToValue(gSettings.p2_us, 0, GAP_MAX_US, delta);
      break;
  }
}

void handleDisplayEncoder(int16_t delta) {
  if (!editingValue) {
    editIndex = (editIndex + (delta > 0 ? 1 : -1) + 4) % 4;
    return;
  }
  switch (editIndex) {
    case 0:
      if (delta != 0) gSettings.invert = !gSettings.invert;
      break;
    case 1:
      if (delta != 0) {
        int16_t val = (int16_t)gSettings.contrast + delta;
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        gSettings.contrast = (uint8_t)val;
        uiSetTheme(gSettings);
      }
      break;
    case 2:
      if (delta != 0) gSettings.splashOn = !gSettings.splashOn;
      break;
    case 3:
      if (delta != 0) gSettings.spinnerOn = !gSettings.spinnerOn;
      break;
  }
}

void handleTimingEncoder(int16_t delta) {
  if (!editingValue) {
    editIndex = (editIndex + (delta > 0 ? 1 : -1) + 4) % 4;
    return;
  }
  switch (editIndex) {
    case 1: {
      int16_t val = (int16_t)preArmDelayMs + delta;
      if (val < 0) val = 0;
      if (val > 500) val = 500;
      preArmDelayMs = (uint16_t)val;
      break;
    }
    case 2: {
      int16_t val = (int16_t)demoPulseMs + delta;
      if (val < 5) val = 5;
      if (val > 20) val = 20;
      demoPulseMs = (uint16_t)val;
      break;
    }
    default:
      break;
  }
}

void handleProfileSelection(int16_t delta) {
  int8_t newIndex = menuIndex + (delta > 0 ? 1 : -1);
  if (newIndex < 0) newIndex = PROFILE_COUNT - 1;
  if (newIndex >= PROFILE_COUNT) newIndex = 0;
  menuIndex = newIndex;
}

void handleMainMenuEncoder(int16_t delta) {
  static const uint8_t mainCount = 7;
  if (delta == 0) return;
  int8_t newIndex = menuIndex + (delta > 0 ? 1 : -1);
  if (newIndex < 0) newIndex = mainCount - 1;
  if (newIndex >= mainCount) newIndex = 0;
  menuIndex = newIndex;
}

void enterScreen(ScreenId s) {
  currentScreen = s;
  menuIndex = 0;
  menuScroll = 0;
  editIndex = 0;
  editingValue = false;
}

void saveAndToast() {
  saveSettings(gSettings);
  uiToast(F("Settings Saved"), 1000);
}

//============================================================
// Loop & Logic
//============================================================

void setup() {
  pinMode(PIN_WELD_TRIGGER, OUTPUT);
  digitalWrite(PIN_WELD_TRIGGER, LOW);

  inputInit();
  Wire.begin();
  Wire.beginTransmission(0x3C);
  i2cOk = (Wire.endTransmission() == 0);
  Serial.begin(115200);
  delay(20);

  fillDefaultSettings(gSettings);
  bool ok = loadSettings(gSettings);
  uiInit();
  if (defaultsLoaded) {
    uiToast(F("Defaults Loaded"), 1500);
  }
  if (!ok) {
    saveSettings(gSettings);
  }

#if ENABLE_SPLASH
  uiSplashAnimated();
#endif

  Serial.println(F("COPPER ARC Spot Welder"));
  Serial.print(F("I2C OLED @ 0x3C: "));
  Serial.println(i2cOk ? F("OK") : F("FAIL"));
  Serial.print(F("EEPROM status: ")); Serial.println(ok ? F("OK") : F("RESET"));
  Serial.print(F("free RAM: ")); Serial.println(freeRam());
}

void updateInputs() {
  updateButton(btnConfirm, PIN_CONFIRM);
  updateButton(btnBack, PIN_BACK);
  updateButton(btnEncoder, PIN_ENCODER_PUSH);
}

void menuSelect() {
  switch (menuIndex) {
    case 0: enterScreen(SCREEN_PULSE_SETUP); break;
    case 1: enterScreen(SCREEN_PROFILE_LOAD); break;
    case 2: enterScreen(SCREEN_PROFILE_SAVE); break;
    case 3: enterScreen(SCREEN_DISPLAY); break;
    case 4: enterScreen(SCREEN_TIMING); break;
    case 5: enterScreen(SCREEN_DIAGNOSTICS); break;
    case 6: enterScreen(SCREEN_ABOUT); break;
  }
}

void handleButtons() {
  if (buttonPressed(PIN_CONFIRM)) {
    fireCurrent();
    uiToast(F("Pulse Fired"), 800);
    return;
  }

  if (buttonPressed(PIN_BACK)) {
    if (editingValue) {
      editingValue = false;
      if (currentScreen == SCREEN_PULSE_SETUP || currentScreen == SCREEN_DISPLAY) {
        saveAndToast();
      } else {
        uiToast(F("Edit Cancel"), 600);
      }
      return;
    }
    if (currentScreen == SCREEN_HOME) {
      enterScreen(SCREEN_MENU);
    } else if (currentScreen == SCREEN_MENU) {
      enterScreen(SCREEN_HOME);
    } else {
      enterScreen(SCREEN_MENU);
    }
    return;
  }

  if (buttonPressed(PIN_ENCODER_PUSH)) {
    switch (currentScreen) {
      case SCREEN_HOME:
        changeStep();
        break;
      case SCREEN_MENU:
        menuSelect();
        break;
      case SCREEN_PULSE_SETUP:
      case SCREEN_DISPLAY:
        editingValue = !editingValue;
        if (editingValue) {
          uiToast(F("Edit"), 600);
        } else {
          saveAndToast();
        }
        break;
      case SCREEN_TIMING:
        if (editIndex == 3) {
          enterScreen(SCREEN_DEMO_FIRE);
          runDemoFire();
          uiToast(F("Demo Fire"), 900);
          enterScreen(SCREEN_TIMING);
        } else {
          editingValue = !editingValue;
          if (editingValue) {
            uiToast(F("Edit"), 600);
          } else {
            uiToast(F("Timing Set"), 600);
          }
        }
        break;
      case SCREEN_PROFILE_LOAD:
        gSettings.activeProfile = menuIndex;
        if (loadProfile(menuIndex, gSettings)) {
          gSettings.activeProfile = menuIndex;
          uiSetTheme(gSettings);
          uiToast(F("Profile Loaded"), 1000);
          saveSettings(gSettings);
        } else {
          uiToast(F("Empty Slot"), 900);
        }
        enterScreen(SCREEN_HOME);
        break;
      case SCREEN_PROFILE_SAVE:
        gSettings.activeProfile = menuIndex;
        saveProfile(menuIndex, gSettings);
        uiToast(F("Profile Saved"), 1000);
        saveSettings(gSettings);
        enterScreen(SCREEN_MENU);
        break;
      case SCREEN_DIAGNOSTICS:
      case SCREEN_ABOUT:
        // no action
        break;
      case SCREEN_DEMO_FIRE:
        break;
    }
  }
}

void handleEncoder(int16_t delta) {
  if (delta == 0) return;
  switch (currentScreen) {
    case SCREEN_HOME:
      if (gSettings.doublePulse) {
        applyEncoderToValue(gSettings.p1_us, PULSE_MIN_US, PULSE_MAX_US, delta);
      } else {
        applyEncoderToValue(gSettings.pulse_us, PULSE_MIN_US, PULSE_MAX_US, delta);
      }
      break;
    case SCREEN_MENU:
      handleMainMenuEncoder(delta);
      break;
    case SCREEN_PULSE_SETUP:
      handlePulseSetupEncoder(delta);
      break;
    case SCREEN_PROFILE_LOAD:
    case SCREEN_PROFILE_SAVE:
      handleProfileSelection(delta);
      break;
    case SCREEN_DISPLAY:
      handleDisplayEncoder(delta);
      break;
    case SCREEN_TIMING:
      handleTimingEncoder(delta);
      break;
    default:
      break;
  }
}

void loop() {
  updateInputs();
  handleButtons();
  int16_t delta = encoderDelta();
  handleEncoder(delta);

  uint32_t now = millis();
  if (now - lastUiFrameMs >= UI_FRAME_MS) {
    lastUiFrameMs = now;
    if (firingActive && now - spinnerLastMs > 120) {
      spinnerFrame = (spinnerFrame + 1) % ICON_SPINNER_FRAME_COUNT;
      spinnerLastMs = now;
    }
    uiDrawScreen();
  }
}

//============================================================
// Integration Notes
//============================================================
// Wiring: OLED SDA->D2, SCL->D3, ENCODER A->D7, B->D8, PUSH->D4, CONFIRM->D5, BACK->D6, Trigger->D10, GND common, OLED 3V3.
// Power: OLED powered from 3.3V LDO, logic from Pro Micro 5V; expect <30mA OLED, <20mA logic idle.
// TODO Hooks: add dead-man switch, electrode pressure sensor, temperature monitoring, pack undervoltage interlock.
