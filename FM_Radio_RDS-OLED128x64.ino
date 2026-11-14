/*
 * =====================================================================
 *  Radio FM z RDS:  Arduino Nano + RDA5807M + OLED 128x64 (SSD1306, I2C)
 * =====================================================================
 *  Na OLED-zie: częstotliwość, PS, RadioText, PI Code, PTY
 *  (dodatkowo: głośność, stereo, wskaźnik RDS i siła sygnału).
 *
 *  JAK DZIAŁA RDS
 *   - PS, PI, PTY pojawiają się od razu po odebraniu pierwszej poprawnej grupy.
 *   - RadioText przy pierwszym wczytaniu pokazuje się "na żywo" (znaki w miarę
 *     odbioru). Potem tekst jest TRZYMANY na ekranie i podmieniany dopiero, gdy
 *     nowa treść zostanie odebrana w całości (bez migania i bez mieszania
 *     starego z nowym). Zanik sygnału nie kasuje tekstu.
 *   - Wszystko kasuje się dopiero po zmianie stacji.
 *
 *  BIBLIOTEKI (Menedżer bibliotek Arduino IDE):
 *    - "U8g2" (olikraus)
 *  RDA5807M obsługiwany jest bezpośrednio przez Wire, a dekoder RDS
 *  (PS / RT / PI / PTY) jest zaimplementowany w tym szkicu.
 *
 *  PODŁĄCZENIE (wszystko na jednej magistrali I2C)
 *    Nano A4 (SDA) -> SDA OLED  i  SDA RDA5807M
 *    Nano A5 (SCL) -> SCL OLED  i  SCL RDA5807M
 *    RDA5807M: VCC = 3,3 V (układ 2,7–3,6 V!), GND = GND
 *    OLED:     VCC = 3,3 V (najlepiej, wtedy cała szyna I2C jest 3,3 V)
 *    Antena:   ok. 75 cm przewodu na pad ANT lub przewód słuchawek
 *    Audio:    wyjście słuchawkowe modułu (L/R/GND) -> słuchawki / wzmacniacz
 *
 *  PRZYCISKI (jeden koniec do pinu, drugi do GND, pull-up wewnętrzny)
 *    D8  - częstotliwość w górę : krótko = +0,05 MHz, długo = szukaj stacji w górę
 *    D9  - częstotliwość w dół  : krótko = -0,05 MHz, długo = szukaj stacji w dół
 *    D10  - głośność +
 *    D11  - głośność -
 *
 *  Ostatnia stacja i głośność są zapamiętywane w EEPROM.
 *
 *  UWAGA: jeśli masz OLED 1,3" na sterowniku SH1106, zamień linię z
 *  konstruktorem na:  U8G2_SH1106_128X64_NONAME_1_HW_I2C
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>

// ============================ KONFIGURACJA ============================
#define PIN_FREQ_UP      8
#define PIN_FREQ_DOWN    9
#define PIN_VOL_UP       10
#define PIN_VOL_DOWN     11

#define FREQ_BASE        1740    // 87,0 MHz (jednostka częstotliwości = 50 kHz)
#define FREQ_MIN         1750    // 87,5 MHz
#define FREQ_MAX         2160    // 108,0 MHz
#define FREQ_DEFAULT     2000    // 100,0 MHz
#define VOLUME_DEFAULT   8       // 0..15
#define LONG_PRESS_MS    600

// Maksymalny poziom błędów bloku RDS akceptowany przez dekoder:
//   0 = tylko bloki bez błędów (najmniej "literówek", wolniej przy słabym sygnale)
//   1 = także bloki poprawione przez układ (szybciej, domyślnie)
//   3 = bez filtrowania
#define RDS_MAX_BLER     1

// 1 = wyłącz wewnętrzne pull-upy Nano (5 V) na SDA/SCL. Włącz, jeśli
// moduły mają własne pull-upy do 3,3 V (typowe dla modułów OLED/RDA5807M).
#define DISABLE_INTERNAL_PULLUPS 0

// Czcionki (U8g2)
#define FONT_FREQ        u8g2_font_9x15B_tr   // częstotliwość
#define FONT_PS          u8g2_font_7x13B_tr   // PS
#define FONT_INFO        u8g2_font_6x10_tr    // PI, PTY, "MHz"
#define FONT_SMALL       u8g2_font_5x7_tr     // głośność, ST, RDS
#define FONT_RT          u8g2_font_5x7_tr     // RadioText
#define RT_LINE_CHARS    25                   // znaków w linii RadioText
#define RT_LINES         3                    // 3 x 25 = 75 >= 64 znaki RT

// Wyświetlacz w trybie "page buffer" (mało RAM-u - ważne na Nano)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ============================== RDA5807M ==============================
#define RDA_ADDR_SEQ     0x10    // dostęp sekwencyjny (odczyt od rejestru 0x0A)
#define RDA_ADDR_REG     0x11    // dostęp do pojedynczego rejestru

#define REG_CONTROL      0x02
#define REG_CHANNEL      0x03
#define REG_SYSCFG       0x04
#define REG_VOLUME       0x05

// 0x02: DHIZ=1 DMUTE=1 | RDS_EN=1 NEW_METHOD=1 ENABLE=1
#define CTRL_BASE        0xC00D
#define CTRL_MUTE_BIT    0x4000
#define CTRL_MUTED       ((uint16_t)(CTRL_BASE & ~CTRL_MUTE_BIT))
#define CTRL_SEEK        0x0100
#define CTRL_SEEKUP      0x0200

// =============================== STAN =================================
uint16_t freq50 = FREQ_DEFAULT;   // częstotliwość w jednostkach 50 kHz
uint8_t  volume = VOLUME_DEFAULT;
uint16_t reg[6];                 // rejestry 0x0A..0x0F

bool     stereo      = false;
uint8_t  rssiBars    = 0;
bool     rdsActive   = false;
uint32_t lastGroupMs = 0;
bool     dirty       = true;     // trzeba przerysować ekran

bool     settingsPending = false;
uint32_t settingsMs      = 0;

// ---- dane RDS ----
uint16_t piOut  = 0;  bool piOk  = false;
uint8_t  ptyOut = 0;  bool ptyOk = false;
char     psTxt[9];                       // PS (wyświetlany)

// RadioText: bufor roboczy (rtStage) i tekst wyświetlany (rtTxt)
char     rtStage[65];
char     rtTxt[65];
bool     rtShown = false;                // czy RT został już raz zatwierdzony
uint16_t rtMask  = 0;                    // bit n = odebrano segment o adresie n
uint8_t  rtEnd   = 64;                   // pozycja znaku 0x0D (64 = nieznana)
bool     rtAB = false, rtABvalid = false;
uint8_t  rtPrevAddr = 0, rtPassMax = 0, rtPassMaxPrev = 0xFF;

uint16_t lastA = 0, lastB = 0, lastC = 0, lastD = 0;   // do odrzucania duplikatów

// ---- teksty/dane do rysowania (kopia robocza na czas jednej klatki) ----
char sFreq[8], sVol[6], sPs[9], sPi[10], sPty[26];
char rtL[RT_LINES][RT_LINE_CHARS + 1];
bool dStereo = false, dRds = false;
uint8_t dBars = 0;

// ---- przyciski ----
enum { EV_NONE = 0, EV_SHORT = 1, EV_LONG = 2 };
struct Btn { uint8_t pin; bool down; bool longSent; uint32_t t0; };
Btn btnUp   = { PIN_FREQ_UP,   false, false, 0 };
Btn btnDown = { PIN_FREQ_DOWN, false, false, 0 };
Btn btnVUp  = { PIN_VOL_UP,    false, false, 0 };
Btn btnVDn  = { PIN_VOL_DOWN,  false, false, 0 };

// Nazwy PTY (RDS, Europa) - 32 pozycje oddzielone znakiem '|'
const char PTY_NAMES[] PROGMEM =
  "No PTY|News|Affairs|Info|Sport|Education|Drama|Culture|Science|Varied|"
  "Pop Music|Rock Music|Easy Listening|Light Classic|Classical|Other Music|"
  "Weather|Finance|Children|Social|Religion|Phone In|Travel|Leisure|"
  "Jazz Music|Country Music|National Music|Oldies Music|Folk Music|"
  "Documentary|Alarm Test|ALARM!";

// ============================ I2C: RDA5807M ===========================
void rdaWrite(uint8_t r, uint16_t val) {
  Wire.beginTransmission((uint8_t)RDA_ADDR_REG);
  Wire.write(r);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

// Wartość rejestru sterującego dla normalnej pracy. Przy głośności 0 zostaje
// włączone wyciszenie sprzętowe (DMUTE) - sama głośność 0 w RDA5807M nie daje
// całkowitej ciszy.
uint16_t ctrlRun() { return (volume == 0) ? CTRL_MUTED : CTRL_BASE; }

// Czyta rejestry 0x0A..0x0F do reg[0..5]
bool readRegs() {
  if (Wire.requestFrom((uint8_t)RDA_ADDR_SEQ, (uint8_t)12) != 12) return false;
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t hi = Wire.read();     // kolejność odczytu musi być jawna
    uint8_t lo = Wire.read();
    reg[i] = ((uint16_t)hi << 8) | lo;
  }
  return true;
}

bool waitStc(uint16_t timeoutMs) {            // czekaj na koniec strojenia/szukania
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (readRegs() && (reg[0] & 0x4000)) return true;   // STC
    delay(10);
  }
  return false;
}

// ================================ RDS =================================
// Kasowanie WSZYSTKICH danych RDS - tylko przy zmianie stacji
void resetRds() {
  strcpy(psTxt, "--------");
  memset(rtStage, ' ', 64);  rtStage[64] = 0;
  memset(rtTxt,   ' ', 64);  rtTxt[64]   = 0;
  rtShown = false;
  rtMask = 0; rtEnd = 64; rtABvalid = false;
  rtPrevAddr = 0; rtPassMax = 0; rtPassMaxPrev = 0xFF;
  piOk = false; ptyOk = false;
  lastA = lastB = lastC = lastD = 0;
  lastGroupMs = 0;
  rdsActive = false;
  dirty = true;
}

// Znak RDS -> ASCII (czcionka OLED nie ma polskich liter)
char cleanChar(uint8_t ch) {
  if (ch >= 0x20 && ch <= 0x7E) return (char)ch;
  if (ch >= 0x80) return '?';
  return ' ';
}

// ---- PS: znaki wpisywane od razu ----
void putPs(uint8_t pos, uint8_t ch) {
  if (pos >= 8) return;
  char c = cleanChar(ch);
  if (psTxt[pos] != c) { psTxt[pos] = c; dirty = true; }
}

// ---- RadioText ----
// Zatwierdza tekst z bufora roboczego na ekran, jeśli odebrano segmenty 0..lastSeg.
// cut = pozycja końca tekstu (64 = cały bufor).
void rtCommit(uint8_t lastSeg, uint8_t cut) {
  uint16_t need = (lastSeg >= 15) ? 0xFFFF : (uint16_t)((1u << (lastSeg + 1)) - 1);
  if ((rtMask & need) != need) return;          // jeszcze nie wszystko odebrane

  uint8_t n = (cut < 64) ? (uint8_t)(cut + 1) : 64;
  bool diff = false;
  for (uint8_t i = 0; i < n; i++) {
    char ch = (i == cut) ? '\r' : rtStage[i];   // '\r' = koniec tekstu
    if (rtTxt[i] != ch) { rtTxt[i] = ch; diff = true; }
  }
  rtShown = true;
  if (diff) dirty = true;
}

// Zapisuje jeden segment (2 lub 4 znaki) do bufora roboczego
void rtStoreSegment(uint8_t addr, uint8_t segLen, const uint8_t *raw) {
  uint8_t base = addr * segLen;
  char cc[4];
  for (uint8_t i = 0; i < segLen; i++) cc[i] = (raw[i] == 0x0D) ? '\r' : cleanChar(raw[i]);

  // Ten segment był już odebrany, a treść jest inna => nadawana jest nowa
  // wiadomość (lub błąd transmisji): zbieraj od nowa, ale NIE ruszaj ekranu.
  if (rtMask & (1u << addr)) {
    bool changed = false;
    for (uint8_t i = 0; i < segLen; i++) if (rtStage[base + i] != cc[i]) changed = true;
    if (changed) { rtMask = 0; rtEnd = 64; }
  }

  for (uint8_t i = 0; i < segLen; i++) {
    rtStage[base + i] = cc[i];
    if (cc[i] == '\r' && (uint8_t)(base + i) < rtEnd) rtEnd = base + i;
  }
  rtMask |= (1u << addr);

  // Pierwsze wczytanie po zmianie stacji: pokazuj znaki od razu, w miarę odbioru
  if (!rtShown) {
    for (uint8_t i = 0; i < segLen; i++) rtTxt[base + i] = cc[i];
    dirty = true;
  }
}

void decodeGroup(uint16_t a, uint16_t b, uint16_t c, uint16_t d, uint8_t blerA) {
  uint8_t groupType = b >> 12;
  bool    versionB  = (b >> 11) & 1;
  uint8_t pty       = (b >> 5) & 0x1F;
  bool    okA       = (blerA <= RDS_MAX_BLER);

  // --- PI (blok A) - od razu ---
  if (okA && a != 0 && (!piOk || piOut != a)) { piOut = a; piOk = true; dirty = true; }

  // --- PTY (blok B) - od razu ---
  if (!ptyOk || ptyOut != pty) { ptyOut = pty; ptyOk = true; dirty = true; }

  if (!okA) return;                             // teksty tylko z czystym blokiem A

  switch (groupType) {
    case 0: {                                   // 0A / 0B : PS (Program Service)
      uint8_t addr = b & 0x03;                  // segment 0..3 (po 2 znaki)
      putPs(addr * 2,     d >> 8);
      putPs(addr * 2 + 1, d & 0xFF);
      break;
    }
    case 2: {                                   // 2A / 2B : RadioText
      bool    ab     = (b >> 4) & 1;            // flaga A/B - zmiana = nowy tekst
      uint8_t addr   = b & 0x0F;
      uint8_t segLen = versionB ? 2 : 4;

      if (!rtABvalid || ab != rtAB) {           // nowa wiadomość: czyść tylko bufor roboczy
        rtAB = ab; rtABvalid = true;
        memset(rtStage, ' ', 64);
        rtMask = 0; rtEnd = 64;
        rtPrevAddr = 0; rtPassMax = 0; rtPassMaxPrev = 0xFF;
      }

      // Koniec cyklu nadawania (adres zawrócił). Obsługa stacji, które nadają
      // krótszy tekst bez znaku końca 0x0D: dwa cykle z tym samym zakresem.
      if (addr < rtPrevAddr) {
        if (rtEnd >= 64 && rtPassMax < 15 && rtPassMax == rtPassMaxPrev)
          rtCommit(rtPassMax, (uint8_t)((rtPassMax + 1) * segLen));
        rtPassMaxPrev = rtPassMax;
        rtPassMax = 0;
      }
      rtPrevAddr = addr;
      if (addr > rtPassMax) rtPassMax = addr;

      uint8_t raw[4];
      if (!versionB) { raw[0] = c >> 8; raw[1] = c & 0xFF; raw[2] = d >> 8; raw[3] = d & 0xFF; }
      else           { raw[0] = d >> 8; raw[1] = d & 0xFF; }
      rtStoreSegment(addr, segLen, raw);

      if (rtEnd < 64) rtCommit(rtEnd / segLen, rtEnd);   // znamy koniec tekstu
      else            rtCommit(15, 64);                  // pełne 64 znaki
      break;
    }
    default: break;
  }
}

// Odczyt statusu radia + grupy RDS
void pollRadio() {
  if (!readRegs()) return;

  uint16_t st = reg[0];                         // 0x0A
  uint16_t q  = reg[1];                         // 0x0B

  bool    st_stereo = st & 0x0400;
  uint8_t rssi      = q >> 9;
  uint8_t bars      = (rssi > 10) ? (rssi - 10) / 8 : 0;
  if (bars > 5) bars = 5;
  if (st_stereo != stereo || bars != rssiBars) {
    stereo = st_stereo; rssiBars = bars; dirty = true;
  }

  if (!(st & 0x8000)) return;                   // RDSR: brak nowej grupy

  uint16_t a = reg[2], b = reg[3], c = reg[4], d = reg[5];
  uint8_t blerA = (q >> 2) & 3;
  uint8_t blerB = q & 3;
  if (blerB > RDS_MAX_BLER) return;             // blok B uszkodzony - odrzuć grupę

  if (a == lastA && b == lastB && c == lastC && d == lastD) return;   // duplikat
  lastA = a; lastB = b; lastC = c; lastD = d;
  lastGroupMs = millis();
  decodeGroup(a, b, c, d, blerA);
}

// ============================= STEROWANIE =============================
void settingsChanged() { settingsPending = true; settingsMs = millis(); }

void saveSettings() {
  EEPROM.update(0, 0xA7);
  EEPROM.put(1, freq50);
  EEPROM.update(3, volume);
  settingsPending = false;
}

void loadSettings() {
  if (EEPROM.read(0) == 0xA7) {
    uint16_t f; EEPROM.get(1, f);
    uint8_t v = EEPROM.read(3);
    if (f >= FREQ_MIN && f <= FREQ_MAX) freq50 = f;
    if (v <= 15) volume = v;
  }
}

void tuneTo(uint16_t f50) {
  if (f50 < FREQ_MIN) f50 = FREQ_MIN;
  if (f50 > FREQ_MAX) f50 = FREQ_MAX;
  freq50 = f50;
  rdaWrite(REG_CONTROL, CTRL_MUTED);
  // CHAN = (f - 87,0 MHz) / 0,05 MHz ; BAND=00 (87-108), SPACE=10 (50 kHz), TUNE=1
  rdaWrite(REG_CHANNEL, (uint16_t)(((f50 - FREQ_BASE) << 6) | 0x0010 | 0x0002));
  delay(40);
  waitStc(1000);
  rdaWrite(REG_CONTROL, ctrlRun());
  resetRds();
  settingsChanged();
}

void stepFreq(int8_t dir) {
  int16_t f = (int16_t)freq50 + dir;
  if (f > FREQ_MAX) f = FREQ_MIN;
  if (f < FREQ_MIN) f = FREQ_MAX;
  tuneTo((uint16_t)f);
}

void showMessage(const char *l1, const char *l2);

void seekStation(bool up) {
  showMessage("Szukam stacji...", up ? "w gore" : "w dol");
  // Szukanie stacji działa jak dotąd na siatce 100 kHz: ustaw odstęp 100 kHz
  // i stań na najbliższym kanale z tej siatki.
  rdaWrite(REG_CONTROL, CTRL_MUTED);
  rdaWrite(REG_CHANNEL, (uint16_t)((((freq50 - FREQ_BASE + 1) / 2) << 6) | 0x0010));   // SPACE=00, TUNE=1
  delay(40);
  waitStc(1000);
  rdaWrite(REG_CONTROL, CTRL_MUTED | CTRL_SEEK | (up ? CTRL_SEEKUP : 0));
  delay(60);
  bool ok = waitStc(6000);
  uint16_t ch = reg[0] & 0x03FF;               // READCHAN (w krokach 100 kHz)
  rdaWrite(REG_CONTROL, ctrlRun());
  uint16_t f = FREQ_BASE + ch * 2;
  if (ok && f >= FREQ_MIN && f <= FREQ_MAX) freq50 = f;   // znaleziono stację
  tuneTo(freq50);                              // strojenie z odstępem 50 kHz (lub powrót na starą)
}

void setVolume(int8_t v) {
  if (v < 0)  v = 0;
  if (v > 15) v = 15;
  volume = v;
  rdaWrite(REG_VOLUME, 0x8880 | volume);
  rdaWrite(REG_CONTROL, ctrlRun());            // głośność 0 = pełne wyciszenie
  dirty = true;
  settingsChanged();
}

void rdaInit() {
  rdaWrite(REG_CONTROL, 0x0002);               // SOFT_RESET
  delay(100);
  rdaWrite(REG_CONTROL, ctrlRun());            // ENABLE, RDS_EN, NEW_METHOD
  delay(100);
  rdaWrite(REG_SYSCFG, 0x0C00);                // DE = 50 us (Europa)
  rdaWrite(REG_VOLUME, 0x8880 | volume);
  tuneTo(freq50);
}

// ============================== PRZYCISKI =============================
uint8_t btnEvent(Btn &b) {
  bool pressed = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (pressed && !b.down) { b.down = true; b.longSent = false; b.t0 = now; return EV_NONE; }
  if (pressed && b.down) {
    if (!b.longSent && now - b.t0 >= LONG_PRESS_MS) { b.longSent = true; return EV_LONG; }
    return EV_NONE;
  }
  if (!pressed && b.down) {
    b.down = false;
    if (!b.longSent && now - b.t0 >= 30) return EV_SHORT;
  }
  return EV_NONE;
}

// =============================== EKRAN ================================
void getPtyName(uint8_t pty, char *out, uint8_t maxLen) {
  const char *p = PTY_NAMES;
  uint8_t idx = 0;
  while (idx < pty) {
    char c = pgm_read_byte(p++);
    if (c == '|') idx++;
    else if (c == 0) break;
  }
  uint8_t n = 0;
  while (n < maxLen - 1) {
    char c = pgm_read_byte(p++);
    if (c == '|' || c == 0) break;
    out[n++] = c;
  }
  out[n] = 0;
}

// Dzieli RadioText na RT_LINES linii: najpierw łamie na spacjach,
// a jeśli się nie mieści - łamie "twardo" co RT_LINE_CHARS znaków.
void wrapRt(const char *rt, uint8_t n) {
  for (uint8_t i = 0; i < RT_LINES; i++) rtL[i][0] = 0;
  if (n == 0) return;

  uint8_t pos = 0;
  for (uint8_t ln = 0; ln < RT_LINES && pos < n; ln++) {
    uint8_t remain = n - pos;
    uint8_t len;
    if (remain <= RT_LINE_CHARS) {
      len = remain;
    } else {
      len = RT_LINE_CHARS;                                  // brak spacji -> łamanie twarde
      for (uint8_t i = RT_LINE_CHARS; i > 0; i--)
        if (rt[pos + i] == ' ') { len = i; break; }
    }
    memcpy(rtL[ln], rt + pos, len);
    rtL[ln][len] = 0;
    pos += len;
    while (pos < n && rt[pos] == ' ') pos++;                // pomiń spacje na łamaniu
  }

  if (pos < n) {                                            // nie zmieściło się -> łamanie twarde
    for (uint8_t ln = 0; ln < RT_LINES; ln++) {
      uint8_t start = ln * RT_LINE_CHARS;
      if (start >= n) { rtL[ln][0] = 0; continue; }
      uint8_t len = n - start;
      if (len > RT_LINE_CHARS) len = RT_LINE_CHARS;
      memcpy(rtL[ln], rt + start, len);
      rtL[ln][len] = 0;
    }
  }
}

// Kopia danych do narysowania (raz na klatkę). Dzięki temu można odbierać RDS
// także w trakcie rysowania, bez ryzyka zmiany tekstu w połowie klatki.
void prepareStrings() {
  snprintf_P(sFreq, sizeof(sFreq), PSTR("%u.%02u"), (unsigned)(freq50 / 20), (unsigned)((freq50 % 20) * 5));
  snprintf_P(sVol,  sizeof(sVol),  PSTR("V:%u"), (unsigned)volume);
  memcpy(sPs, psTxt, 9);

  if (piOk) snprintf_P(sPi, sizeof(sPi), PSTR("PI:%04X"), (unsigned)piOut);
  else      strcpy_P(sPi, PSTR("PI:----"));

  if (ptyOk) {
    char name[16];
    getPtyName(ptyOut, name, sizeof(name));
    snprintf_P(sPty, sizeof(sPty), PSTR("PTY:%02u-%s"), (unsigned)ptyOut, name);
  } else {
    strcpy_P(sPty, PSTR("PTY:-- ---"));
  }

  dStereo = stereo;
  dRds    = rdsActive;
  dBars   = rssiBars;

  // RadioText: do pierwszego 0x0D, bez spacji na końcu
  char rt[65];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 64; i++) {
    if (rtTxt[i] == '\r') break;
    rt[n++] = rtTxt[i];
  }
  while (n && rt[n - 1] == ' ') n--;
  rt[n] = 0;
  wrapRt(rt, n);
}

void drawScreen() {
  // --- wiersz 1: częstotliwość ---
  u8g2.setFont(FONT_FREQ);
  u8g2.drawStr(0, 12, sFreq);
  uint8_t w = u8g2.getStrWidth(sFreq);
  u8g2.setFont(FONT_INFO);
  u8g2.drawStr(w + 3, 12, "MHz");

  // --- prawy górny róg: głośność, ST, RDS, siła sygnału ---
  u8g2.setFont(FONT_SMALL);
  u8g2.drawStr(78, 6, sVol);
  if (dStereo) u8g2.drawStr(78, 13, "ST");
  if (dRds)    u8g2.drawStr(93, 13, "RDS");
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t h = 2 + i * 2;
    uint8_t x = 113 + i * 3;
    if (i < dBars) u8g2.drawBox(x, 13 - h, 2, h);
    else           u8g2.drawHLine(x, 12, 2);
  }

  // --- wiersz 2: PS oraz PI ---
  u8g2.setFont(FONT_PS);
  u8g2.drawStr(0, 25, sPs);
  u8g2.setFont(FONT_INFO);
  u8g2.drawStr(72, 25, sPi);

  // --- wiersz 3: PTY ---
  u8g2.drawStr(0, 36, sPty);

  // --- RadioText (3 linie) ---
  u8g2.setFont(FONT_RT);
  for (uint8_t i = 0; i < RT_LINES; i++)
    u8g2.drawStr(0, 46 + i * 8, rtL[i]);
}

void drawFrame() {
  u8g2.firstPage();
  do {
    drawScreen();
    pollRadio();          // odbieraj RDS także w trakcie rysowania - żadna grupa nie ginie
  } while (u8g2.nextPage());
}

void showMessage(const char *l1, const char *l2) {
  u8g2.firstPage();
  do {
    u8g2.setFont(FONT_PS);
    u8g2.drawStr(0, 26, l1);
    if (l2) {
      u8g2.setFont(FONT_INFO);
      u8g2.drawStr(0, 46, l2);
    }
  } while (u8g2.nextPage());
}

// ============================ SETUP / LOOP ============================
void setup() {
  pinMode(PIN_FREQ_UP,   INPUT_PULLUP);
  pinMode(PIN_FREQ_DOWN, INPUT_PULLUP);
  pinMode(PIN_VOL_UP,    INPUT_PULLUP);
  pinMode(PIN_VOL_DOWN,  INPUT_PULLUP);

  u8g2.begin();                 // inicjalizuje też Wire
  u8g2.setBusClock(100000);
#if DISABLE_INTERNAL_PULLUPS
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
#endif

  loadSettings();
  resetRds();

  Wire.beginTransmission((uint8_t)RDA_ADDR_SEQ);
  if (Wire.endTransmission() != 0) {
    showMessage("BLAD I2C", "Brak RDA5807M (0x10)");
    while (true) { }
  }

  showMessage("FM Radio RDS", "RDA5807M start...");
  delay(2000);
  rdaInit();
  dirty = true;
}

void loop() {
  uint8_t e;

  e = btnEvent(btnUp);
  if (e == EV_SHORT) stepFreq(+1); else if (e == EV_LONG) seekStation(true);

  e = btnEvent(btnDown);
  if (e == EV_SHORT) stepFreq(-1); else if (e == EV_LONG) seekStation(false);

  if (btnEvent(btnVUp)) setVolume((int8_t)volume + 1);
  if (btnEvent(btnVDn)) setVolume((int8_t)volume - 1);

  static uint32_t tPoll = 0;
  if (millis() - tPoll >= 10) {
    tPoll = millis();
    pollRadio();

    bool act = (lastGroupMs != 0) && (millis() - lastGroupMs < 5000);
    if (act != rdsActive) { rdsActive = act; dirty = true; }
  }

  static uint32_t tDraw = 0;
  if (dirty && millis() - tDraw >= 100) {
    dirty = false;              // zmiany w trakcie rysowania ustawią flagę ponownie
    prepareStrings();
    drawFrame();
    tDraw = millis();
  }

  if (settingsPending && millis() - settingsMs > 3000) saveSettings();
}
