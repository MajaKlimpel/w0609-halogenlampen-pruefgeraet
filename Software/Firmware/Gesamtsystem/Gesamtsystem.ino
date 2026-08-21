#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

// ================= DAC =================
#define DAC_ADDR 0x0F

const int numChannels = 10;
int enablePins[numChannels] = {0,1,2,3,4,5,6,7,8,9};
float channelVoltage[numChannels] = {0};

const float VREF = 5.0;
const uint16_t DAC_MAX = 65535;

// ================= FAN =================
// Fan1 = rechts (FanR), Fan2 = links (FanL)
const int pinPWM = 33;
const int tach1 = 28;   // Fan1 = rechts
const int tach2 = 29;   // Fan2 = links

volatile unsigned long pulsesFan1 = 0;
volatile unsigned long pulsesFan2 = 0;

volatile unsigned long lastPulseTime1 = 0;
volatile unsigned long lastPulseTime2 = 0;

unsigned long lastStatusTime = 0;   // gemeinsamer Takt fuer die Statusausgabe (1 s)

// Luefter-Kennlinie: linear 0V -> 0%, ab 5V -> 100% (auch bei spaeteren 7V)
const float FAN_FULL_VOLT = 5.0;   // Spannung, ab der 100% erreicht sind
const int   FAN_MIN_COOL  = 30;    // Mindestdrehzahl in der Abkuehlphase [%]
bool fanFloorActive = false;       // 30%-Floor erst nach dem ersten Hochlauf aktiv

// Luefter-Nachlauf: nach Testende/Stop noch ~2 min auf 30% (Restwaerme abfuehren)
const int           FAN_RUNOUT_PCT = 30;
const unsigned long FAN_RUNOUT_MS  = 120000UL;   // 2 Minuten
unsigned long       fanRunoutUntil = 0;

// ================= TEMP =================
// T1..T6 mit fest verdrahteten I2C-Adressen (aus den Binaeradressen umgerechnet)
// T1=1001011=0x4B, T2=1001101=0x4D, T3=1001000=0x48,
// T4=1001010=0x4A, T5=1001100=0x4C, T6=1001001=0x49
uint8_t sensors[] = {0x4B, 0x4D, 0x48, 0x4A, 0x4C, 0x49};
const int numSensors = 6;

// ================= STROM- & SPANNUNGSMESSUNG =================
// Beide Multiplexer (CD74HC4067) teilen sich die Steuerleitungen S0..S3 (parallel).
const int muxS0   = 23;   // S0
const int muxS1   = 22;   // S1
const int muxS2   = 21;   // S2
const int muxS3   = 20;   // S3

// Strom:    Shunt 75 mOhm -> INA190A1 (Gain 25 V/V) -> Mux 1 -> Pin 14
const int muxSigI = 14;   // Ausgang Strom-Mux
// Spannung: Teiler Rtop=11,3k / Rbot=10k -> Spannungsfolger TLV9001 -> Mux 2 -> Pin 15
const int muxSigU = 15;   // Ausgang Spannungs-Mux

const int   numCurrent = 10;        // I1..I10 = Mux-Kanal 0..9
const int   numVoltage = 10;        // U1..U10 = Mux-Kanal 0..9

const float SHUNT_OHM  = 0.075;     // 75 mOhm
const float INA_GAIN   = 25.0;      // INA190A1 = 25 V/V

const float VDIV_RTOP   = 11300.0;  // Spannungsteiler oben [Ohm]
const float VDIV_RBOT   = 10000.0;  // Spannungsteiler unten [Ohm]
const float VDIV_FACTOR = (VDIV_RTOP + VDIV_RBOT) / VDIV_RBOT;  // = 2,13

const float ADC_REF    = 3.3;       // Teensy 4.1 ADC-Referenz
const int   ADC_MAX    = 4095;      // 12-bit Aufloesung

// ================= SPANNUNGSREGELUNG =================
// Jeder Kanal wird anhand der gemessenen Ausgangsspannung auf den Sollwert nachgeregelt.
const float REG_GAIN     = 0.4;     // Regelverstaerkung (sanft -> kein Ueberschwingen)
const float REG_DEADBAND = 0.004;   // V: > ADC-Aufloesung (~1,7 mV), sonst Pendeln
const float REG_STEP_MAX = 0.01;    // max. Korrektur pro Zyklus [V] (gegen Ausreisser)
const float TRIM_LIMIT   = 2.0;     // max. Korrektur am DAC-Wert [V]

// Gelernte Korrektur je Kanal. Gilt NUR im Haltepunkt (volle Spannung) - waehrend der
// Rampen wird sie absichtlich nicht angewendet (siehe setVoutRamp), und bei jedem
// Teststart wird sie verworfen (siehe startTest).
float vdacTrim[numChannels]    = {0};
float voltageMeas[numChannels] = {0};  // zuletzt gemessene Ausgangsspannung je Kanal

// Letzte Messwerte (fuer GUI-Abfragen, DATA-Stream, Logging)
float lastCurrent[numChannels] = {0};
float lastTemp[numSensors]     = {0};
int   lastRpmR = 0, lastRpmL = 0;
// Drehzahl-Messfenster bewusst ~1 s, UNABHAENGIG vom (schnellen) Stream-Intervall.
// Bei 100 ms Stream fallen bei ~2000 U/min nur 6-7 Tacho-Pulse an -> ein Puls
// mehr/weniger = +-300 U/min "Springen". Ueber ~1 s werden ~66 Pulse gezaehlt
// -> Aufloesung ~30 U/min, die Anzeige bleibt ruhig.
unsigned long lastRpmTime = 0;
const unsigned long RPM_WINDOW_MS = 1000;
unsigned long statusMs = 1000;   // Mess-/Ausgabe-Intervall [ms]
bool  streamOn = true;           // automatische DATA-Ausgabe an die GUI
bool  guiConnected = false;      // GUI/Software steuert das Geraet -> anderer Startbildschirm

// Zweipunkt-Kalibrierung der Spannungsmessung: U_korr = vCalA * U_roh + vCalB
// vCalA: 2-Punkt (1V/5V). vCalB am 25.06.2026 bei 5V UNTER LAST feinabgeglichen
// (vCalB += V_Lampe - U), damit die Messung am Arbeitspunkt exakt der Lampenspannung entspricht.
//                          K1       K2        K3       K4        K5       K6       K7       K8       K9       K10
float vCalA[numChannels] = {0.98657, 0.99445,  0.99250, 0.99473,  0.99126, 0.98780, 0.98315, 0.98827, 0.98801, 0.98726};
float vCalB[numChannels] = {0.02990,-0.00639,  0.00362,-0.00466,  0.00826, 0.02613, 0.05216, 0.02710, 0.02621, 0.03221};

bool holdMode = false;   // Kalibrier-/Halte-Modus: Test + Regelung pausiert

// ================= AUSGANGS-KALIBRIERUNG (Vorsteuerung) =================
// Pro Kanal: vdac = dacM[i] * Vout_soll + dacB[i]  (hardcoded).
// Damit trifft die reine Vorsteuerung (open-loop) bereits den Sollwert -> keine Regelung noetig.
// dacB am 25.06.2026 per 'dac'-Befehl direkt am Multimeter auf echte 5,00 V abgeglichen.
// dacM (Steigung, fuer die Rampe) stammt aus der 2-Punkt-Charakterisierung.
float dacM[numChannels] = {-0.71530,-0.71947,-0.71494,-0.71584,-0.71494,-0.71530,-0.71494,-0.71620,-0.71711,-0.71729};
float dacB[numChannels] = { 5.03150, 5.01235, 5.01270, 5.00420, 5.04170, 4.99550, 5.00970, 5.02500, 5.01755, 5.02345};

// Ausgangs-Feinkalibrierung (per GUI): korrigiert den Sollwert VOR der Vorsteuerung,
// damit die real anliegende Spannung dem eingestellten Sollwert entspricht.
//   Vout_korr = outCalM[i] * Vout_soll + outCalB[i]   (Standard 1 / 0 = keine Aenderung)
// Wird per Befehl CALSET gesetzt und mit CALSAVE dauerhaft im EEPROM abgelegt.
float outCalM[numChannels] = {1,1,1,1,1,1,1,1,1,1};
float outCalB[numChannels] = {0,0,0,0,0,0,0,0,0,0};
const uint32_t CAL_MAGIC = 0x57303643UL;   // 'W06C' -> gueltige EEPROM-Kalibrierung

bool useRegulation = true;    // Closed-loop-Nachregelung AN: kompensiert Lastabfall + Lampenunterschiede

// ================= TASTER + LED =================
const int pinRed    = 36;
const int pinGreen  = 37;
const int pinBlue   = 38;
const int pinButton = 35;

bool lastButton = LOW;   // INPUT_PULLDOWN -> Ruhezustand LOW, gedrueckt HIGH

// ================= TEST-ABLAUF =================
// IDLE   = bereit, LED weiss
// RUNNING= Test laeuft, LED blau
// DONE   = Test fertig, LED gruen
enum TestState { TEST_IDLE, TEST_RUNNING, TEST_DONE };
TestState testState = TEST_IDLE;

float testVoltage = 5.0;       // globale Soll-Spannung im Test (0..7 V, Vorgabe)
// Soll-Spannung PRO KANAL im Test (Vorgabe 5 V; per 'TESTVCH' einzeln setzbar)
float testVoltageCh[numChannels] = {5,5,5,5,5,5,5,5,5,5};
int   numCycles   = 10;        // Anzahl Zyklen pro Lampe (einstellbar)
const float MAX_VOUT = 7.0;    // erlaubte Maximalspannung

// --- Zeiten ---
const unsigned long STAGGER_MS = 1000;   // Zeitversatz zwischen den Slots

// Zeiten je Modus (Eval = schnell zum Testen, Real = echter Pruefzyklus):
// Rampe (Vout 0<->5V) + An-Zeit (volle Spannung) + Rampe runter + Aus-/Abkuehlzeit (0 V)
// Presets je Modus
const unsigned long RAMP_MS_EVAL = 2000;          // 2 s
const unsigned long ON_MS_EVAL   = 5000;          // 5 s
const unsigned long OFF_MS_EVAL  = 2000;          // 2 s
const unsigned long RAMP_MS_REAL = 1UL*60*1000;   // 1 min
const unsigned long ON_MS_REAL   = 30UL*60*1000;  // 30 min
const unsigned long OFF_MS_REAL  = 15UL*60*1000;  // 15 min

// Laufzeit-Parameter (per Serial/GUI einstellbar) - Start = Real
bool evalMode = false;
unsigned long rampDur = RAMP_MS_REAL;   // Rampe hoch = runter
unsigned long onDur   = ON_MS_REAL;     // Haltezeit bei Vollspannung
unsigned long offDur  = OFF_MS_REAL;    // Abkuehlzeit (0 V)

unsigned long rampMs() { return rampDur; }
unsigned long onMs()   { return onDur; }
unsigned long offMs()  { return offDur; }

// Phasen, die ein einzelner Slot durchlaeuft
enum SlotPhase { S_WAIT_START, S_RAMP_UP, S_HOLD, S_RAMP_DOWN, S_OFF, S_DONE };

struct Slot {
  SlotPhase     phase;
  uint8_t       cycle;        // aktueller Zyklus 0..numCycles
  unsigned long phaseStart;   // millis() beim Beginn der aktuellen Phase
};
Slot slots[numChannels];

unsigned long lastTestTick = 0;
const unsigned long TEST_TICK_MS = 20;   // Rampen-/DAC-Update alle 20 ms

// ================= DISPLAY (ILI9341) =================
#define TFT_CS   10
#define TFT_RST  40
#define TFT_DC   39
#define TFT_LITE 41
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

#define KNAUER_BLUE 0x19F1   // Markenblau (Navy/Royal, RGB565)

// Bildschirm-Zustaende
enum Screen { SCR_LOGO, SCR_START, SCR_TEST };
Screen screen = SCR_LOGO;
unsigned long screenStart = 0;
const unsigned long LOGO_MS = 3000;   // Logo 3 s anzeigen

// Tabellen-Geometrie
const int TBL_X = 10, TBL_Y = 10, TBL_W = 300, TBL_H = 221;
const int TBL_ROWS = 6, TBL_COLS = 2;
const int TBL_ROWH = TBL_H / TBL_ROWS;   // 36
const int TBL_COLW = TBL_W / TBL_COLS;   // 150

// Fehlererkennung + Zeichen-Cache (nur bei Aenderung neu zeichnen)
bool      lampDefect[numChannels]   = {false};   // Lampe defekt (gelatcht, bis Testende)
uint16_t  lampDrawnColor[numChannels];           // zuletzt gezeichnete Farbe je Lampe
const float CURRENT_MIN = 0.5;   // < 500 mA im Haltepunkt -> Lampe durchgebrannt

// Fortschrittsbalken (fliessend, zeitbasiert)
unsigned long testStartMs = 0;     // Startzeitpunkt des Tests
unsigned long testTotalMs = 1;     // erwartete Gesamtdauer des Tests
int           barLastPx   = 0;     // zuletzt gezeichnete Balkenbreite [px]
unsigned long lastBarUpdate = 0;   // Throttle fuer die Balken-Animation

// ================= SERIAL =================
String inputString = "";

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  calLoadEEPROM();   // gespeicherte Ausgangs-Feinkalibrierung laden (falls vorhanden)

  Wire.begin();
  Wire1.begin();

  for (int i=0;i<numChannels;i++){
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);       // Buck-Wandler aus
    channelVoltage[i] = 0;
    setDAC(i, voutToVdac(0));                // DAC in definierten 0-V-Zustand (Enable bleibt LOW)
  }

  pinMode(pinPWM, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(pinPWM, 25000);
  analogWrite(pinPWM, 0);

  pinMode(tach1, INPUT_PULLUP);
  pinMode(tach2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(tach1), tachISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(tach2), tachISR2, RISING);
  lastRpmTime = millis();   // Startzeitpunkt fuer das ~1-s-Drehzahlfenster

  // Multiplexer / Strommessung
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);
  selectMuxChannel(0);
  analogReadResolution(12);
  analogReadAveraging(32);   // Hardware-Mittelung gegen ADC-Rauschen

  // Taster + LED
  pinMode(pinRed,   OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue,  OUTPUT);
  pinMode(pinButton, INPUT_PULLDOWN);

  setLED(TEST_IDLE);   // Start: weiss

  // Display
  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);    // Hintergrundbeleuchtung an
  tft.begin();
  tft.setRotation(1);
  tft.setTextSize(1);
  drawLogo();                      // Knauer-Logo fuer 2 s
  screen = SCR_LOGO;
  screenStart = millis();

  Serial.println("W0609 Halogenlampentest bereit. 'HELP' fuer Befehle.");
}

// ================= LOOP =================
void loop() {
  handleSerial();
  handleButton();
  updateTest();
  updateFanControl();
  updateStatus();
  updateDisplay();
  updateLED();     // LED-Farbe/Blinken (inkl. Kalibriermodus = Magenta)
}

// ================= LED =================
void setLED(TestState s) {
  bool r = false, g = false, b = false;
  switch (s) {
    case TEST_IDLE:    r = true; g = true; b = true; break; // weiss
    case TEST_RUNNING: b = true;                     break; // blau
    case TEST_DONE:    g = true;                     break; // gruen
  }
  digitalWrite(pinRed,   r ? HIGH : LOW);
  digitalWrite(pinGreen, g ? HIGH : LOW);
  digitalWrite(pinBlue,  b ? HIGH : LOW);
}

// LED-Anzeige inkl. Kalibriermodus. Wird in jedem loop() aufgerufen (fuer das Blinken).
// Im Kalibriermodus (holdMode): GELB (Rot+Gruen).
//   - solide, solange keine Spannung anliegt (0 V)  -> "Kalibriermodus aktiv, ungefaehrlich"
//   - blinkend, sobald eine Kalibrierspannung anliegt (>0 V) -> "Achtung, Spannung live"
// Sonst: normale Zustandsfarbe (weiss/blau/gruen) ueber setLED().
unsigned long      ledBlinkLast = 0;
bool               ledBlinkOn   = true;
const unsigned long LED_BLINK_MS = 400;   // Blink-Takt in ms

void updateLED() {
  if (holdMode) {
    bool live = false;
    for (int i = 0; i < numChannels; i++)
      if (channelVoltage[i] > 0.01f) { live = true; break; }
    bool on = true;
    if (live) {
      unsigned long now = millis();
      if (now - ledBlinkLast >= LED_BLINK_MS) { ledBlinkLast = now; ledBlinkOn = !ledBlinkOn; }
      on = ledBlinkOn;
    }
    digitalWrite(pinRed,   on ? HIGH : LOW);   // Gelb = Rot + Grün
    digitalWrite(pinGreen, on ? HIGH : LOW);
    digitalWrite(pinBlue,  LOW);
  } else {
    setLED(testState);
  }
}

// ================= TASTER =================
void handleButton() {
  bool b = digitalRead(pinButton);

  // steigende Flanke (LOW -> HIGH) = Taster gedrueckt
  if (lastButton == LOW && b == HIGH) {
    delay(30);  // kleines Entprellen

    if (testState == TEST_IDLE) {
      startTest();
    } else if (testState == TEST_DONE) {
      stopTest();   // zurueck zu IDLE / bereit fuer naechsten Lauf
    }
    // waehrend TEST_RUNNING: Tasterdruck wird ignoriert
  }

  lastButton = b;
}

// ================= TEST-STEUERUNG =================
void startTest() {
  unsigned long now = millis();
  holdMode = false;   // evtl. aktiven Kalibrier-/Halte-Modus sicher beenden
  for (int i=0;i<numChannels;i++){
    slots[i].phase      = S_WAIT_START;
    slots[i].cycle      = 0;
    slots[i].phaseStart = now;
    digitalWrite(enablePins[i], LOW);
    channelVoltage[i] = 0;
    vdacTrim[i] = 0;            // gelernte Korrektur verwerfen -> jeder Test startet gleich
    setDAC(i, voutToVdac(0));   // DAC vorinitialisieren (Enable noch LOW)
  }
  for (int i=0;i<numChannels;i++) lampDefect[i] = false;   // Fehler-Flags zuruecksetzen
  fanFloorActive = false;   // Luefter darf beim Start wieder von 0% hochlaufen
  testState = TEST_RUNNING;
  setLED(TEST_RUNNING);

  // erwartete Gesamtdauer fuer den fliessenden Fortschrittsbalken
  testStartMs = now;
  unsigned long cycleTotal = 2UL*rampMs() + onMs() + offMs();
  testTotalMs = (unsigned long)(numChannels - 1) * STAGGER_MS
              + (unsigned long)numCycles * cycleTotal;

  screen = SCR_TEST;        // Display auf Testansicht umschalten
  drawTestLayout();
  Serial.print("Test gestartet (");
  Serial.print(evalMode ? "Eval 5s/2s" : "Real 30min/15min");
  Serial.println(").");
}

void stopTest() {
  for (int i=0;i<numChannels;i++){
    digitalWrite(enablePins[i], LOW);
    channelVoltage[i] = 0;
    slots[i].phase = S_DONE;
  }
  fanRunoutUntil = millis() + FAN_RUNOUT_MS;   // Luefter laufen ~2 min nach
  fanFloorActive = false;
  holdMode = false;
  testState = TEST_IDLE;
  setLED(TEST_IDLE);
  screen = SCR_START;       // zurueck zum Startbildschirm
  drawIdle();
  Serial.println("Test gestoppt / zurueckgesetzt.");
}

// ================= KALIBRIER-/HALTE-MODUS =================
// Alle Kanaele open-loop auf eine feste Spannung legen (zum Kalibrieren / Messen).
void startHold(float v) {
  holdMode = true;
  testState = TEST_IDLE;
  fanFloorActive = false;

  for (int i = 0; i < numChannels; i++) {
    slots[i].phase = S_DONE;          // Test-Statemachine stilllegen
    if (v <= 0.0) {
      digitalWrite(enablePins[i], LOW);
      channelVoltage[i] = 0;
      setDAC(i, 0);
    } else {
      digitalWrite(enablePins[i], HIGH);
      channelVoltage[i] = v;
      setDAC(i, voutToVdac(v));        // reine Vorsteuerung, kein Trim, keine Regelung
    }
  }

  setLED(TEST_IDLE);
  Serial.print("HOLD-Modus: alle Kanaele open-loop auf ");
  Serial.print(v);
  Serial.println(" V (Regelung pausiert). 'raw' zum Auslesen, 'stop' zum Beenden.");
}

// Alle Kanaele auf eine Spannung setzen und dabei (kalibriert) nachregeln.
void setAllRegulated(float v) {
  holdMode = false;
  testState = TEST_IDLE;
  fanFloorActive = false;

  for (int i = 0; i < numChannels; i++) {
    slots[i].phase = S_DONE;          // Test-Statemachine inaktiv
    if (v <= 0.0) {
      digitalWrite(enablePins[i], LOW);
      channelVoltage[i] = 0;
      setDAC(i, 0);
    } else {
      digitalWrite(enablePins[i], HIGH);
      setVout(i, v);                  // Vorsteuerung + Trim; regulate() zieht nach
    }
  }

  setLED(TEST_IDLE);
  Serial.print("Alle Kanaele auf ");
  Serial.print(v);
  Serial.println(" V (kalibrierte Vorsteuerung).");
}

// DAC eines einzelnen Kanals DIREKT setzen (ohne Kennlinie/Kalibrierung/Regelung).
// Zum manuellen Einstellen, bis das Multimeter genau 5,00 V zeigt.
// ch1 = 1-basiert. Hinweis: hoeherer DAC-Wert -> kleinere Ausgangsspannung (invertierend).
void setDacDirect(int ch1, float vdac) {
  int ch = ch1 - 1;
  if (ch < 0 || ch >= numChannels) {
    Serial.println("Kanal muss 1..10 sein.");
    return;
  }
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > 5.0) vdac = 5.0;

  holdMode = true;          // Test + Regelung pausieren
  testState = TEST_IDLE;

  // alle anderen Kanaele ausschalten, nur den gemessenen aktiv
  for (int i = 0; i < numChannels; i++) {
    slots[i].phase = S_DONE;
    if (i != ch) {
      digitalWrite(enablePins[i], LOW);
      channelVoltage[i] = 0;
      setDAC(i, 0);
    }
  }

  digitalWrite(enablePins[ch], HIGH);
  channelVoltage[ch] = 5.0;   // nur fuer die Luefterregelung (Kuehlung an)
  setDAC(ch, vdac);           // DAC direkt, ungefiltert

  setLED(TEST_IDLE);
  Serial.print("Kanal ");
  Serial.print(ch1);
  Serial.print(": DAC = ");
  Serial.print(vdac, 3);
  Serial.println(" V (direkt). Multimeter messen; hoeher = weniger Vout. 'stop' beendet.");
}

// Rohe Spannungsmesswerte aller Kanaele ausgeben (fuer die Kalibrierung)
void printRaw() {
  Serial.print("RAW U: ");
  for (int i = 0; i < numVoltage; i++) {
    Serial.print("U");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(readVoltageRaw(i), 3);
    Serial.print(" V  ");
  }
  Serial.println();
}

void updateTest() {
  if (testState != TEST_RUNNING) return;

  unsigned long now = millis();
  if (now - lastTestTick < TEST_TICK_MS) return;
  lastTestTick = now;

  bool allDone = true;

  for (int i=0;i<numChannels;i++){
    Slot &s = slots[i];
    if (s.phase == S_DONE) continue;
    allDone = false;

    unsigned long elapsed = now - s.phaseStart;

    switch (s.phase) {

      case S_WAIT_START:
        // Anfangs-Zeitversatz: Slot i wartet i * STAGGER_MS
        if (elapsed >= (unsigned long)i * STAGGER_MS) {
          digitalWrite(enablePins[i], HIGH);
          s.phase = S_RAMP_UP;
          s.phaseStart = now;
        }
        break;

      case S_RAMP_UP: {
        if (elapsed >= rampMs()) {
          setVout(i, testVoltageCh[i]);
          s.phase = S_HOLD;
          s.phaseStart = now;
          fanFloorActive = true;   // ab jetzt Mindestdrehzahl in Abkuehlphasen
        } else {
          float v = testVoltageCh[i] * (float)elapsed / (float)rampMs();
          setVoutRamp(i, v);          // ohne gelernte Korrektur -> Rampe startet bei 0 V
        }
        break;
      }

      case S_HOLD:
        // volle Spannung halten (DAC bleibt unveraendert)
        if (elapsed >= onMs()) {
          s.phase = S_RAMP_DOWN;
          s.phaseStart = now;
        }
        break;

      case S_RAMP_DOWN: {
        if (elapsed >= rampMs()) {
          setVoutRamp(i, 0);          // ohne Korrektur -> wirklich 0 V, kein Rest-Offset
          digitalWrite(enablePins[i], LOW);
          if (lampDefect[i]) {
            s.phase = S_DONE;      // defekt -> stillgelegt, keine weiteren Zyklen
          } else {
            s.phase = S_OFF;
            s.phaseStart = now;
          }
        } else {
          float v = testVoltageCh[i] * (1.0 - (float)elapsed / (float)rampMs());
          setVoutRamp(i, v);          // ohne gelernte Korrektur (gilt nur im Haltepunkt)
        }
        break;
      }

      case S_OFF:
        if (elapsed >= offMs()) {
          s.cycle++;
          if (s.cycle >= numCycles) {
            s.phase = S_DONE;
          } else {
            digitalWrite(enablePins[i], HIGH);
            s.phase = S_RAMP_UP;
            s.phaseStart = now;
          }
        }
        break;

      default:
        break;
    }
  }

  if (allDone) {
    testState = TEST_DONE;
    setLED(TEST_DONE);
    fanRunoutUntil = millis() + FAN_RUNOUT_MS;   // Luefter-Nachlauf nach Testende
    Serial.println("EVENT TESTDONE");
  }
}

// Globale Nenn-Kennlinie (nur fuer Kalibrier-/Hold-Modus): Vout -> DAC-Spannung
float voutToVdac(float vout) {
  float vdac = (7.03 - vout) / 1.41;
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > 5.0) vdac = 5.0;
  return vdac;
}

// Reine (unkorrigierte) Vorsteuerung: Vout -> DAC-Spannung, ohne Ausgangs-Feinkalibrierung.
// Wird beim Kalibrier-Messen (CALOUT) genutzt, damit die Messung unabhaengig von outCal ist.
float voutToVdacChRaw(int channel, float vout) {
  float vdac = dacM[channel] * vout + dacB[channel]; 
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > 5.0) vdac = 5.0;
  return vdac;
}

// Kanalindividuelle, kalibrierte Vorsteuerung inkl. Ausgangs-Feinkalibrierung.
float voutToVdacCh(int channel, float vout) {
  float vc = outCalM[channel] * vout + outCalB[channel];   // Ausgangs-Feinkalibrierung
  if (vc < 0.0) vc = 0.0;
  if (vc > MAX_VOUT) vc = MAX_VOUT;
  return voutToVdacChRaw(channel, vc);
}

// Setzt die gewuenschte Lampen-Spannung an einem Kanal (Enable verwaltet der Aufrufer)
void setVout(int channel, float vout) {
  channelVoltage[channel] = vout;                                 // Sollwert merken
  float vdac = voutToVdacCh(channel, vout) + vdacTrim[channel];   // kalibrierte Vorsteuerung (+ opt. Trim)
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > 5.0) vdac = 5.0;
  setDAC(channel, vdac);
}

// Wie setVout(), aber OHNE die gelernte Korrektur (vdacTrim) - nur die Vorsteuerung.
// WICHTIG fuer die Rampen: vdacTrim wird im Haltepunkt (volle Spannung) gelernt und
// gilt nur DORT. Wuerde man es auch bei kleinen Sollwerten addieren, waere der
// Rampenanfang verschoben - die Rampe wuerde nicht bei 0 V starten, sondern
// sofort auf mehrere Volt springen (beobachtet: Start bei 2,83 V statt 0 V).
void setVoutRamp(int channel, float vout) {
  channelVoltage[channel] = vout;                 // Sollwert merken
  float vdac = voutToVdacCh(channel, vout);       // reine kalibrierte Vorsteuerung
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > 5.0) vdac = 5.0;
  setDAC(channel, vdac);
}

// true, wenn der Kanal gerade im eingeschwungenen Zustand geregelt werden soll
bool channelRegulating(int i) {
  if (testState == TEST_RUNNING) return slots[i].phase == S_HOLD;  // nur bei Vollspannung
  if (testState == TEST_IDLE)    return channelVoltage[i] > 0.0;   // manueller Betrieb
  return false;                                                    // TEST_DONE: aus
}

// Nachregelung: gemessene Spannung auf den Sollwert ziehen.
// Inverse Strecke (hoeheres vdac -> kleineres Vout) -> Korrektur mit Minus.
void regulate() {
  if (holdMode) return;   // im Kalibrier-/Halte-Modus nicht regeln
  for (int i = 0; i < numChannels; i++) {
    if (!channelRegulating(i)) continue;

    float error = channelVoltage[i] - voltageMeas[i];   // Soll - Ist
    if (fabs(error) < REG_DEADBAND) continue;           // nah genug -> einfrieren

    float step = REG_GAIN * error;                       // Korrekturschritt
    if (step >  REG_STEP_MAX) step =  REG_STEP_MAX;      // pro Zyklus begrenzen
    if (step < -REG_STEP_MAX) step = -REG_STEP_MAX;

    float base = voutToVdacCh(i, channelVoltage[i]);     // Vorsteuerung fuer den Sollwert
    float trim = vdacTrim[i] - step;                     // inverse Strecke -> Minus

    // --- Anti-Windup -------------------------------------------------------
    // Ist der Sollwert gar nicht erreichbar (z. B. 7,00 V = Hardware-Maximum,
    // real nur ~6,96 V), bleibt dauerhaft ein Restfehler. Ohne Begrenzung liefe
    // die Korrektur endlos weiter und "wickelte sich auf" bis TRIM_LIMIT (2 V),
    // obwohl der DAC laengst am Anschlag ist. Darum nur so viel Korrektur
    // zulassen, wie der DAC ueberhaupt noch stellen kann (Bereich 0...5 V).
    if (trim < -base)      trim = -base;          // sonst waere der DAC < 0 V
    if (trim > 5.0 - base) trim = 5.0 - base;     // sonst waere der DAC > 5 V
    if (trim >  TRIM_LIMIT) trim =  TRIM_LIMIT;   // zusaetzliche Sicherheitsgrenze
    if (trim < -TRIM_LIMIT) trim = -TRIM_LIMIT;
    vdacTrim[i] = trim;

    float vdac = base + vdacTrim[i];
    if (vdac < 0.0) vdac = 0.0;
    if (vdac > 5.0) vdac = 5.0;
    setDAC(i, vdac);
  }
}

// ================= SERIAL / BEFEHLS-PROTOKOLL =================
// Zeilenbasiert. Befehle gross/klein egal. Antworten: "OK ...", "ERR ...",
// Messdaten: "DATA ...", Ereignisse: "EVENT ...". 'HELP' listet alles.
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputString.trim();
      if (inputString.length() > 0) processCommand(inputString);
      inputString = "";
    } else if (c != '\r') {
      inputString += c;
    }
  }
}

// Modus-Presets laden
void setMode(bool eval) {
  evalMode = eval;
  if (eval) { rampDur = RAMP_MS_EVAL; onDur = ON_MS_EVAL; offDur = OFF_MS_EVAL; }
  else      { rampDur = RAMP_MS_REAL; onDur = ON_MS_REAL; offDur = OFF_MS_REAL; }
  Serial.print("OK MODE "); Serial.println(eval ? "EVAL" : "REAL");
}

// Einen Kanal manuell setzen (nur wenn kein Test laeuft), geregelt + kalibriert
void setChannelManual(int ch1, float v) {
  if (testState == TEST_RUNNING) { Serial.println("ERR test running"); return; }
  int ch = ch1 - 1;
  if (ch < 0 || ch >= numChannels) { Serial.println("ERR channel"); return; }
  if (v < 0) v = 0;
  if (v > MAX_VOUT) v = MAX_VOUT;

  holdMode = false;
  testState = TEST_IDLE;

  if (v <= 0.0) {
    digitalWrite(enablePins[ch], LOW);
    channelVoltage[ch] = 0;
    setDAC(ch, 0);
  } else {
    digitalWrite(enablePins[ch], HIGH);
    setVout(ch, v);
  }
  Serial.print("OK SETV "); Serial.print(ch1); Serial.print(' '); Serial.println(v, 3);
}

// Konfiguration ausgeben
void printCfg() {
  Serial.print("CFG mode=");     Serial.print(evalMode ? "EVAL" : "REAL");
  Serial.print(" ramp_s=");      Serial.print(rampDur / 1000.0, 1);
  Serial.print(" on_s=");        Serial.print(onDur / 1000.0, 1);
  Serial.print(" off_s=");       Serial.print(offDur / 1000.0, 1);
  Serial.print(" cycles=");      Serial.print(numCycles);
  Serial.print(" testv=");       Serial.print(testVoltage, 2);
  Serial.print(" channels=");    Serial.print(numChannels);
  Serial.print(" sensors=");     Serial.print(numSensors);
  Serial.print(" reg=");         Serial.print(useRegulation ? 1 : 0);
  Serial.print(" stream_ms=");   Serial.print(statusMs);
  Serial.print(" streamon=");    Serial.println(streamOn ? 1 : 0);
}

// Kompakte, maschinenlesbare Messdatenzeile (fuer GUI: Plots + Logging)
void printData() {
  Serial.print("DATA t="); Serial.print(millis());
  Serial.print(" state=");
  if (holdMode)                        Serial.print("CALIBRATION");  // manueller/Kalibrier-Modus (eindeutig gegenueber der Kanal-Phase S_HOLD)
  else if (testState == TEST_RUNNING)  Serial.print("RUNNING");
  else if (testState == TEST_DONE)     Serial.print("DONE");
  else                                 Serial.print("IDLE");

  int completed = 0;
  if (testState == TEST_RUNNING || testState == TEST_DONE) {
    completed = numCycles;
    for (int i = 0; i < numChannels; i++)
      if (slots[i].cycle < completed) completed = slots[i].cycle;
  }
  Serial.print(" cyc="); Serial.print(completed); Serial.print('/'); Serial.print(numCycles);

  Serial.print(" I=");
  for (int i = 0; i < numCurrent; i++) { if (i) Serial.print(','); Serial.print(lastCurrent[i], 3); }
  Serial.print(" U=");
  // Abgeschaltete Kanaele (Wandler aus) exakt 0,00 V melden -> kein Mess-Offset im Balken.
  for (int i = 0; i < numVoltage; i++) { if (i) Serial.print(','); Serial.print(channelVoltage[i] > 0.0f ? voltageMeas[i] : 0.0f, 3); }
  Serial.print(" T=");
  for (int i = 0; i < numSensors; i++) { if (i) Serial.print(','); Serial.print(lastTemp[i], 2); }
  Serial.print(" rpmR="); Serial.print(lastRpmR);
  Serial.print(" rpmL="); Serial.print(lastRpmL);

  Serial.print(" def=");
  bool first = true;
  for (int i = 0; i < numChannels; i++)
    if (lampDefect[i]) { if (!first) Serial.print(','); Serial.print(i + 1); first = false; }
  if (first) Serial.print('-');
  Serial.println();
}

// Alle Kanaele open-loop auf v legen mit ROHER Vorsteuerung (ohne Ausgangs-Feinkalibrierung).
// Genau der richtige Zustand, um mit dem Multimeter fuer die Kalibrierung zu messen.
void startCalOut(float v) {
  if (v < 0) v = 0;
  if (v > MAX_VOUT) v = MAX_VOUT;
  holdMode = true;
  testState = TEST_IDLE;
  fanFloorActive = false;
  for (int i = 0; i < numChannels; i++) {
    slots[i].phase = S_DONE;
    if (v <= 0.0) {
      digitalWrite(enablePins[i], LOW);
      channelVoltage[i] = 0;
      setDAC(i, 0);
    } else {
      digitalWrite(enablePins[i], HIGH);
      channelVoltage[i] = v;                 // fuer die Luefterregelung
      setDAC(i, voutToVdacChRaw(i, v));      // rohe Vorsteuerung, ohne outCal
    }
  }
  setLED(TEST_IDLE);
  Serial.print("OK CALOUT "); Serial.println(v, 3);
}

// Ausgangs-Feinkalibrierung dauerhaft im EEPROM ablegen / laden.
void calSaveEEPROM() {
  int a = 0;
  EEPROM.put(a, CAL_MAGIC);            a += sizeof(CAL_MAGIC);
  for (int i = 0; i < numChannels; i++) { EEPROM.put(a, outCalM[i]); a += sizeof(float); }
  for (int i = 0; i < numChannels; i++) { EEPROM.put(a, outCalB[i]); a += sizeof(float); }
}
void calLoadEEPROM() {
  int a = 0; uint32_t magic;
  EEPROM.get(a, magic);                a += sizeof(CAL_MAGIC);
  if (magic != CAL_MAGIC) return;      // noch nichts gespeichert -> Identitaet behalten
  for (int i = 0; i < numChannels; i++) { EEPROM.get(a, outCalM[i]); a += sizeof(float); }
  for (int i = 0; i < numChannels; i++) { EEPROM.get(a, outCalB[i]); a += sizeof(float); }
}

void printHelp() {
  Serial.println("Befehle (gross/klein egal):");
  Serial.println(" START | STOP | PING | GETCFG | HELP");
  Serial.println(" MODE EVAL|REAL   (Kurzform: EVAL / REAL)");
  Serial.println(" SETV <ch> <0-7>  (Kanal setzen)  | OFF <ch> | ALL <v> | OFFALL");
  Serial.println(" RAMP <s> | ONTIME <s> | OFFTIME <s> | CYCLES <n> | TESTV <0-7>");
  Serial.println(" GETI <ch> | GETU <ch> | GETT <n> | GETALL");
  Serial.println(" STREAM <ms> | STREAM OFF");
  Serial.println(" Kalibrierung: DAC <ch> <vdac> | SETALL <v> | RAW");
  Serial.println(" Ausgangskal.: CALOUT <v> | CALSET <ch> <m> <b> | CALGET | CALRESET | CALSAVE");
  Serial.println(" Anzeige: GUI 1 (PC-Steuerung) | GUI 0 (Standard-Startbildschirm)");
}

void processCommand(String s) {
  // Komma-Kurzform "ch,volt" weiterhin unterstuetzen
  if (s.indexOf(',') > 0 && isDigit(s.charAt(0))) {
    int ci = s.indexOf(',');
    setChannelManual(s.substring(0, ci).toInt(), s.substring(ci + 1).toFloat());
    return;
  }

  int sp = s.indexOf(' ');
  String cmd  = (sp < 0) ? s : s.substring(0, sp);
  String args = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();
  args.trim();

  if      (cmd == "PING" || cmd == "ID") Serial.println("W0609 Halogenlampentest Teensy4.1");
  else if (cmd == "HELP")    printHelp();
  else if (cmd == "GETCFG")  printCfg();
  else if (cmd == "START") {
    if (testState != TEST_RUNNING) { startTest(); Serial.println("OK START"); }
    else Serial.println("ERR running");
  }
  else if (cmd == "STOP")    { stopTest(); Serial.println("OK STOP"); }
  else if (cmd == "EVAL")    setMode(true);
  else if (cmd == "REAL")    setMode(false);
  else if (cmd == "MODE")    setMode(args.equalsIgnoreCase("EVAL"));
  else if (cmd == "SETV") {
    int a = args.indexOf(' ');
    if (a < 0) { Serial.println("ERR SETV <ch> <v>"); return; }
    setChannelManual(args.substring(0, a).toInt(), args.substring(a + 1).toFloat());
  }
  else if (cmd == "OFF")     setChannelManual(args.toInt(), 0);
  else if (cmd == "ALL")     setAllRegulated(args.toFloat());
  else if (cmd == "OFFALL")  setAllRegulated(0);
  else if (cmd == "SETALL")  startHold(args.toFloat());
  else if (cmd == "RAW")     printRaw();
  else if (cmd == "CALOUT")  startCalOut(args.toFloat());
  else if (cmd == "CALSET") {
    // CALSET <ch> <m> <b>  -> Ausgangs-Feinkalibrierung eines Kanals setzen
    int a1 = args.indexOf(' ');
    int a2 = (a1 < 0) ? -1 : args.indexOf(' ', a1 + 1);
    if (a1 < 0 || a2 < 0) { Serial.println("ERR CALSET <ch> <m> <b>"); return; }
    int ch = args.substring(0, a1).toInt();
    float m = args.substring(a1 + 1, a2).toFloat();
    float b = args.substring(a2 + 1).toFloat();
    if (ch >= 1 && ch <= numChannels) {
      outCalM[ch - 1] = m; outCalB[ch - 1] = b;
      Serial.print("OK CALSET "); Serial.print(ch); Serial.print(' ');
      Serial.print(m, 5); Serial.print(' '); Serial.println(b, 5);
    } else Serial.println("ERR ch");
  }
  else if (cmd == "CALRESET") {
    for (int i = 0; i < numChannels; i++) { outCalM[i] = 1.0; outCalB[i] = 0.0; }
    Serial.println("OK CALRESET");
  }
  else if (cmd == "CALSAVE") { calSaveEEPROM(); Serial.println("OK CALSAVE"); }
  else if (cmd == "GUI") {
    guiConnected = (args.toInt() != 0);
    if (screen == SCR_START) drawIdle();   // Idle-Anzeige sofort umschalten
    Serial.print("OK GUI "); Serial.println(guiConnected ? 1 : 0);
  }
  else if (cmd == "CALGET") {
    Serial.print("CAL M=");
    for (int i = 0; i < numChannels; i++) { if (i) Serial.print(','); Serial.print(outCalM[i], 5); }
    Serial.print(" B=");
    for (int i = 0; i < numChannels; i++) { if (i) Serial.print(','); Serial.print(outCalB[i], 5); }
    Serial.println();
  }
  else if (cmd == "DAC") {
    int a = args.indexOf(' ');
    if (a < 0) { Serial.println("ERR DAC <ch> <vdac>"); return; }
    setDacDirect(args.substring(0, a).toInt(), args.substring(a + 1).toFloat());
  }
  else if (cmd == "RAMP")    { rampDur = (unsigned long)(args.toFloat() * 1000.0); Serial.print("OK RAMP ");    Serial.println(rampDur); }
  else if (cmd == "ONTIME")  { onDur   = (unsigned long)(args.toFloat() * 1000.0); Serial.print("OK ONTIME ");  Serial.println(onDur); }
  else if (cmd == "OFFTIME") { offDur  = (unsigned long)(args.toFloat() * 1000.0); Serial.print("OK OFFTIME "); Serial.println(offDur); }
  else if (cmd == "CYCLES")  { int n = args.toInt(); if (n < 1) n = 1; if (n > 99) n = 99; numCycles = n; Serial.print("OK CYCLES "); Serial.println(numCycles); }
  else if (cmd == "TESTV")   { float v = args.toFloat(); if (v < 0) v = 0; if (v > MAX_VOUT) v = MAX_VOUT; testVoltage = v; for (int i=0;i<numChannels;i++) testVoltageCh[i] = v; Serial.print("OK TESTV "); Serial.println(testVoltage, 3); }
  else if (cmd == "TESTVCH") { int a = args.indexOf(' '); if (a < 0) { Serial.println("ERR TESTVCH <ch> <v>"); return; } int ch = args.substring(0, a).toInt(); float v = args.substring(a + 1).toFloat(); if (v < 0) v = 0; if (v > MAX_VOUT) v = MAX_VOUT; if (ch >= 1 && ch <= numChannels) { testVoltageCh[ch-1] = v; Serial.print("OK TESTVCH "); Serial.print(ch); Serial.print(' '); Serial.println(v, 3); } else Serial.println("ERR ch"); }
  else if (cmd == "GETI")    { int ch = args.toInt(); if (ch >= 1 && ch <= numChannels) { Serial.print("I"); Serial.print(ch); Serial.print('='); Serial.println(lastCurrent[ch-1], 3); } else Serial.println("ERR ch"); }
  else if (cmd == "GETU")    { int ch = args.toInt(); if (ch >= 1 && ch <= numChannels) { Serial.print("U"); Serial.print(ch); Serial.print('='); Serial.println(voltageMeas[ch-1], 3); } else Serial.println("ERR ch"); }
  else if (cmd == "GETT")    { int n  = args.toInt(); if (n  >= 1 && n  <= numSensors)  { Serial.print("T"); Serial.print(n);  Serial.print('='); Serial.println(lastTemp[n-1], 2); }  else Serial.println("ERR sensor"); }
  else if (cmd == "GETALL")  printData();
  else if (cmd == "STREAM") {
    if (args.equalsIgnoreCase("OFF")) { streamOn = false; Serial.println("OK STREAM OFF"); }
    else { unsigned long ms = (unsigned long)args.toInt(); if (ms < 100) ms = 100; statusMs = ms; streamOn = true; Serial.print("OK STREAM "); Serial.println(statusMs); }
  }
  else { Serial.print("ERR unknown: "); Serial.println(cmd); }
}

// ================= DAC =================
void setDAC(uint8_t channel, float voltage) {
  uint16_t value = (uint16_t)((voltage / VREF) * DAC_MAX);

  Wire.beginTransmission(DAC_ADDR);
  uint8_t command = 0x10 | (channel & 0x0F);
  Wire.write(command);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

// ================= FAN CONTROL =================
void updateFanControl() {
  // hoechste anliegende Lampenspannung suchen (gilt fuer Test, manuell und Hold)
  float maxVout = 0;
  for (int i=0;i<numChannels;i++) {
    if (channelVoltage[i] > maxVout) {
      maxVout = channelVoltage[i];
    }
  }

  // linear: 0V -> 0%, 5V -> 100% (>5V, z.B. 7V, ebenfalls 100%)
  int pwmPercent = (int)(maxVout / FAN_FULL_VOLT * 100.0 + 0.5);
  if (pwmPercent > 100) pwmPercent = 100;
  if (pwmPercent < 0)   pwmPercent = 0;

  // Mindestdrehzahl in der Abkuehlphase (nur waehrend eines laufenden Tests)
  if (testState == TEST_RUNNING && fanFloorActive && pwmPercent < FAN_MIN_COOL) {
    pwmPercent = FAN_MIN_COOL;
  }

  // Nachlauf nach Testende/Stop: ~2 min mind. 30% (Restwaerme abfuehren)
  if (millis() < fanRunoutUntil && pwmPercent < FAN_RUNOUT_PCT) {
    pwmPercent = FAN_RUNOUT_PCT;
  }

  int pwmValue = map(pwmPercent, 0, 100, 0, 255);
  analogWrite(pinPWM, pwmValue);
}

// ================= TACHO =================
void tachISR1() {
  unsigned long now = micros();
  if (now - lastPulseTime1 > 500) {
    pulsesFan1++;
    lastPulseTime1 = now;
  }
}

void tachISR2() {
  unsigned long now = micros();
  if (now - lastPulseTime2 > 500) {
    pulsesFan2++;
    lastPulseTime2 = now;
  }
}

// ================= MULTIPLEXER / STROM =================
void selectMuxChannel(int ch) {
  digitalWrite(muxS0, (ch >> 0) & 0x01);
  digitalWrite(muxS1, (ch >> 1) & 0x01);
  digitalWrite(muxS2, (ch >> 2) & 0x01);
  digitalWrite(muxS3, (ch >> 3) & 0x01);
}

float readCurrent(int ch) {
  selectMuxChannel(ch);
  delayMicroseconds(150);                 // Mux/ADC einschwingen lassen
  analogRead(muxSigI);                     // Dummy-Read: Sample&Hold laden
  int raw = analogRead(muxSigI);
  float vADC = (float)raw / ADC_MAX * ADC_REF;
  float current = vADC / (SHUNT_OHM * INA_GAIN);
  return current;
}

// Rohe Spannungsmessung (nominaler Teiler, ohne Kalibrierung)
float readVoltageRaw(int ch) {
  selectMuxChannel(ch);
  delayMicroseconds(150);                 // Mux/ADC einschwingen lassen
  analogRead(muxSigU);                      // Dummy-Read: Sample&Hold laden
  int raw = analogRead(muxSigU);
  float vADC = (float)raw / ADC_MAX * ADC_REF;
  return vADC * VDIV_FACTOR;               // Spannungsteiler zurueckrechnen
}

// Kalibrierte Spannungsmessung: U_korr = vCalA * U_roh + vCalB
float readVoltage(int ch) {
  return vCalA[ch] * readVoltageRaw(ch) + vCalB[ch];
}

// ================= STATUSAUSGABE (1 s) =================
void updateStatus() {
  unsigned long now = millis();
  if (now - lastStatusTime < statusMs) return;
  lastStatusTime = now;

  // ---- Drehzahlen (2 Pulse pro Umdrehung) ----
  // Eigenes ~1-s-Fenster (siehe RPM_WINDOW_MS): Pulse sammeln sich ueber ~1 s an,
  // die Drehzahl wird nur einmal pro Sekunde neu berechnet. Zwischen den Fenstern
  // wird der letzte Wert gehalten -> der schnelle 100-ms-Stream zeigt eine ruhige,
  // stabile Drehzahl statt +-300 U/min zu springen.
  if (now - lastRpmTime >= RPM_WINDOW_MS) {
    unsigned long dtr = now - lastRpmTime;
    lastRpmTime = now;
    noInterrupts();
    unsigned long c1 = pulsesFan1;
    unsigned long c2 = pulsesFan2;
    pulsesFan1 = 0;
    pulsesFan2 = 0;
    interrupts();
    lastRpmR = (int)(((float)c1 / (dtr / 1000.0)) * 60.0 / 2.0);   // Fan1 = rechts
    lastRpmL = (int)(((float)c2 / (dtr / 1000.0)) * 60.0 / 2.0);   // Fan2 = links
  }

  // ---- Stroeme + Fehlererkennung (Strom < Schwelle im Haltepunkt = defekt) ----
  for (int i = 0; i < numCurrent; i++) {
    lastCurrent[i] = readCurrent(i);
    if (testState == TEST_RUNNING && slots[i].phase == S_HOLD && lastCurrent[i] < CURRENT_MIN) {
      if (!lampDefect[i]) {                 // neu erkannt -> Ereignis melden
        lampDefect[i] = true;
        Serial.print("EVENT DEFECT "); Serial.println(i + 1);
        // defekte Lampe: Spannung mit Rampe herunterfahren und Kanal stilllegen
        slots[i].phase      = S_RAMP_DOWN;
        slots[i].phaseStart = millis();
      }
    }
  }

  // ---- Spannungen (auch fuer die Regelung) ----
  for (int i = 0; i < numVoltage; i++) voltageMeas[i] = readVoltage(i);

  // ---- Temperaturen ----
  for (int i = 0; i < numSensors; i++) lastTemp[i] = readTemp(sensors[i]);

  // ---- Spannung pro Kanal nachregeln (optional) ----
  if (useRegulation) regulate();

  // ---- strukturierte Datenzeile an die GUI ----
  if (streamOn) printData();
}

float readTemp(uint8_t addr) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);
  Wire1.endTransmission();

  Wire1.requestFrom(addr, (uint8_t)2);

  if (Wire1.available() < 2) return -999;

  uint8_t msb = Wire1.read();
  uint8_t lsb = Wire1.read();

  int16_t raw = (msb << 8) | lsb;
  raw >>= 4;

  if (raw & 0x800) raw |= 0xF000;

  return raw * 0.0625;
}

// ================= DISPLAY-FUNKTIONEN =================

// Startlogo (Knauer): glattes Chromatogramm (4 Gauss-Peaks) + Schriftzug, zentriert
void drawLogo() {
  tft.fillScreen(ILI9341_BLACK);
  uint16_t c = KNAUER_BLUE;

  // Schriftbreite messen, um das ganze Logo (Peaks + Text) zu zentrieren
  tft.setFont(&FreeSansBold24pt7b);
  int16_t bx, by; uint16_t tw, th;
  tft.getTextBounds("KNAUER", 0, 0, &bx, &by, &tw, &th);

  const int peaksW = 64;          // Breite des Chromatogramm-Blocks
  const int gap    = 26;          // Abstand Peaks <-> Schrift
  int total = peaksW + gap + (int)tw;
  int x0 = (320 - total) / 2;     // linke Kante des Logos
  int baseY = 156;                // gemeinsame Unterkante: Peaks UND Schrift
  int lineY = baseY + 24;         // Achsenlinie deutlich darunter (wie im Bild)

  // --- Chromatogramm (Laplace-Peaks: spitze Maxima, flache Minima) ---
  const float ctr[4] = {10, 21, 32, 52};   // Peakzentren (1-3 dicht, 4 abgesetzt)
  const float hgt[4] = {80, 100, 62, 38};  // Peakhoehen
  const float sig    = 2.2;                // klein -> spitze Gipfel, flachere Taeler

  int prevX = 0, prevY = 0;
  for (int u = 0; u <= peaksW; u++) {
    float env = 0;
    for (int k = 0; k < 4; k++) {
      float d = fabs((float)u - ctr[k]);
      env += hgt[k] * expf(-d / sig);        // spitzer Gipfel, flacher Auslauf
    }
    int px = x0 + u;
    int py = baseY - (int)env;
    if (u > 0) {
      tft.drawLine(prevX, prevY,     px, py,     c);
      tft.drawLine(prevX, prevY + 1, px, py + 1, c);  // 2 px dick
    }
    prevX = px; prevY = py;
  }

  // --- durchgehende Achsenlinie: ab dem 1. Strich bis unter KNAUER ---
  int axisL = x0;
  int axisR = x0 + total + 2;
  tft.drawFastHLine(axisL, lineY,     axisR - axisL, c);
  tft.drawFastHLine(axisL, lineY + 1, axisR - axisL, c);

  // --- vertikale Striche: Beginn / Mitte / Ende des Chromatogramms, nach oben ---
  int tickH = 8;
  int ticks[3] = { x0, x0 + 22, x0 + peaksW };
  for (int t = 0; t < 3; t++) {
    tft.drawFastVLine(ticks[t],     lineY - tickH, tickH, c);
    tft.drawFastVLine(ticks[t] + 1, lineY - tickH, tickH, c);
  }

  // kurzer vertikaler Abschluss am rechten Ende der Linie
  tft.drawFastVLine(axisR - 1, lineY - tickH, tickH, c);
  tft.drawFastVLine(axisR - 2, lineY - tickH, tickH, c);

  // --- Schriftzug KNAUER (Unterkante buendig mit dem Chromatogramm) ---
  tft.setTextColor(c);
  tft.setCursor(x0 + peaksW + gap, baseY);
  tft.print("KNAUER");
}

// Text horizontal zentriert ausgeben (Display 320 px breit), y = Grundlinie
void printCentered(const char* text, int y) {
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  tft.setCursor((320 - (int)w) / 2 - x1, y);
  tft.print(text);
}

// "Taster drücken" zentriert, mit echtem ü (Umlautpunkte von Hand ueber dem u)
void drawTasterHint(int y) {
  tft.setFont(&FreeSansBold9pt7b);

  // Breite als "Taster drucken" fuer die Zentrierung
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds("Taster drucken", 0, y, &x1, &y1, &w, &h);
  tft.setCursor((320 - (int)w) / 2 - x1, y);

  tft.print("Taster dr");
  int uStart = tft.getCursorX();
  tft.print("u");
  int uEnd = tft.getCursorX();
  tft.print("cken");

  // Umlautpunkte ueber dem 'u'
  int cx = (uStart + uEnd) / 2;
  int dotY = y - 13;
  tft.fillRect(cx - 3, dotY, 2, 2, ILI9341_WHITE);
  tft.fillRect(cx + 1, dotY, 2, 2, ILI9341_WHITE);
}

// Schlichter Startbildschirm: Geraetename + Halogenlampentest + Hinweis
void drawStartScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);

  tft.setFont(&FreeSansBold24pt7b);
  printCentered("W0609", 80);

  tft.setFont(&FreeSansBold12pt7b);
  printCentered("Halogenlampentest", 120);

  tft.setFont(&FreeSansBold9pt7b);
  printCentered("Zum Starten des Tests", 180);
  drawTasterHint(205);
}

// Glühbirnen-Symbol (wie das GUI-/Taskleisten-Icon): gelber Kolben, Wendel, Sockel
// Linie ~2-3 px dick (GFX kann nur 1 px)
void thickLine(int x0, int y0, int x1, int y1, uint16_t c) {
  tft.drawLine(x0, y0, x1, y1, c);
  tft.drawLine(x0 + 1, y0, x1 + 1, y1, c);
  tft.drawLine(x0, y0 + 1, x1, y1 + 1, c);
}

// Tangenten-Berührpunkt vom Punkt (px,py) an den Kreis (cx,cy,r); side>0 = rechter, <0 = linker
void tangentPoint(float px, float py, float cx, float cy, float r, int side, float *tx, float *ty) {
  float dvx = cx - px, dvy = cy - py, dist = sqrt(dvx * dvx + dvy * dvy);
  float th = asin(r / dist), phi = atan2(dvy, dvx), L = sqrt(dist * dist - r * r);
  float bx = 0, by = 0; bool have = false;
  for (int s = -1; s <= 1; s += 2) {
    float ang = phi + s * th, x = px + L * cos(ang), y = py + L * sin(ang);
    if (!have || (side > 0 && x > bx) || (side < 0 && x < bx)) { bx = x; by = y; have = true; }
  }
  *tx = bx; *ty = by;
}

// Glühbirne: geschlossener Birnen-Umriss (Taille), Glaskörper komplett gelb,
// Wolframdraht (zwei Beinchen mit Schlaufen), Sockel.
void drawBulb(int cx, int cy, int Rg) {
  uint16_t amber = tft.color565(245, 200, 66);
  uint16_t dark  = tft.color565(30, 32, 38);
  uint16_t out   = ILI9341_WHITE;

  float nw = Rg * 0.36, ny = cy + Rg * 1.45;     // schmaler Hals unter dem Kopf

  // Tangenten-Berührpunkte (Silhouette + Fläche)
  float Trx, Try, Tlx, Tly;
  tangentPoint(cx + nw, ny, cx, cy, Rg, +1, &Trx, &Try);
  tangentPoint(cx - nw, ny, cx, cy, Rg, -1, &Tlx, &Tly);

  // kompletter Glaskörper gelb: Kopfkreis + Schulterfläche bis zum Hals
  tft.fillCircle(cx, cy, Rg, amber);
  tft.fillTriangle((int)Tlx, (int)Tly, (int)Trx, (int)Try, cx + (int)nw, (int)ny, amber);
  tft.fillTriangle((int)Tlx, (int)Tly, cx + (int)nw, (int)ny, cx - (int)nw, (int)ny, amber);

  // Wolframdraht nach Vorlage: zwei senkrechte Beine (~2/3 Kopfhoehe), oben je eine
  // kleine kreisrunde Schlaufe nach aussen, verbunden durch einen breiten, oben runden
  // Bogen (Scheitel ~10-15% unter der Kopfspitze); oberes Drittel bleibt frei.
  {
    float loopR = Rg * 0.15, loopCX = Rg * 0.34, legX = loopCX - loopR;
    float yC = cy + Rg * 0.24;                     // ganzer Faden als Einheit, final tiefer
    float yt = yC - loopR, peak = cy - Rg * 0.06;
    int legBot = (int)(cy + Rg * 1.34);            // gleiche Beinlaenge, nur tiefer
    for (int s = -1; s <= 1; s += 2) {
      int lx = cx + (int)(s * legX);
      thickLine(lx, legBot, lx, (int)yC, dark);             // Bein bis Schlaufen-Hoehe
      int hx = cx + (int)(s * loopCX);
      for (int rr = (int)loopR - 2; rr <= (int)loopR; rr++) // kleine Schlaufe nach aussen
        if (rr > 0) tft.drawCircle(hx, (int)yC, rr, dark);
    }
    // breiter, oben runder Verbindungsbogen (Parabel, waagerechte Tangente in der Mitte)
    int fpx = cx - (int)loopCX, fpy = (int)yt;
    for (int i = 1; i <= 48; i++) {
      float t = -1.0 + 2.0 * i / 48.0;
      int x = (int)(cx + t * loopCX);
      int y = (int)(peak + (yt - peak) * t * t);
      thickLine(fpx, fpy, x, y, dark);
      fpx = x; fpy = y;
    }
  }

  // geschlossene Silhouette (weiß): Hals -> Bogen über den Kopf -> Hals
  float aR = atan2(Try - cy, Trx - cx), aL = atan2(Tly - cy, Tlx - cx);
  int prevx = cx + (int)nw, prevy = (int)(ny + Rg * 0.12);
  thickLine(prevx, prevy, cx + (int)nw, (int)ny, out); prevx = cx + (int)nw; prevy = (int)ny;
  const int steps = 48;
  for (int i = 0; i <= steps; i++) {
    float a = aR + (aL - 2.0 * PI - aR) * i / steps;
    int x = cx + (int)(Rg * cos(a)), y = cy + (int)(Rg * sin(a));
    thickLine(prevx, prevy, x, y, out); prevx = x; prevy = y;
  }
  thickLine(prevx, prevy, cx - (int)nw, (int)ny, out);
  thickLine(cx - (int)nw, (int)ny, cx - (int)nw, (int)(ny + Rg * 0.12), out);

  // Sockel unter dem Hals: drei helle Ringe + Kontaktfuß
  int by = (int)(ny + Rg * 0.16), bw = (int)(Rg * 0.64);
  for (int i = 0; i < 3; i++)
    tft.fillRoundRect(cx - bw / 2, by + i * 7, bw, 5, 2, ILI9341_LIGHTGREY);
  tft.fillRoundRect(cx - 6, by + 21, 12, 7, 3, ILI9341_DARKGREY);
}

// Bildschirm, wenn die Software (GUI) angeschlossen ist
void drawGuiScreen() {
  tft.fillScreen(ILI9341_BLACK);

  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(ILI9341_WHITE);
  printCentered("W0609 Halogenlampentest", 20);

  drawBulb(160, 80, 35);        // Symbol 20% kleiner (Rg 44 -> 35), mittig oben

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(tft.color565(245, 200, 66));
  printCentered("PC-Steuerung aktiv", 206);

  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(ILI9341_WHITE);
  printCentered("Bedienung per Software", 230);
}

// passende Idle-Anzeige waehlen (mit/ohne GUI)
void drawIdle() {
  if (guiConnected) drawGuiScreen();
  else              drawStartScreen();
}

// Grundgeruest der Testansicht (Rahmen + Linien); Zellen/Balken fuellt updateTestScreen
void drawTestLayout() {
  tft.fillScreen(ILI9341_BLACK);                                    // Rand schwarz
  tft.fillRect(TBL_X-1, TBL_Y-1, TBL_W+2, TBL_H+2, ILI9341_WHITE);  // Tabellenflaeche weiss

  tft.drawRect(TBL_X,   TBL_Y,   TBL_W,   TBL_H,   ILI9341_BLACK);
  tft.drawRect(TBL_X-1, TBL_Y-1, TBL_W+2, TBL_H+2, ILI9341_BLACK);

  // Trennlinie nach der Cycle-Zeile
  tft.drawLine(TBL_X, TBL_Y + TBL_ROWH + 5, TBL_X + TBL_W, TBL_Y + TBL_ROWH + 5, ILI9341_BLACK);
  tft.drawLine(TBL_X, TBL_Y + TBL_ROWH + 6, TBL_X + TBL_W, TBL_Y + TBL_ROWH + 6, ILI9341_BLACK);

  // weitere Zeilentrenner
  for (int i = 2; i < TBL_ROWS; i++) {
    tft.drawLine(TBL_X, TBL_Y + i*TBL_ROWH + 5, TBL_X + TBL_W, TBL_Y + i*TBL_ROWH + 5, ILI9341_BLACK);
    tft.drawLine(TBL_X, TBL_Y + i*TBL_ROWH + 6, TBL_X + TBL_W, TBL_Y + i*TBL_ROWH + 6, ILI9341_BLACK);
  }
  // Spaltentrenner (ab Zeile 1)
  for (int i = 1; i < TBL_COLS; i++) {
    tft.drawLine(TBL_X + i*TBL_COLW,   TBL_Y + TBL_ROWH, TBL_X + i*TBL_COLW,   TBL_Y + TBL_H, ILI9341_BLACK);
    tft.drawLine(TBL_X + i*TBL_COLW+1, TBL_Y + TBL_ROWH, TBL_X + i*TBL_COLW+1, TBL_Y + TBL_H, ILI9341_BLACK);
  }

  // Lampen-Cache invalidieren -> updateTestScreen faerbt die Zellen
  for (int i = 0; i < numChannels; i++) lampDrawnColor[i] = 0xFFFF;

  // leeren Fortschrittsbalken vorbereiten (Markierungen + Label)
  barLastPx = 0;
  drawCycleTicks();
  drawCycleLabel();
}

// Eine Lampenzelle einfaerben + beschriften (lamp 1-basiert)
void drawLampCell(int lamp, uint16_t color) {
  int row = (lamp - 1) / TBL_COLS + 1;
  int col = (lamp - 1) % TBL_COLS;

  int xpos = TBL_X + col * TBL_COLW + 1;
  int ypos = TBL_Y + row * TBL_ROWH + 7;
  int boxW = TBL_COLW - 1;
  int boxH = TBL_ROWH - 2;

  if (lamp % 2 == 0) {
    tft.fillRect(xpos + 1, ypos, boxW - 2, boxH, color);
  } else {
    tft.fillRect(xpos, ypos, boxW, boxH, color);
  }

  // Rahmen erneuern
  tft.drawRect(TBL_X,   TBL_Y,   TBL_W,   TBL_H,   ILI9341_BLACK);
  tft.drawRect(TBL_X-1, TBL_Y-1, TBL_W+2, TBL_H+2, ILI9341_BLACK);

  // Beschriftung (schwarz, 12pt - wie im Original)
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(TBL_X + col * TBL_COLW + 33, TBL_Y + row * TBL_ROWH + TBL_ROWH/2 + 13);
  tft.print("Lamp " + String(lamp));
}

// Zyklus-Grenzmarken (numCycles Abschnitte) zeichnen
void drawCycleTicks() {
  if (numCycles < 1) return;
  for (int k = 1; k < numCycles; k++) {
    int tickX = TBL_X + (int)((long)TBL_W * k / numCycles);
    tft.drawLine(tickX,   TBL_Y,      tickX,   TBL_Y + 5,  ILI9341_BLACK);
    tft.drawLine(tickX+1, TBL_Y,      tickX+1, TBL_Y + 5,  ILI9341_BLACK);
    tft.drawLine(tickX,   TBL_Y + 35, tickX,   TBL_Y + 40, ILI9341_BLACK);
    tft.drawLine(tickX+1, TBL_Y + 36, tickX+1, TBL_Y + 41, ILI9341_BLACK);
  }
}

// "Cycle"-Beschriftung in der Balkenzeile (schwarz, 12pt)
void drawCycleLabel() {
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(TBL_X + 115, TBL_Y + 3 + TBL_ROWH/2 + 8);
  tft.print("Cycle");
}

// Dynamische Aktualisierung der Testansicht (nur bei Aenderung neu zeichnen)
void updateTestScreen() {
  // Lampenfarben
  for (int i = 0; i < numChannels; i++) {
    uint16_t col;
    if      (lampDefect[i])          col = ILI9341_RED;     // Fehler (gelatcht)
    else if (testState == TEST_DONE) col = ILI9341_GREEN;   // fehlerfrei beendet
    else                             col = ILI9341_BLUE;    // laeuft, kein Fehler

    if (col != lampDrawnColor[i]) {
      drawLampCell(i + 1, col);
      lampDrawnColor[i] = col;
    }
  }

  // Fortschrittsbalken: fliessend ueber die erwartete Gesamtdauer
  unsigned long now = millis();
  if (now - lastBarUpdate >= 60) {     // ~16 Updates/s -> fluessig
    lastBarUpdate = now;

    float progress;
    if (testState == TEST_DONE) progress = 1.0;
    else progress = (float)(now - testStartMs) / (float)testTotalMs;
    if (progress < 0) progress = 0;
    if (progress > 1) progress = 1;

    const int innerW = TBL_W - 2;
    int targetPx = (int)(progress * innerW + 0.5);

    if (targetPx > barLastPx) {
      // nur den neu hinzugekommenen Streifen fuellen -> waechst fluessig, kein Flackern
      tft.fillRect(TBL_X + 1 + barLastPx, TBL_Y + 1, targetPx - barLastPx, 40, ILI9341_GREEN);
      drawCycleTicks();                         // Zyklus-Grenzen wieder sichtbar
      if (targetPx >= 104 && barLastPx <= 190)  // Label neu, sobald der Balken es erreicht
        drawCycleLabel();
      barLastPx = targetPx;
    }
  }
}

// Bildschirm-Statemachine (im loop aufgerufen)
void updateDisplay() {
  switch (screen) {
    case SCR_LOGO:
      if (millis() - screenStart >= LOGO_MS) {
        screen = SCR_START;
        drawIdle();
      }
      break;
    case SCR_START:
      break;   // statisch
    case SCR_TEST:
      updateTestScreen();
      break;
  }
}
