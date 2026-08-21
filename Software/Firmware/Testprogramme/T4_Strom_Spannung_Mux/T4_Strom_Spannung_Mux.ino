/*
  T4 - Strom- und Spannungsmessung ueber die Multiplexer
  ======================================================

  Erstellt: 30.06.2026 (Di)

  Liest fuer alle zehn Kanaele Strom und Spannung ueber die beiden
  Analog-Multiplexer aus und gibt Rohwerte und umgerechnete Werte aus.

  Adressierung : muxS0..S3 = Pin 23 / 22 / 21 / 20  (4 Bit, Kanal 0..9)
  Messeingaenge: Strom   -> Pin 14
                 Spannung-> Pin 15
  ADC          : 12 Bit, Referenz 3,3 V, 32-fache Hardware-Mittelung

  Strommessung : Shunt 75 mOhm, Messverstaerker INA190A1 (25 V/V)
                 I = U_ADC / (0,075 * 25)
  Spannungsmess: Teiler 11,3k / 10k  ->  Faktor 2,13

  Ohne angeschlossene Last sollten alle Stroeme nahe 0 A liegen. Zum Pruefen
  der Spannungsmessung einen Kanal mit T1/T2 auf eine bekannte Spannung legen
  und den Wert mit dem Multimeter vergleichen.

  Ausgabe: serieller Monitor, 115200 Baud
  Befehle: R  Rohwerte (ADC-Counts) mit ausgeben
           M  nur Messwerte (Standard)
*/

const int muxS0 = 23, muxS1 = 22, muxS2 = 21, muxS3 = 20;
const int muxSigI = 14;      // Ausgang Strom-Mux
const int muxSigU = 15;      // Ausgang Spannungs-Mux

const int   ADC_MAX  = 4095;
const float ADC_REF  = 3.3;
const float SHUNT_OHM = 0.075;
const float INA_GAIN  = 25.0;
const float VDIV_FACTOR = (11300.0 + 10000.0) / 10000.0;   // = 2,13

bool zeigeRoh = false;

void selectMuxChannel(int ch) {
  digitalWrite(muxS0, (ch >> 0) & 0x01);
  digitalWrite(muxS1, (ch >> 1) & 0x01);
  digitalWrite(muxS2, (ch >> 2) & 0x01);
  digitalWrite(muxS3, (ch >> 3) & 0x01);
}

int readRaw(int ch, int pin) {
  selectMuxChannel(ch);
  delayMicroseconds(150);      // Mux und ADC einschwingen lassen
  analogRead(pin);             // Dummy-Read: Sample&Hold laden
  return analogRead(pin);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(muxS0, OUTPUT); pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT); pinMode(muxS3, OUTPUT);
  selectMuxChannel(0);

  analogReadResolution(12);
  analogReadAveraging(32);

  Serial.println();
  Serial.println("=== T4: Strom- und Spannungsmessung ueber Multiplexer ===");
  Serial.println("Befehle: R = Rohwerte anzeigen, M = nur Messwerte");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();
    if (s == "R") { zeigeRoh = true;  Serial.println(">> Rohwerte an"); }
    if (s == "M") { zeigeRoh = false; Serial.println(">> Rohwerte aus"); }
  }

  Serial.println("Kanal |   I [A]  |   U [V]  " + String(zeigeRoh ? "|  ADC_I  ADC_U" : ""));
  Serial.println("------+----------+----------" + String(zeigeRoh ? "+--------------" : ""));
  for (int ch = 0; ch < 10; ch++) {
    int rawI = readRaw(ch, muxSigI);
    int rawU = readRaw(ch, muxSigU);
    float vI = (float)rawI / ADC_MAX * ADC_REF;
    float vU = (float)rawU / ADC_MAX * ADC_REF;
    float strom = vI / (SHUNT_OHM * INA_GAIN);
    float spann = vU * VDIV_FACTOR;

    Serial.print("  ");
    if (ch + 1 < 10) Serial.print(" ");
    Serial.print(ch + 1);
    Serial.print("   |  ");
    Serial.print(strom, 3);
    Serial.print("   |  ");
    Serial.print(spann, 3);
    if (zeigeRoh) {
      Serial.print("   |  "); Serial.print(rawI);
      Serial.print("    "); Serial.print(rawU);
    }
    Serial.println();
  }
  Serial.println();
  delay(2000);
}
