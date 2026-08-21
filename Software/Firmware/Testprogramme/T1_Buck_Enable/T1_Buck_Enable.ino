/*
  T1 - Spannungsversorgung und Buck-Wandler (Enable-Signale)
  ==========================================================

  Erstellt: 25.06.2026 (Do)

  Prueft die zehn Buck-Wandler und ihre Enable-Leitungen.

  VORSICHT: Die Regelstrecke ist INVERTIEREND. Ein Kanal, dessen DAC auf 0 V
  steht, liefert die MAXIMALE Ausgangsspannung (~7 V). Dieses Programm setzt
  den DAC deshalb beim Start auf den Wert fuer 0 V Ausgangsspannung, bevor
  ueberhaupt ein Enable freigegeben wird. Vor dem ersten Test die Lampen
  abklemmen und mit dem Multimeter messen.

  Aufbau : Leiterplatte am Labornetzteil, Teensy per USB am Rechner.
           VIN/VUSB am Teensy muss aufgetrennt sein!
  Ausgabe: serieller Monitor, 115200 Baud

  Befehle:
    E <1-10>   Kanal einschalten (Enable HIGH)
    A <1-10>   Kanal ausschalten (Enable LOW)
    ALLEAUS    alle Kanaele aus
    P          Pin-Test: schaltet jedes Enable nacheinander 1 s ein
    ?          Hilfe
*/

#include <Wire.h>

const int numChannels = 10;
int enablePins[numChannels] = {0,1,2,3,4,5,6,7,8,9};

#define DAC_ADDR 0x0F
const float VREF = 5.0;
const uint16_t DAC_MAX = 65535;

// Nenn-Kennlinie: hoehere DAC-Spannung -> kleinere Ausgangsspannung
float voutToVdac(float vout) {
  float v = (7.03 - vout) / 1.41;
  if (v < 0) v = 0;
  if (v > 5) v = 5;
  return v;
}

void setDAC(uint8_t ch, float voltage) {
  uint16_t value = (uint16_t)((voltage / VREF) * DAC_MAX);
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x10 | (ch & 0x0F));
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

void alleAus() {
  for (int i = 0; i < numChannels; i++) digitalWrite(enablePins[i], LOW);
  Serial.println("Alle Enable-Signale LOW.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  for (int i = 0; i < numChannels; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);        // zuerst alles aus
  }

  Wire.begin();
  Wire.beginTransmission(DAC_ADDR);
  bool dacOk = (Wire.endTransmission() == 0);

  Serial.println();
  Serial.println("=== T1: Buck-Wandler und Enable-Signale ===");
  Serial.print("DAC 0x0F: ");
  Serial.println(dacOk ? "antwortet" : "KEINE ANTWORT - Ausgaenge koennten auf 7 V gehen!");

  for (int i = 0; i < numChannels; i++) setDAC(i, voutToVdac(0.0));
  Serial.println("DAC auf 0 V Ausgangsspannung gesetzt, alle Kanaele aus.");
  Serial.println("Befehle: E <ch> | A <ch> | ALLEAUS | P | ?");
}

void loop() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim(); s.toUpperCase();
  if (s.length() == 0) return;

  if (s == "?") {
    Serial.println("E <1-10> ein | A <1-10> aus | ALLEAUS | P Pin-Test");
  }
  else if (s == "ALLEAUS") {
    alleAus();
  }
  else if (s == "P") {
    Serial.println("Pin-Test: jedes Enable nacheinander 1 s HIGH ...");
    for (int i = 0; i < numChannels; i++) {
      Serial.print("  Kanal "); Serial.println(i + 1);
      digitalWrite(enablePins[i], HIGH);
      delay(1000);
      digitalWrite(enablePins[i], LOW);
      delay(200);
    }
    Serial.println("Pin-Test beendet.");
  }
  else if (s.startsWith("E ") || s.startsWith("A ")) {
    int ch = s.substring(2).toInt();
    if (ch < 1 || ch > numChannels) { Serial.println("Kanal muss 1..10 sein."); return; }
    bool ein = s.startsWith("E");
    digitalWrite(enablePins[ch - 1], ein ? HIGH : LOW);
    Serial.print("Kanal "); Serial.print(ch);
    Serial.println(ein ? " EIN  (jetzt mit Multimeter messen)" : " AUS");
  }
  else {
    Serial.println("Unbekannter Befehl. '?' fuer Hilfe.");
  }
}
