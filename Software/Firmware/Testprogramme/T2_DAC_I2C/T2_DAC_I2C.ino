/*
  T2 - Ansteuerung des DAC ueber den ersten I2C-Bus (Wire)
  ========================================================

  Erstellt: 26.06.2026 (Fr)

  Prueft den Ausgangs-DAC (Adresse 0x0F, 16 Bit, 0...5 V) an SDA 18 / SCL 19.

  Der DAC beeinflusst die Rueckkopplung der Buck-Wandler. Die Strecke ist
  INVERTIEREND: hoher DAC-Wert -> kleine Lampenspannung.
      DAC 5,00 V  ->  Ausgang ~0 V
      DAC 0,00 V  ->  Ausgang ~7 V

  Ausgabe: serieller Monitor, 115200 Baud

  Befehle:
    S            I2C-Bus scannen
    D <ch> <V>   DAC-Spannung eines Kanals direkt setzen (ch 1-10, V 0-5)
    O <ch> <V>   gewuenschte AUSGANGSspannung setzen (rechnet in DAC um)
    R <ch>       Rampe: DAC von 5 V auf 0 V und zurueck (langsam, zum Mitmessen)
    N            alle Kanaele auf 0 V Ausgangsspannung
    ?            Hilfe
*/

#include <Wire.h>

#define DAC_ADDR 0x0F
const int numChannels = 10;
const float VREF = 5.0;
const uint16_t DAC_MAX = 65535;

float voutToVdac(float vout) {
  float v = (7.03 - vout) / 1.41;
  if (v < 0) v = 0;
  if (v > 5) v = 5;
  return v;
}

bool setDAC(uint8_t ch, float voltage) {
  if (voltage < 0) voltage = 0;
  if (voltage > 5) voltage = 5;
  uint16_t value = (uint16_t)((voltage / VREF) * DAC_MAX);
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(0x10 | (ch & 0x0F));
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  return (Wire.endTransmission() == 0);
}

void scan() {
  Serial.println("I2C-Scan auf 'Wire' (SDA 18 / SCL 19):");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("   gefunden: 0x");
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if (a == DAC_ADDR) Serial.print("   <-- Ausgangs-DAC");
      Serial.println();
      n++;
    }
  }
  if (n == 0) Serial.println("   keine Geraete gefunden!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(100000);

  Serial.println();
  Serial.println("=== T2: DAC-Ansteuerung ueber I2C ===");
  scan();
  for (int i = 0; i < numChannels; i++) setDAC(i, voutToVdac(0.0));
  Serial.println("Alle DAC-Kanaele auf 0 V Ausgangsspannung gesetzt.");
  Serial.println("Befehle: S | D <ch> <V> | O <ch> <V> | R <ch> | N | ?");
}

void loop() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim(); s.toUpperCase();
  if (s.length() == 0) return;

  if (s == "?") {
    Serial.println("S Scan | D <ch> <V> DAC direkt | O <ch> <V> Ausgang | R <ch> Rampe | N alle 0 V");
  }
  else if (s == "S") {
    scan();
  }
  else if (s == "N") {
    for (int i = 0; i < numChannels; i++) setDAC(i, voutToVdac(0.0));
    Serial.println("Alle Kanaele auf 0 V Ausgangsspannung.");
  }
  else if (s.startsWith("D ") || s.startsWith("O ")) {
    int sp = s.indexOf(' ', 2);
    if (sp < 0) { Serial.println("Format: D <ch> <V>"); return; }
    int ch = s.substring(2, sp).toInt();
    float v = s.substring(sp + 1).toFloat();
    if (ch < 1 || ch > numChannels) { Serial.println("Kanal 1..10"); return; }
    float vdac = s.startsWith("D") ? v : voutToVdac(v);
    bool ok = setDAC(ch - 1, vdac);
    Serial.print("Kanal "); Serial.print(ch);
    Serial.print(": DAC = "); Serial.print(vdac, 3); Serial.print(" V");
    Serial.print("  -> erwartete Ausgangsspannung ~");
    Serial.print(7.03 - 1.41 * vdac, 2); Serial.print(" V");
    Serial.println(ok ? "" : "   [FEHLER: kein ACK vom DAC]");
  }
  else if (s.startsWith("R ")) {
    int ch = s.substring(2).toInt();
    if (ch < 1 || ch > numChannels) { Serial.println("Kanal 1..10"); return; }
    Serial.print("Rampe auf Kanal "); Serial.println(ch);
    for (int p = 50; p >= 0; p--) {          // 5,00 V -> 0,00 V
      float vdac = p / 10.0;
      setDAC(ch - 1, vdac);
      Serial.print("  DAC "); Serial.print(vdac, 2);
      Serial.print(" V  -> erwartet ~"); Serial.print(7.03 - 1.41 * vdac, 2); Serial.println(" V");
      delay(300);
    }
    for (int p = 0; p <= 50; p++) { setDAC(ch - 1, p / 10.0); delay(100); }
    setDAC(ch - 1, voutToVdac(0.0));
    Serial.println("Rampe beendet, Kanal wieder auf 0 V Ausgangsspannung.");
  }
  else {
    Serial.println("Unbekannter Befehl. '?' fuer Hilfe.");
  }
}
