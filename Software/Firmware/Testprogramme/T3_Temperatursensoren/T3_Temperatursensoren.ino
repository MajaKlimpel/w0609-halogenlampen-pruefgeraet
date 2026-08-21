/*
  T3 - Temperatursensoren ueber den zweiten I2C-Bus (Wire1)
  =========================================================

  Erstellt: 29.06.2026 (Mo)

  Liest die sechs Temperatursensoren an SDA 17 / SCL 16 zyklisch aus.

  Sensoradressen laut Bestueckung:
      T1 = 0x4B   T2 = 0x4D   T3 = 0x48
      T4 = 0x4A   T5 = 0x4C   T6 = 0x49

  Messwert: 12 Bit, 0,0625 C je Bit, Register 0x00.

  Hinweis aus der Inbetriebnahme: Meldet sich kein Sensor, zuerst pruefen,
  ob SDA und SCL vertauscht sind (Wire1 erwartet SDA auf Pin 17, SCL auf 16).

  Ausgabe: serieller Monitor, 115200 Baud
*/

#include <Wire.h>

const int numSensors = 6;
uint8_t sensors[numSensors]     = {0x4B, 0x4D, 0x48, 0x4A, 0x4C, 0x49};
const char* sensorName[numSensors] = {"T1", "T2", "T3", "T4", "T5", "T6"};

float readTemp(uint8_t addr) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);
  if (Wire1.endTransmission() != 0) return -999;        // kein ACK

  Wire1.requestFrom(addr, (uint8_t)2);
  if (Wire1.available() < 2) return -999;

  uint8_t msb = Wire1.read();
  uint8_t lsb = Wire1.read();
  int16_t raw = (msb << 8) | lsb;
  raw >>= 4;
  if (raw & 0x800) raw |= 0xF000;                       // Vorzeichen
  return raw * 0.0625;
}

void scan() {
  Serial.println("I2C-Scan auf 'Wire1' (SDA 17 / SCL 16):");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire1.beginTransmission(a);
    if (Wire1.endTransmission() == 0) {
      Serial.print("   gefunden: 0x");
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      for (int i = 0; i < numSensors; i++)
        if (sensors[i] == a) { Serial.print("   <-- "); Serial.print(sensorName[i]); }
      Serial.println();
      n++;
    }
  }
  Serial.print("   Summe: "); Serial.print(n); Serial.println(" Geraet(e)");
  if (n < numSensors)
    Serial.println("   ACHTUNG: nicht alle sechs Sensoren gefunden!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire1.begin();
  Wire1.setClock(100000);

  Serial.println();
  Serial.println("=== T3: Temperatursensoren ===");
  scan();
  Serial.println();
  Serial.println("   T1      T2      T3      T4      T5      T6      (Grad C)");
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    float t = readTemp(sensors[i]);
    if (t < -900) Serial.print("  ----  ");
    else { Serial.print("  "); Serial.print(t, 2); Serial.print(" "); }
  }
  Serial.println();
  delay(1000);
}
