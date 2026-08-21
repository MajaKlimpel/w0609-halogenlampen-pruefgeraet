/*
  T5 - Luefteransteuerung und Drehzahlrueckmeldung
  ================================================

  Erstellt: 01.07.2026 (Mi)

  Steuert beide Luefter ueber ein gemeinsames PWM-Signal und wertet die
  Tachosignale aus.

  PWM   : Pin 33, 25 kHz (ausserhalb des Hoerbereichs), 8 Bit
  Tacho : Pin 28 = Luefter rechts, Pin 29 = Luefter links
          zwei Impulse je Umdrehung, Auswertung ueber Interrupts

  Hinweis aus der Inbetriebnahme: Das PWM-Signal lag urspruenglich auf Pin 30.
  Dieser Pin ist beim Teensy 4.1 NICHT PWM-faehig, dort liegt nur ein
  statischer Pegel an und die Drehzahl laesst sich nicht regeln. Deshalb
  wurde auf den PWM-faehigen Pin 33 gewechselt.

  Ausgabe: serieller Monitor, 115200 Baud

  Befehle:
    <0-100>   Tastverhaeltnis in Prozent setzen
    K         Kennlinie automatisch aufnehmen (0 bis 100 % in 10er-Schritten)
    ?         Hilfe
*/

const int pinPWM = 33;
const int tach1  = 28;   // rechts
const int tach2  = 29;   // links

volatile unsigned long pulse1 = 0, pulse2 = 0;
void isr1() { pulse1++; }
void isr2() { pulse2++; }

int aktuellPWM = 0;

void setPWM(int prozent) {
  if (prozent < 0) prozent = 0;
  if (prozent > 100) prozent = 100;
  aktuellPWM = prozent;
  analogWrite(pinPWM, map(prozent, 0, 100, 0, 255));
}

// misst ueber ein Fenster von 1 s und liefert die Drehzahlen
void messeDrehzahl(int &rpmR, int &rpmL) {
  noInterrupts(); pulse1 = 0; pulse2 = 0; interrupts();
  unsigned long t0 = millis();
  delay(1000);
  unsigned long dt = millis() - t0;
  noInterrupts();
  unsigned long c1 = pulse1, c2 = pulse2;
  interrupts();
  rpmR = (int)((c1 / (dt / 1000.0)) * 60.0 / 2.0);   // 2 Impulse je Umdrehung
  rpmL = (int)((c2 / (dt / 1000.0)) * 60.0 / 2.0);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(pinPWM, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(pinPWM, 25000);
  analogWrite(pinPWM, 0);

  pinMode(tach1, INPUT_PULLUP);
  pinMode(tach2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tach1), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(tach2), isr2, RISING);

  Serial.println();
  Serial.println("=== T5: Luefter und Drehzahlmessung ===");
  Serial.println("Befehle: <0-100> Prozent | K Kennlinie | ?");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim(); s.toUpperCase();

    if (s == "?") {
      Serial.println("Zahl 0-100 = Tastverhaeltnis, K = Kennlinie aufnehmen");
    }
    else if (s == "K") {
      Serial.println();
      Serial.println("  PWM [%] | rechts [1/min] | links [1/min]");
      Serial.println("  --------+----------------+--------------");
      for (int p = 0; p <= 100; p += 10) {
        setPWM(p);
        delay(2000);                       // Luefter hochlaufen lassen
        int r, l; messeDrehzahl(r, l);
        Serial.print("   "); if (p < 100) Serial.print(" "); if (p < 10) Serial.print(" ");
        Serial.print(p);
        Serial.print("     |      "); Serial.print(r);
        Serial.print("       |     ");  Serial.println(l);
      }
      setPWM(0);
      Serial.println("  Kennlinie beendet, Luefter aus.");
    }
    else if (s.length() > 0 && isDigit(s.charAt(0))) {
      setPWM(s.toInt());
      Serial.print("PWM = "); Serial.print(aktuellPWM); Serial.println(" %");
    }
  }

  // laufende Anzeige
  static unsigned long last = 0;
  if (millis() - last > 2000) {
    last = millis();
    int r, l; messeDrehzahl(r, l);
    Serial.print("PWM "); Serial.print(aktuellPWM);
    Serial.print(" %   rechts "); Serial.print(r);
    Serial.print(" 1/min   links "); Serial.print(l); Serial.println(" 1/min");
    last = millis();
  }
}
