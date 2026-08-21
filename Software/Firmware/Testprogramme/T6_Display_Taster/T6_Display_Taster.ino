/*
  T6 - Display, Taster und Status-LED
  ===================================

  Erstellt: 02.07.2026 (Do)

  Prueft die Bedien- und Anzeigeelemente auf der Leiterplatte.

  Display  : ILI9341 ueber SPI
             CS = 10, DC = 39, RST = 40, Hintergrundbeleuchtung = 41
  Taster   : Pin 35, INPUT_PULLDOWN (gedrueckt = HIGH)
  Status-LED (RGB): Rot = 36, Gruen = 37, Blau = 38

  Ablauf: Beim Start laufen ein Farbtest des Displays und ein Durchlauf der
  LED-Farben. Danach zeigt das Display fortlaufend den Tasterzustand und
  einen Zaehler der Tastendruecke; jeder Druck wird zusaetzlich seriell
  gemeldet und schaltet die LED-Farbe weiter.

  Ausgabe: serieller Monitor, 115200 Baud
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS   10
#define TFT_DC   39
#define TFT_RST  40
#define TFT_LITE 41

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

const int pinButton = 35;
const int pinRed = 36, pinGreen = 37, pinBlue = 38;

int  farbIndex = 0;
long zaehler   = 0;
bool letzterTasterZustand = LOW;

void setLED(bool r, bool g, bool b) {
  digitalWrite(pinRed,   r ? HIGH : LOW);
  digitalWrite(pinGreen, g ? HIGH : LOW);
  digitalWrite(pinBlue,  b ? HIGH : LOW);
}

void naechsteFarbe() {
  farbIndex = (farbIndex + 1) % 5;
  switch (farbIndex) {
    case 0: setLED(1,1,1); Serial.println("LED: weiss");   break;
    case 1: setLED(1,0,0); Serial.println("LED: rot");     break;
    case 2: setLED(0,1,0); Serial.println("LED: gruen");   break;
    case 3: setLED(0,0,1); Serial.println("LED: blau");    break;
    case 4: setLED(1,1,0); Serial.println("LED: gelb");    break;
  }
}

void displayTest() {
  uint16_t farben[] = {ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE,
                       ILI9341_WHITE, ILI9341_BLACK};
  const char* namen[] = {"ROT", "GRUEN", "BLAU", "WEISS", "SCHWARZ"};
  for (int i = 0; i < 5; i++) {
    tft.fillScreen(farben[i]);
    Serial.print("Display: "); Serial.println(namen[i]);
    delay(600);
  }
}

void zeichneStatus(bool gedrueckt) {
  tft.fillRect(0, 60, 320, 120, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(20, 70);
  tft.print("Taster: ");
  tft.setTextColor(gedrueckt ? ILI9341_GREEN : ILI9341_RED);
  tft.print(gedrueckt ? "EIN" : "AUS");
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 120);
  tft.print("Druecke: ");
  tft.print(zaehler);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(pinRed, OUTPUT); pinMode(pinGreen, OUTPUT); pinMode(pinBlue, OUTPUT);
  pinMode(pinButton, INPUT_PULLDOWN);
  setLED(0,0,0);

  pinMode(TFT_LITE, OUTPUT);
  digitalWrite(TFT_LITE, HIGH);          // Hintergrundbeleuchtung an
  tft.begin();
  tft.setRotation(1);

  Serial.println();
  Serial.println("=== T6: Display, Taster und LED ===");

  displayTest();

  Serial.println("LED-Farbtest ...");
  setLED(1,0,0); delay(500);
  setLED(0,1,0); delay(500);
  setLED(0,0,1); delay(500);
  setLED(1,1,1); delay(500);
  farbIndex = 0;

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.print("T6 Funktionstest");
  zeichneStatus(false);

  Serial.println("Bereit. Taster druecken (schaltet die LED-Farbe weiter).");
}

void loop() {
  bool jetzt = digitalRead(pinButton);

  if (letzterTasterZustand == LOW && jetzt == HIGH) {   // steigende Flanke
    delay(30);                                          // entprellen
    if (digitalRead(pinButton) == HIGH) {
      zaehler++;
      Serial.print("Taster gedrueckt (");
      Serial.print(zaehler); Serial.println("x)");
      naechsteFarbe();
      zeichneStatus(true);
    }
  }
  if (letzterTasterZustand == HIGH && jetzt == LOW) {
    zeichneStatus(false);
  }
  letzterTasterZustand = jetzt;
  delay(20);
}
