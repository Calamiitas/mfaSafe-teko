#include <LiquidCrystal.h>

#include <Keypad.h>

char P1 = '1';
char P2 = '2';
char P3 = '3';
char P4 = '4';        // Hier werden die vier Zeichen des Passwortes eingegeben Hier: "123A"
char C1, C2, C3, C4;  // Unter C1 bis C4 werden im Loop die vier eingegebenen Zeichen gespeichert

//Servo servoblau;     //Servo wird ab jetzt mit „servoblau“ angesprochen
int roteLED = 11;    //Die rote LED ist an Pin 12 angeschlossen
int grueneLED = 12;  //Die grüne LED wird an Pin 13 angeschlossen
int blaueLED = 13;
//Hier wird die größe des Keypads definiert
const byte COLS = 4;     //4 Spalten
const byte ROWS = 4;     //4 Zeilen
int z1 = 0, z2, z3, z4;  // Diese Variablen werden verwendet um für die einzelnen Zahlencodes die EIngabe freizuschalten. Damit wird im Sketch verhindert, dass eine einzene Codeziffer einer falschen Position zugeordnet wird.
//Die Ziffern und Zeichen des Keypads werden eingegeben:
char hexaKeys[ROWS][COLS] = {
  { 'D', '#', '0', '*' },
  { 'C', '9', '8', '7' },
  { 'B', '6', '5', '4' },
  { 'A', '3', '2', '1' }
};

byte colPins[COLS] = { 40, 41, 42, 43 };  //Definition der Pins für die 3 Spalten
byte rowPins[ROWS] = { 44, 45, 46, 47 };  //Definition der Pins für die 4 Zeilen
char Taste;                           //Taste ist die Variable für die jeweils gedrückte Taste.
Keypad Tastenfeld = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); //Das Keypad kann absofort mit "Tastenfeld" angesprochen werden
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                    // Wire Bibliothek einbinden
#include <LiquidCrystal_I2C.h>       // Vorher hinzugefügte LiquidCrystal_I2C Bibliothek einbinden
LiquidCrystal_I2C lcd(0x27, 16, 2);  //Hier wird festgelegt um was für einen Display es sich handelt. In diesem Fall eines mit 16 Zeichen in 2 Zeilen und der HEX-Adresse 0x27. Für ein vierzeiliges I2C-LCD verwendet man den Code "LiquidCrystal_I2C lcd(0x27, 20, 4)"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Stepper.h>
int SPU = 2048;
Stepper Motor(SPU, 23, 25, 27, 29);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

int trigger = 32;
int echo = 33;
long dauer = 0;
long distanz = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <MFRC522.h>
#define SS_PIN 53
#define RST_PIN 5
MFRC522 mfrc522(SS_PIN, RST_PIN);
bool rfidOk = false;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {

  Serial.begin(9600);
  pinMode(roteLED, OUTPUT);  //Die LEDs werden als Ausgang festgelegt
  pinMode(grueneLED, OUTPUT);
  pinMode(blaueLED, OUTPUT);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  lcd.init();       //Im Setup wird der LCD gestartet
  lcd.backlight();  //Hintergrundbeleuchtung einschalten (lcd.noBacklight(); schaltet die Beleuchtung aus).

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Motor.setSpeed(8);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(9600);        //Serielle Kommunikation starten, damit man sich später die Werte am Serial Monitor ansehen kann.
  pinMode(trigger, OUTPUT);  // Trigger-Pin ist ein Ausgang
  pinMode(echo, INPUT);      // Echo-Pin ist ein Eingang

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //pinMode(3, OUTPUT);
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
}

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
   {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  long code = 0;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    code = ((code + mfrc522.uid.uidByte[i]) * 10);

    //Serial.print(mfrc522.uid.uidByte[i], DEC);
  }
  Serial.print("Die Kartennummer lautet:");  //Stringausgabe
  Serial.println(code);                      //Ausgabe des gelesenen Code

  if (code == 1993910)  //Gültige Kartennummer
  {
    digitalWrite(13, HIGH);
    lcd.setCursor(0, 0);        //Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile.
    lcd.print("Hallo Jackie");  //Die LED leuchtet auf, wenn eine gültige Karte gelesen wird
    lcd.setCursor(0, 1);        // In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile.
    lcd.print("Bitte PIN eingeben");
    delay(2000);           //Das ist die Dauer einer leuchtenden LED
    digitalWrite(13, LOW);  //LED löscht wieder ab
    rfidOk = true;
  } else if (code == 268330)  //Weitere Gültige Kartennummer
  {
    digitalWrite(13, HIGH);
    lcd.setCursor(0, 0);       //Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile.
    lcd.print("Hallo Simon");  //Die LED leuchtet auf, wenn eine gültige Karte gelesen wird
    lcd.setCursor(0, 1);       // In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile.
    lcd.print("Bitte PIN eingeben");
    delay(2000);           //Das ist die Dauer einer leuchtenden LED
    digitalWrite(13, LOW);  //LED löscht wieder ab
    rfidOk = true;
  } else {
    digitalWrite(11, HIGH);
    lcd.setCursor(0, 0);          //Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile.
    lcd.print("NOPE NOPE NOPE");  //Die LED leuchtet auf, wenn eine ungültige Karte gelesen wird
    delay(2000);                  //Das ist die Dauer einer leuchtenden LED
    digitalWrite(11, LOW);        //LED löscht wieder ab 
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

                        // Dies ist eine Markierung, zu der per "goto-"Befehl gesprungen werden kann.
  Taste = Tastenfeld.getKey();  //Mit Unter der Variablen pressedKey entspricht der gedrückten Taste
  if (Taste && rfidOk == true)                    //Wenn eine Taste gedrückt wurde...
                                //Ab hier werden die Eingaben des Tastenfeldes verarbeitet. Zunächst die "*"Taste, da diese eine Sonderfunktion für die Verriegelung besitzt und danach die #-Taste, nach deren Eingabe der zuvor eingegebene Code auf Richtigkeit geprüft wird.
  {
    if (Taste == '*')  // Wenn die "*" Taste gedrückt wurde...
    {
      Serial.println("Tuer verriegelt");
      delay(3000);
      Motor.step(-1024);              //Servo zum verriegeln auf 90 Grad ansteuern.
      digitalWrite(roteLED, HIGH);   //..die rote LED einschalten
      digitalWrite(grueneLED, HIGH);  //..die grüne LED einschalten
      digitalWrite(blaueLED, HIGH);
      z1 = 0;
      z2 = 1;
      z3 = 1;
      z4 = 1;       // Zugang zur ersten Zeicheneingabe freischalten
       //An dieser Stelle springt der Sketch zur Eingabe der Taste zurück, damit das Zeichen "*" nicht im folgenden Abschlitt als Codeeingabe gewertet wird.
    }

    if (Taste == '#')  // Wenn die Rautetaste gedrückt wurde...
    {
      if (C1 == P1 && C2 == P2 && C3 == P3 && C4 == P4)  //wird gepüft, ob die eingaben Codezeichen (C1 bis C4) mit den Zeichen des Passwortes (P1 bis P4) übereinstimmen. Falls der eingegebene Code richtig ist...
      {
        Serial.println("Code korrekt, Schloss offen");
        Motor.step(512);                //Servo zum öffnen auf 0 Grad ansteuern.
        digitalWrite(roteLED, LOW);     //..die rote LED nicht leuchten..
        digitalWrite(grueneLED, HIGH);  //..die grüne LED leuchten..
      } else                            // ist das nicht der Fall, bleibt das Schloss gesperrt
      {
        Serial.println("Code falsch, Schloss gesperrt");
        digitalWrite(roteLED, HIGH);   // Die rote LED leuchtet
        digitalWrite(grueneLED, LOW);  // Die grüne LED is aus
        delay(3000);
        z1 = 0;
        z2 = 1;
        z3 = 1;
        z4 = 1;       // Der Zugang für die erste Zeicheneingabe wird wieder freigeschaltet
         //An dieser Stelle springt der Sketch zur Eingabe der Taste zurück, damit das Zeichen "#" nicht im folgenden Abschlitt als Codeeingabe gewertet wird.
      }
    }
    // Ab hier werden die vier Code-positionen unter den Variablen C1 bis C4 abgespeichert. Damit die eingegebenen Zeichen auch an der richtigen Position des Passwortes gespeichert werden, wird mit den Variablen z1 bis z4 der Zugang zu den einzelnen Positinen freigegeben oder gesperrt.
    if (z1 == 0)  // Wenn das erste Zeichen noch nicht gespeichert wurde...
    {
      C1 = Taste;                  //Unter der Variablen "C1" wird nun die aktuell gedrückte Taste gespeichert
      Serial.print("Die Taste ");  //Teile uns am Serial Monitor die gedrückte Taste mit
      Serial.print(C1);
      Serial.println(" wurde gedrueckt");
      z1 = 1;
      z2 = 0;
      z3 = 1;
      z4 = 1;  // Zugang zur zweiten Zeicheneingabe freischalten
      
    }

    if (z2 == 0)  // Wenn das zweite Zeichen noch nicht gespeichert wurde...
    {
      C2 = Taste;                  //Unter der Variablen "C2" wird nun die aktuell gedrückte Taste gespeichert
      Serial.print("Die Taste ");  //Teile uns am Serial Monitor die gedrückte Taste mit
      Serial.print(C2);
      Serial.println(" wurde gedrueckt");
      z1 = 1;
      z2 = 1;
      z3 = 0;
      z4 = 1;  // Zugang zur dritten Zeicheneingabe freischalten
    
    }

    if (z3 == 0)  // Wenn das dritte Zeichen noch nicht gespeichert wurde...
    {
      C3 = Taste;                  //Unter der Variablen "C3" wird nun die aktuell gedrückte Taste gespeichert
      Serial.print("Die Taste ");  //Teile uns am Serial Monitor die gedrückte Taste mit
      Serial.print(C3);
      Serial.println(" wurde gedrueckt");
      z1 = 1;
      z2 = 1;
      z3 = 1;
      z4 = 0;  // Zugang zur vierten Zeicheneingabe freischalten
      
    }

    if (z4 == 0)  // Wenn das vierte Zeichen noch nicht gespeichert wurde...
    {
      C4 = Taste;                  //Unter der Variablen "C4" wird nun die aktuell gedrückte Taste gespeichert
      Serial.print("Die Taste ");  //Teile uns am Serial Monitor die gedrückte Taste mit
      Serial.print(C4);
      Serial.println(" wurde gedrueckt");
      z1 = 1;
      z2 = 1;
      z3 = 1;
      z4 = 1;  // Zugang zur weiteren Zeicheneingabe sperren
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    digitalWrite(trigger, HIGH);
    delay(10);
    digitalWrite(trigger, LOW);
    dauer = pulseIn(echo, HIGH);
    distanz = (dauer / 2 * 0.03432);
    while (distanz <= 50) {
      delay(3000);
      Motor.step(-512);
      
    }
  }
}
