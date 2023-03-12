#include <Keypad.h>
int P = 1127;  // Hier werden die vier Zeichen des Passwortes eingegeben Hier: "1234"
String C;  // Unter C werden im Loop die vier eingegebenen Zeichen gespeichert

//Servo servoblau;     //Servo wird ab jetzt mit „servoblau“ angesprochen
int roteLED = 11;    //Die rote LED ist an Pin 12 angeschlossen
int grueneLED = 12;  //Die grüne LED wird an Pin 13 angeschlossen
int blaueLED = 13;
//Hier wird die größe des Keypads definiert
//Hier wird die größe des Keypads definiert
const byte COLS = 4; //4 Spalten
const byte ROWS = 4; //4 Zeilen
int z1=0, z2, z3, z4; // Diese Variablen werden verwendet um für die einzelnen Zahlencodes die EIngabe freizuschalten. Damit wird im Sketch verhindert, dass eine einzene Codeziffer einer falschen Position zugeordnet wird.
//Die Ziffern und Zeichen des Keypads werden eingegeben:
char hexaKeys[ROWS][COLS]={
{'D','#','0','*'},
{'C','9','8','7'},
{'B','6','5','4'},
{'A','3','2','1'}
};

byte colPins[COLS] = {2,3,4,6}; //Definition der Pins für die 3 Spalten
byte rowPins[ROWS] = {7,8,9,10}; //Definition der Pins für die 4 Zeilen
char Taste; //Taste ist die Variable für die jeweils gedrückte Taste.
Keypad Tastenfeld = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); //Das Keypad kann absofort mit "Tastenfeld" angesprochen werden


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


long readRfid() {
  long code = 0;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    code = ((code + mfrc522.uid.uidByte[i]) * 10);

    //Serial.print(mfrc522.uid.uidByte[i], DEC);
  }
  Serial.print("Die Kartennummer lautet:");  //Stringausgabe
  Serial.println(code);                      //Ausgabe des gelesenen Code
  return code;
}

void greetUser(String name) {

    digitalWrite(13, HIGH);
    clearDisplay();        //Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile.
    lcd.print("Hallo " + name);  //Die LED leuchtet auf, wenn eine gültige Karte gelesen wird
    delay(2000);
    clearDisplay();        // In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile.
    lcd.print("Bitte PIN eingeben"); //Das ist die Dauer einer leuchtenden LED
    digitalWrite(13, LOW);  //LED löscht wieder ab
    
}

void closeDoor(){
      Serial.println("Tuer verriegelt");
      delay(3000);
      Motor.step(512);              //Servo zum verriegeln auf 90 Grad ansteuern.
      digitalWrite(roteLED, LOW);   //..die rote LED einschalten
      digitalWrite(grueneLED, LOW);  //..die grüne LED einschalten
      digitalWrite(blaueLED, LOW);
}

bool pinValid(String currentPin){
  return currentPin.toInt() == P;
}

void openDoor(){

  Serial.println("Code korrekt, Schloss offen");
  Motor.step(-512);                //Servo zum öffnen auf 0 Grad ansteuern.
  digitalWrite(roteLED, LOW);     //..die rote LED nicht leuchten..
  digitalWrite(grueneLED, HIGH);  //..die grüne LED leuchten..
}

String readPin() {
  String pin = "";
  Taste = readSinglePinEntry();

  while (Taste != '0'){
    if (Taste == '*') {
      closeDoor();  // Zugang zur ersten Zeicheneingabe freischalten
    }
      // Ab hier werden die vier Code-positionen unter den Variablen C1 bis C4 abgespeichert. Damit die eingegebenen Zeichen auch an der richtigen Position des Passwortes gespeichert werden, wird mit den Variablen z1 bis z4 der Zugang zu den einzelnen Positinen freigegeben oder gesperrt.

    pin += Taste;                  //Unter der Variablen "C1" wird nun die aktuell gedrückte Taste gespeichert
    Serial.print("Die Taste ");  //Teile uns am Serial Monitor die gedrückte Taste mit
    Serial.print(Taste);
    Serial.println(" wurde gedrueckt");
    Serial.println("aktueller pin: " + pin);
    
    Taste = readSinglePinEntry();
  }
  return pin;
}

void clearDisplay(){
  lcd.clear();
  lcd.setCursor(0,0);
}

char readSinglePinEntry(){
  return Tastenfeld.waitForKey();
}

void loop() {
  lcd.setCursor(0,0);
  lcd.print("Karte bitte :)");
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  long code = readRfid();
  if (code == 1993910)  //Gültige Kartennummer
  {
    greetUser("Jackie");
    rfidOk = true;
  } else if (code == 268330)  //Weitere Gültige Kartennummer
  {
    greetUser("Simon");
    rfidOk = true;
  } else {
    digitalWrite(11, HIGH);
    clearDisplay();          //Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile.
    lcd.print("NOPE NOPE NOPE");  //Die LED leuchtet auf, wenn eine ungültige Karte gelesen wird
    delay(2000);                  //Das ist die Dauer einer leuchtenden LED
    digitalWrite(11, LOW);
    return;  //LED löscht wieder ab
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  String pin = readPin();
  Serial.println(pin);
  if (pinValid(pin)) {
    Serial.println("pin was valid");
    clearDisplay();
    lcd.print("Pin war korrekt");
    openDoor();
    char key = readSinglePinEntry();
    while (key != '*'){
      key = readSinglePinEntry();
    }
    closeDoor();
    clearDisplay();
  }
}
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* 

  }

*/
