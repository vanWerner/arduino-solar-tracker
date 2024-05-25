#include <I2C_16Bit.h>
#include <I2C_32Bit.h>
#include <I2C_8Bit.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>
#include <Stepper.h>


#define I2C_MULTI 0x20
#define I2C_INA226 0x44
#define SUNRELAY_PIN 3
#define MOTOR_SPEED 200
int xMotorPin1 = 2;
int xMotorPin2 = 4;
int xMotorPin3 = 3;
int xMotorPin4 = 5;
int yMotorPin1 = 6;
int yMotorPin2 = 8;
int yMotorPin3 = 7;
int yMotorPin4 = 9;
int sonnenMotorPin1 = 10;
int sonnenMotorPin2 = 12;
int sonnenMotorPin3 = 11;
int sonnenMotorPin4 = 13;

int xLimit = 0; //MCP ID GPA0
int yLimit = 1; //^      GPA1
int sonnenLimit = 2; // GPA2
int maxSteps = 4096;
long currentMillis = 0;
unsigned int moveBits[3] = {0, 0, 0};

Adafruit_MCP23X17 mcp;

// Klasse die die Stepper Klasse vererbt
// Grund dafür ist dass verfolgt werden soll wie weit der Stepper vom Nullpunkt entfernt wird, nullpunkt wird mit dem calibrieren festgestellt 
class Motor : public Stepper{
  private:
    int speed = MOTOR_SPEED;
    int pos = 0;
    int limitPin;
    int pin1, pin2, pin3, pin4;
    int inverse = 0;
  public:
    Motor(int int1, int int2, int int3, int int4, int pin) : Stepper(maxSteps, int1, int2, int3, int4){
      limitPin = pin;
      mcp.pinMode(limitPin, INPUT_PULLUP); //TODO: correct var
      pin1 = int1;
      pin2 = int2;
      pin3 = int3;
      pin4 = int4;
    };
    // bewegen den Stepper soweit bis der Limit Switch berührt wird, das ist der nullpunkt
    void calibrate(){
      Serial.println("Calibrating...");
      if(!inverse){
        do{
          step(2);
          Serial.print(mcp.digitalRead(limitPin));
          delay(100);
        }while(mcp.digitalRead(limitPin));
      }else{
        do{
          step(-2);
          Serial.print(mcp.digitalRead(limitPin));
          delay(100);
        }while(mcp.digitalRead(limitPin));
      }
      Serial.println("Calibrated!");
      stop();
      pos = 0;
    };
    void move(int i){
      if(i>0){
        for(i; i>0;i--){
          step(1);
          pos++;
          delay(20);
        }
      }
      else{
        for(i; i<0;i++){
          step(-1);
          pos--;
          delay(20);
        }
      }
      stop();
    }
    void setInverse(){
      inverse = 1;
    }
    void moveTo(int i){
      Serial.print("Moving to:");
      Serial.print(i);
      Serial.print(" Current pos:");
      Serial.print(pos);
      Serial.print(" Moving by:");
      i = i - pos;
      Serial.print(i);
      move(i);
    }
    int getPos(){
      return pos;
    }
    float getPosWinkel(){
      return (float)pos / (float)maxSteps;
    }
    void stop(){
      digitalWrite(pin1, 0);
      digitalWrite(pin2, 0);
      digitalWrite(pin3, 0);
      digitalWrite(pin4, 0);
    }
};

class ldrGruppe{
  private:
    int ldrPin1 = 0;
    int ldrPin2 = 0;
    int moveBits = 0;
    int offset = 0;
  public:
    ldrGruppe(int ldr1, int ldr2, int offsetValue, int* bits){
      ldrPin1 = ldr1;
      ldrPin2 = ldr2;
      if(bits > 7){
        Serial.println("Motor Bits sind mehr als 3 Bits");
        return;
      }
      moveBits = bits;
      offset = offsetValue;
    };
    int getStrenght(){
      return int(((analogRead(ldrPin1) + analogRead(ldrPin2)) / 2) - offset);
    }
    unsigned int getMotorBits(){
      return moveBits;
    }
    
};

// Initialisieren der Instanzen
Motor xMotor(xMotorPin1, xMotorPin2, xMotorPin3, xMotorPin4, xLimit);
Motor yMotor(yMotorPin1, yMotorPin2, yMotorPin3, yMotorPin4, yLimit);
Motor sonne(sonnenMotorPin1, sonnenMotorPin2, sonnenMotorPin3, sonnenMotorPin4, sonnenLimit);
ldrGruppe ldrOben(A2, A3, 0, 3);
ldrGruppe ldrRechts(A0, A3, 0, 4);
ldrGruppe ldrUnten(A0, A1, 0, 2);
ldrGruppe ldrLinks(A1, A2, 0, 5);

// Variablen zum Vergleich der Stärke der LDRs, werden auf null initialisiert
ldrGruppe* strongestLDRptr = nullptr;
float strongestStrenght = 0.0;

/*void moveMotorByBit(motorBits[]){
  int direction = 0;
  if(motorBits[3] == 1)
    direction = -1;
  else
    direction = 1;
  if(motorBits[1] == 1)
}*/
void setup() {
  // Initialisieren von Serial und I2C
  Serial.begin(9600);
  Serial.println("I2C begin");
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }
  Serial.println("mcp begin");
  mcp.pinMode(xLimit, INPUT_PULLUP);
  mcp.pinMode(yLimit, INPUT_PULLUP);
  mcp.pinMode(sonnenLimit, INPUT_PULLUP);
  // configure pin for output
  mcp.pinMode(SUNRELAY_PIN, OUTPUT);
  randomSeed(analogRead(0));
  xMotor.setSpeed(20);
  yMotor.setSpeed(20);
  sonne.setSpeed(20);
  sonne.setInverse();
  /*xMotor.calibrate();
  delay(1000);
  //yMotor.calibrate();
  delay(1000);*/
  sonne.calibrate();
  xMotor.move(-10);
  yMotor.move(-10);
}

void loop() {
  
  // setzte einen Pointer auf die Instanz von ldrGruppe die die größte Stärke aufweist
  // dabei wir ldrOben als erstes als größte festgehalten weil es noch nichts zum vergleiche gibt
  // über if statements wird dann die Stärkste Instanz ermittelt und als strongestStrenght gespeichert, sowie ein Pointer "strongestLDRptr" auf die Instanz gesetzt
  // TODO: Ändern auf kleinsten Wert
  strongestStrenght = ldrOben.getStrenght();
  strongestLDRptr = &ldrOben;
  if(strongestStrenght < ldrRechts.getStrenght()){
    strongestStrenght = ldrRechts.getStrenght();
    strongestLDRptr = &ldrRechts;
  }
  if(strongestStrenght < ldrUnten.getStrenght()){
    strongestStrenght = ldrUnten.getStrenght();
    strongestLDRptr = &ldrUnten;
  }
  if(strongestStrenght < ldrLinks.getStrenght()){
    strongestStrenght = ldrLinks.getStrenght();
    strongestLDRptr = &ldrLinks;
  }
  Serial.println("*****");
  Serial.println(strongestLDRptr->getMotorBits());
  Serial.println(strongestLDRptr->getStrenght());
  /*Serial.print("Oben: ");
  Serial.print(ldrOben.getStrenght());
  Serial.print("   Rechts: ");
  Serial.print(ldrRechts.getStrenght());
  Serial.print("   Unten: ");
  Serial.print(ldrUnten.getStrenght());
  Serial.print("   Links: ");
  Serial.println(ldrLinks.getStrenght());*/
  /*Serial.print("Oben: ");
  Serial.print(ldrOben.getStrenght());
  Serial.print("   Rechts: ");
  Serial.print(ldrRechts.getStrenght());
  Serial.print("   Unten: ");
  Serial.print(ldrUnten.getStrenght());
  Serial.print("   Links: ");
  Serial.println(ldrLinks.getStrenght());*/
  Serial.println("*****");
  Serial.print("Unten: ");
  Serial.println(ldrUnten.getStrenght());
  Serial.print("Links: ");
  Serial.println(ldrLinks.getStrenght());
  Serial.print("Oben: ");
  Serial.println(ldrOben.getStrenght());
  Serial.print("Rechts: ");
  Serial.println(ldrRechts.getStrenght());
  Serial.println("*****");
  Serial.println(analogRead(A0));
  Serial.println(analogRead(A1));
  Serial.println(analogRead(A2));
  Serial.println(analogRead(A3));
  Serial.println("Moving Motor");
  moveMotorByBits(strongestLDRptr->getMotorBits());
  //unsigned int output[] = ldrOben.getMotorBits();
  //Serial.println(output[1]);
  /*Serial.println("ON");
  mcp.digitalWrite(SUNRELAY_PIN, 1);
  delay(5000);
  Serial.println("OFF");
  mcp.digitalWrite(SUNRELAY_PIN, 0);
  delay(5000);*/
  if(currentMillis < (millis() - 20000)){
    mcp.digitalWrite(SUNRELAY_PIN, LOW);
    sonne.moveTo(random(155,873));
    mcp.digitalWrite(SUNRELAY_PIN, HIGH);
    currentMillis = millis();
  }
  Serial.print("C:");
  Serial.print(currentMillis/1000);
  Serial.print("M:");
  Serial.println(millis()/1000);
  delay(20);
}

void moveMotorByBits(int motorBits){
  int direction = 0;
  if(bitRead(motorBits, 0) == 1){
    direction = -1;
  }else{
    direction = 1;
  }
  if(bitRead(motorBits, 1) == 1){
    yMotor.move(direction);
  }
  else if(bitRead(motorBits, 2) == 1){
    xMotor.move(direction);
  }else{
    Serial.println("Error in Reading Bits, not MotorBit specified!");
  }
}
