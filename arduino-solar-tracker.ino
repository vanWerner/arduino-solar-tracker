#include <I2C_16Bit.h>
#include <I2C_32Bit.h>
#include <I2C_8Bit.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>
#include <Stepper.h>


#define I2C_MULTI 0x20
#define I2C_INA226 0x44
//#define SUNRELAY_PIN 0
#define MOTOR_SPEED 2
int motorPin1 = 2;
int motorPin2 = 3;
int motorPin3 = 4;
int motorPin4 = 5;
int motorPin5 = 6;
int motorPin6 = 7;
int motorPin7 = 8;
int motorPin8 = 9;

int xLimit = 0; //MCP ID GPB0
int yLimit = 0; //^      GPB1
int maxSteps = 4096;

Adafruit_MCP23X17 mcp;

// Klasse die die Stepper Klasse vererbt
// Grund dafür ist dass verfolgt werden soll wie weit der Stepper vom Nullpunkt entfernt wird, nullpunkt wird mit dem calibrieren festgestellt 
class Motor : public Stepper{
  private:
    int speed = MOTOR_SPEED;
    int pos;
    int limitPin;
  public:
    Motor(int int1, int int2, int int3, int int4, int pin) : Stepper(maxSteps, int1, int2, int3, int4){
      limitPin = pin;
      mcp.pinMode(xLimit, INPUT_PULLUP);
    };
    // bewegen den Stepper soweit bis der Limit Switch berührt wird, das ist der nullpunkt
    void calibrate(){
      Serial.println("Calibrating...");
      do{
        step(1);
        delay(50);
      }while(mcp.digitalRead(limitPin));
      Serial.println("Calibrated!");
      pos = 0;
    };
    void move(int i){
      if(i>0){
        for(i; i>0;i--){
          step(1);
          pos++;
          delay(50);
        }
      }
      else{
        for(i; i<0;i++){
          step(-1);
          pos--;
          delay(50);
        }
      }
    }
    void moveTo(int i){
      i = i - pos;
      move(i);
    }
    int getPos(){
      return pos;
    }
    float getPosWinkel(){
      return (float)pos / (float)maxSteps;
    }
};

class ldrGruppe{
  private:
    int ldrPin1;
    int ldrPin2;
  public:
    ldrGruppe(int ldr1, int ldr2){
      ldrPin1 = ldr1;
      ldrPin2 = ldr2;
    };
    float getStrenght(){
      return analogRead(ldrPin1) / analogRead(ldrPin2);
    }
};

// Initialisieren der Instanzen
Motor xMotor(motorPin1, motorPin2, motorPin3, motorPin4, xLimit);
Motor yMotor(motorPin5, motorPin6, motorPin7, motorPin8, yLimit);
ldrGruppe ldrOben(A5,A4);
ldrGruppe ldrRechts(A4,A3);
ldrGruppe ldrUnten(A3,A2);
ldrGruppe ldrLinks(A2,A5);

// Variablen zum Vergleich der Stärke der LDRs, werden auf null initialisiert
ldrGruppe* strongestLDRptr = nullptr;
float strongestStrenght = 0.0;

void setup() {
  // Initialisieren von Serial und I2C
  Serial.begin(9600);
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }
  Serial.println("mcp begin");
  //mcp.pinMode(xLimit, INPUT_PULLUP);
  // configure pin for output
  //mcp.pinMode(SUNRELAY_PIN, OUTPUT);
  xMotor.calibrate();
  yMotor.calibrate();
}

void loop() {
  Serial.println(mcp.digitalRead(xLimit));
  delay(500);

  // setzte einen Pointer auf die Instanz von ldrGruppe die die größte Stärke aufweist
  // dabei wir ldrOben als erstes als größte festgehalten weil es noch nichts zum vergleiche gibt
  // über if statements wird dann die Stärkste Instanz ermittelt und als strongestStrenght gespeichert, sowie ein Pointer "strongestLDRptr" auf die Instanz gesetzt
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
}
