#include <I2C_16Bit.h>
#include <I2C_32Bit.h>
#include <I2C_8Bit.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23XXX.h>
#include <Stepper.h>


#define I2C_MULTI 0x20
#define I2C_INA226 0x44
#define SUNRELAY_PIN 0
#define MOTOR_SPEED 2
int motorPin1 = 2;
int motorPin2 = 3;
int motorPin3 = 4;
int motorPin4 = 5;
int motorPin5 = 6;
int motorPin6 = 7;
int motorPin7 = 8;
int motorPin8 = 9;

int xLimit = 10; //richtige pin Nummer setzen
int yLimit = 11; //^
int MAX_STEPS = 4096;
Adafruit_MCP23X17 mcp;

class motor : public Stepper{
  private:
  int speed = MOTOR_SPEED;
  public:
  motor(int pin1, int pin2, int pin3, int pin4) : Stepper(4096, pin1, pin2, pin3, pin4){
    
  };
  void test(int i){
    step(i);
  };
  int calibrate(){
    // kalibrieren
    while(!mcp.digitalRead(xLimit)){
      mystepper.step(1);
    }
    posStep = 0;
    return posStep;
  }
  int posStep;
  int posWinkel;
  int step(int step){};
  int cal(){};
};

class ldrGruppe{
  private:
  public:
    getStrenght(){

    }
};
motor test(motorPin1,motorPin2,motorPin3,motorPin4);

void setup() {
  Serial.begin(9600);

  //motor xMotor(4096, motorPin1, motorPin2, motorPin3, motorPin4);
  //motor yMotor(4096, motorPin5, motorPin6, motorPin7, motorPin8);
  ldrGruppe ldrOben;
  ldrGruppe ldrRechts;
  ldrGruppe ldrUnten;
  ldrGruppe ldrLinks;
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }

  // configure pin for output
  mcp.pinMode(SUNRELAY_PIN, OUTPUT);
  //xMotor.calibrate();
  //yMotor.calibrate();
}

void loop() {
  test.test(1);
}
