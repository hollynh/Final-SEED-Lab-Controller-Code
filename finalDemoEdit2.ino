/////////////////////////////////////////////////////////////////////////////
// Group 11
// EENG 350
// Spring 2022
// Dr. Sager
// Demo 1
//
/////////////////////////////////////////////////////////////////////////////
#include <Encoder.h>
#include <Wire.h>

bool startBool = false;

///////////////////////////////////////
// ADDED THESE
// for deltaV
float currPosLeftAng = 0;
float currPosRightAng = 0;
float angVel = 0;

//Tabs
void vBarFuncPos();
void deltaVFunc();
void motSat();
void vBarFuncVel();

int readIn = 0;

// for vBarPos
float currPosLeftPos = 0;
float currPosRightPos = 0;

// for vBarVel
float currPosLeftVel = 0;
float currPosRightVel = 0;
float currPosAve = 0;
float oldPosAve = 0;

// for recieve delays
float timeReceive = 0;
////////////////////////////////////////



#define SLAVE_ADDRESS 0x05
int counter1 = 0;
int counter2 = 0;

float currr = 0;

bool demo1 = false;

bool inRange = false;
bool stopNow = false;
bool outOfAngle = false;

bool straightDone = false;

// name pins for motor to run
const int D2Left = 4;
const int D2Right = 5;
const int motDirLeft = 7;
const int motDirRight = 8;
const int motorVLeft = 9;
const int motorVRight = 10;
const float diameter = 5.75;  //inches
const float robRad = 5.875;   //inches
int commandLeft = 0;
int commandRight = 0;
int commandLeftPrev = 0;
int commandRightPrev = 0;
bool reset = 0;
int vmax = 255;

// encoder declarations
int aPinLeft = 3;   //aPin will use interrupts, but bPin will not.
int bPinLeft = 11;
int aPinRight = 2;   //aPin will use interrupts, but bPin will not.
int bPinRight = 6;
//float currPosLeft = 0; // REMOVED
//float currPosRight = 0; // REMOVED
//int currPosIntLeft = 0;
//int currPosIntRight = 0;
//float radPosLeft = 0.0;
//float radPosRight = 0.0;
bool resetLeft = 0;
bool resetRight = 0;
float tau = 10; //ms?
int t1 = 0;
int t2 = 0;

Encoder myEncLeft(aPinLeft, bPinLeft);
Encoder myEncRight(aPinRight, bPinRight);

// controller parameters
float Kp1 = 5; // was 8
float Ki1 = 0.0;

//deltaV control
float Kp2 = 5; // was 9
float Ki2 = 0;

// time variables to keep track of period wait times
int period = 10;
unsigned long time_now = 0;

// controller time variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

// controller error variables
float errorL = 1;
float errorR = 1;
float cumErrorLeft = 0;
float cumErrorRight = 0;

float setDist = 0;
float setVel = 0;     // feet/s
float setAngle = 0;   //degrees
float fudgeFactor = 0.03;
float angleDist = 0;
float setStraight = 0;
float currAngle = 0;

float vBar = 0;
float deltaV = 0;

bool isDone = false;

float dumVar = 0;

void setup() {


  // communication with PI
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);


  // declare inputs/outputs
  pinMode(D2Left, OUTPUT);
  pinMode(D2Right, OUTPUT);
  pinMode(motDirLeft, OUTPUT);
  pinMode(motDirRight, OUTPUT);
  pinMode(motorVLeft, OUTPUT);
  pinMode(motorVRight, OUTPUT);
  pinMode(aPinLeft, INPUT_PULLUP);
  pinMode(bPinLeft, INPUT_PULLUP);
  pinMode(aPinRight, INPUT_PULLUP);
  pinMode(bPinRight, INPUT_PULLUP);
  Serial.begin(9600);
  setAngle = setAngle * (1 + fudgeFactor);

  // motors off
  digitalWrite(D2Left, LOW);
  digitalWrite(D2Right, LOW);

  // stop sending PWM to the motors
  analogWrite(motorVLeft, 0);
  analogWrite(motorVRight, 0);
}

void loop() {

  if (startBool == true) {
    // ADDED TIME VARS
    currentTime = millis();
    time_now = currentTime;
    elapsedTime = currentTime - previousTime;


    if (readIn == 100) {
      // run motors

      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);
      //Serial.println("turn 360");
      angleDist = (2 * PI * robRad / (12 * 360)) * 360;
      setStraight = 0;
      //vmax = 45;
      deltaVFunc();
      vBarFuncPos();
      //Serial.println("no marker found");
    }

    // negative values
    /*else if (readIn == 92){
      // run motors
      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);
      angleDist = (2 * PI * robRad / (12 * 360)) * -5;
      Serial.print("readIn : \t");
      Serial.print(readIn);
      Serial.print("\t");
      Serial.print("angleDist: \t");
      Serial.println(angleDist);

      setVel = 1;
      deltaVFunc();
      vBarFuncVel();
      }
    */
    // do we have a case for just getting to the tape after the 360? or are we using the case below to
    // turn and go straight to it?

    else if (((readIn <= 27) && (readIn > 2)) || ((readIn < -2) && (readIn >= -27))) { // we're just following the tape here
      // run motors
      //vmax = 100;
      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);
      angleDist = (2 * PI * robRad / (12 * 360)) * readIn;
      setVel = 10; // changed this
      deltaVFunc();
      vBarFuncVel();
      //Serial.println("going straight and turning less tight");
    }
    else if (readIn >= -2 && readIn <= 2) { // changed from 2
      // run motors
      //vmax = 80;
      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);
      angleDist = (2 * PI * robRad / (12 * 360)) * readIn;
      //Serial.print("Go straight\t");
      //Serial.println(angleDist);
      setVel = 10; // changed this
      deltaVFunc();
      vBarFuncVel();
      //Serial.println("going straight and turning tight");
    }
    else if (readIn == 99) { // ADDED THIS CASE TO USE POSITION CONTROLLER, if we get to break in the tape at the end
      // run motors
      //vmax = 80;
      digitalWrite(D2Left, HIGH);
      digitalWrite(D2Right, HIGH);
      //Serial.println("End");
      setStraight = 0.5;
      angleDist = 0;
      vBarFuncPos();
      deltaVFunc();
    }
    else if (readIn == 98) {
      // run motors
      digitalWrite(D2Left, LOW);
      digitalWrite(D2Right, LOW);
      //Serial.println("stop");
      setVel = 0;
      vBarFuncVel();
      deltaVFunc();
    }

    // update motor commands
    commandLeft = vBar + deltaV;
    commandRight = vBar - deltaV;

    motSat();

    // ADDED TIME VARS
    previousTime = currentTime;



  }
  
  Serial.print("command left : \t");
  Serial.print(commandLeft);
  Serial.print("\t");
  Serial.print("command right: \t");
  Serial.println(commandRight);
  

}



// callback for recieved data
void receiveData(int byteCount) {
  if (Wire.available()) {
    //if((millis() - timeReceive) >= elapsedTime){ // timing this loop so it has a delay
    myEncLeft.write(0);
    myEncRight.write(0);
    readIn = (Wire.read()) - 30;
    //Serial.print("Angle from Pi: \t");
    //Serial.println(readIn);
    startBool = true;

    //timeReceive = millis();
    //}
  }
}
