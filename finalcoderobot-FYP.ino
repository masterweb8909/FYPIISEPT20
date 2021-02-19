//-----initialize Bluetooth module------/
//#include <SoftwareSerial.h>
//SoftwareSerial BT (10, 11);

//-----initialize ultrasonic sensor-----/
#define trig1 2
#define echo1 3
#define trig2 4
#define echo2 5
#define trig3 7
#define echo3 8

//-----initialize MLX90614 temp sensor---/
#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//-----initialize water sensor-----------/
const int read = A0;
int valWtrLvl;

//-----initializa LED & Buzzer & Button--/
int ledindicator = 46;
int ledpin = 44;
int buzzpin = 42;
int btnpin = 6;


//-----initialize DC motor---------------/
//*********WHEELS***********************/
const int ENA = 24;
const int IN1 = 26;
const int IN2 = 28;
const int IN3 = 30;
const int IN4 = 35;
const int ENB = 37;

//*********BLADE***********************/
const int IN1B1 = 34;
const int IN2B1 = 36;
const int IN3B2 = 38;
const int IN4B2 = 40;

//*********WATER PUMP DC**************/ will be added the exact pin once connected with H-bridge
const int IN1W = 50;
const int IN2W = 52;

//-------global declaration---------//
long duration, distance;
int Us1state = LOW, Us2state = LOW, Us3state = LOW, Us4state = LOW;
int btnstate;  //initialize button state before begin
int bladestate = LOW;
int ledstate = LOW;
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 0;    // algorithm to be included for button state and switch on & off circuit
int Waterlvl;
int Maxlevel = 275;   //max water level

//------------controller declaration----------//
int drivemode=0;    //(auto)1 or (manual)2
char manualstate;   //joystick controller F-front, B-back, L-left, R-right, S-stop
char path;          //circular-c, rectangular-r, square-s

void setup() {
  // put your setup code here, to run once:
  pinMode(btnpin, INPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  pinMode(echo3, INPUT);
  pinMode(ledindicator, OUTPUT);
  pinMode(ledpin, OUTPUT);
  pinMode(buzzpin, OUTPUT);
  pinMode(trig1, OUTPUT);
  pinMode(trig2, OUTPUT);
  pinMode(trig3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1B1, OUTPUT);
  pinMode(IN2B1, OUTPUT);
  pinMode(IN3B2, OUTPUT);
  pinMode(IN4B2, OUTPUT);
  //  BT.begin(9600);
  mlx.begin();
  Serial.begin(9600);
  digitalWrite(ledindicator, ledstate);
  digitalWrite(IN1B1, bladestate);
  digitalWrite(IN3B2, bladestate);

}
//---------------------------ultrasonic sensor-----------//

//-------------------------water level sensor-----------//
//-------------------------mlx90614---------------------//
//--------------------------dc MOTOR--------------------//
//****************movement***************************//
void front() {
  digitalWrite(IN1, HIGH);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 132);    //analog is to control
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 132);
}
void back() {
  digitalWrite(IN1, LOW);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 132);    //analog is to control
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 132);
}
void right() {
  digitalWrite(IN1, LOW);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);    //analog is to control
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 132);
}
void left () {
  digitalWrite(IN1, HIGH);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 132);    //analog is to control
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
void stops() {
  digitalWrite(IN1, LOW);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);    //analog is to control
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

//***************DC for BLADE****************//
void bladeon() {          //no analogread since i set it to maximum speed = 255
  digitalWrite(IN1B1, bladestate);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2B1, LOW);
  digitalWrite(IN3B2, bladestate);
  digitalWrite(IN4B2, LOW);
}
void bladestop() {
  digitalWrite(IN1B1, LOW);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2B1, LOW);
  digitalWrite(IN3B2, LOW);
  digitalWrite(IN4B2, LOW);
}

//***************WATER PUMP DC***************//
void waterpump() {            //set for max speed(PWM) = 255
  digitalWrite(IN1W, HIGH);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2W, LOW);
}
void offwaterpump() {       //this function will stop the dc from moving
  digitalWrite(IN1W, LOW);  //digital represent the 1 & 0 for on or off
  digitalWrite(IN2W, LOW);
}
//-----------------------------MAIN LOOP (CODE)----------//
void loop() {
  //********initialize distance & duration for ultrasonic sensor(3 us)*****/
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1,HIGH);
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2,HIGH);
  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3,HIGH);
  
  long leftus , frontus, rightus, timeus1, timeus2, timeus3;
  timeus1 = pulseIn(echo1, HIGH);   //distance input from echo pin
  timeus2 = pulseIn(echo2, HIGH);
  timeus3 = pulseIn(echo3, HIGH);
  leftus  = (timeus1 / 2) / 29.1; //distance output at trig pin after calculating the distance in cm unit
  frontus = (timeus2 / 2) / 29.1;
  rightus = (timeus3 / 2) / 29.1;
  //    drivemode = Serial.read();
  //    manualstate = Serial.read();
  Serial.println(drivemode);
  Serial.println(manualstate);
  int btnreading = digitalRead(btnpin);
  //-----Main switch ON/OFF-----//
  if (btnreading != lastButtonState) {    // If the switch changed, due to noise or pressing:
    lastDebounceTime = millis();
  }      // reset the debouncing timer

  if ((millis() - lastDebounceTime) > debounceDelay) {  // whatever the reading is at, it's been there for longer than the debounce // delay, so take it as the actual current state:
    // if the button state has changed:
    if (btnreading != btnstate) {
      btnstate = btnreading;
      // only toggle the program if the new button state is HIGH
      if (btnstate == HIGH) { //user manually switch ON/OFF
        ledstate = !ledstate;
        digitalWrite(ledindicator, ledstate);
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = btnreading;

  while (Serial.available() > 0) { //checking if BT is connectec
    delay(10);
    drivemode = Serial.read();
    manualstate = Serial.read();
    Serial.println(drivemode);
    Serial.println(manualstate);
  }
  //*******for mlx90614******//
  int humidTemp = mlx.readAmbientTempC(); //calc humid tempt(ambient air) - humid temp <26
  int groundTemp = mlx.readObjectTempC(); //calc ground tempt(ambient air) - ground temp <26
      if (groundTemp < 28) {  //exclude humidTemp < 26  &&
        waterpump();
      }
      else {
        offwaterpump();
      }
    Serial.println(groundTemp);

  if (drivemode == 1) {   //if user select automode
    Serial.println("Auto Mode Activated");
    bladestate = !bladestate;
    bladeon();    //****blade start to operate***//

    if (leftus < 7) {         //obstacle on left side
      if (frontus < 7) {        //check obstacle in front of the robot
        if (rightus < 7) {        //check obstacle on right side of robot
          back();
        }                //all sensor meet obstacle = robot will reverse until no obstacle
        else {                    //left & right != obstacle = move front
          front();
        }
      }
      else {                      //if no osbtacle in front = move forward
        front();
      }
    }
    else {
      front();
    }

    if (frontus < 7) {      //obstacle infront of robot
      if (leftus > 7  && rightus > 7) {   //no obstacle within the radius = turn right
        right();
      }
      else if (leftus < 7 && rightus > 7) {
        left();
      }
      else if (leftus > 7 && rightus < 7) {
        right();
      }
    }
    else {
      if (leftus > 7 && rightus > 7) {
        front();
      }
    }

    //********detect ground temperature & humidity****//
    if (groundTemp < 27) {  //exclude humidTemp < 26  &&
      stops();    //stops when temp < 27 & activate waterpump
      delay(500);
      waterpump();
    }
    else {
      offwaterpump();
      front();
    }

    //*******WATER LEVEL INDICATOR****//
    if (Waterlvl > Maxlevel) {      //maximum water level warning alarm (led = ON + buzzer = ON)
      stops();
      digitalWrite(ledpin, HIGH);
      tone(buzzpin, 1000);
      delay(1000);
      noTone(buzzpin);
      delay(1000);
    }

    if (manualstate == 'E') {   //when user press reset button
      return (0);
    }
  }
  else if (drivemode == 2) {  //will act as joystick
    Serial.println("Manual Mode Activated");
    bladestate = !bladestate;
    bladeon();
    Serial.println("Robot Stop");
    stops();
    //*******WATER LEVEL INDICATOR****//
    if (Waterlvl > Maxlevel) {
      stops();
      digitalWrite(ledpin, HIGH);
      tone(buzzpin, 1000);
      delay(1000);
      noTone(buzzpin);
      delay(1000);
    }
    else {
      if (groundTemp < 27) {  //exclude humidTemp < 26  &&
        waterpump();
      }
      else {
        offwaterpump();
      }
    }
  }
  //-----manual joystick controller
  if (manualstate == 'F') {
    Serial.println("Forward");
    front();
  }
  else if (manualstate == 'R') {
    Serial.println("Right");
    right();
  }
  else if (manualstate == 'L') {
    Serial.println("Left");
    left();
  }
  else if (manualstate == 'B') {
    Serial.println("Backward");
    back();
  }
  else if (manualstate == 'E') {  //resetbutton
    return 0;
  }
  Serial.println(leftus);
  Serial.println(frontus);
  Serial.println(rightus);
  return 0;
}
