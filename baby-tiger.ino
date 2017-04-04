//#include <toneAC2.h>

//#include <toneAC.h>




#include <NewPing.h>


#include <pitches.h>
#include <Servo.h>

#define TRIGGER_PIN   12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN      11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE  200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

class ServoExtended
{
  // By Jude Gustafson 2-22-2017
  public:
  Servo servo; //the servo we are extending
  int digPin;      // the pin we are going to put our servo on
  int servoSpeed;     // the speed our servo runs at in milliseconds - the servos we are using take two ms per degree
  int powerSupply;    // I know we are only using one at the moment but this is for later support default to one
  int orientation;                 // current orientation of the servo before we move it
  unsigned long travelTime;   // the computed the in ms that it will take for our servo to get to its destination
 
  
  ServoExtended(int pin, int speedInMS = 2.6, int ps = 1, int orient = 0)
  {
  digPin = pin;
  //servo.attach(digPin);
  //servo.write(0); 
    
  servoSpeed = speedInMS;
  powerSupply = ps;
  
  orientation = orient; 
  travelTime = 0;
  }
 
 
  void attach(int pin)
  {
    servo.attach(pin);
  }
  void write(int goal, int addDelay = 0)
  {
    int destination = (servo.read() - goal);
    int changeAmount = abs(destination);
    //Serial.println("orientation");
    //Serial.println(orientation);
    Serial.println("goal");
    Serial.println(goal);
    //Serial.println("destination");
    //Serial.println(destination);
    servo.write(goal);
    orientation = goal;
    travelTime = (changeAmount * servoSpeed) + addDelay;
  }
  void writeSlow(int goal, int addDelay = 0)
  {
    int destination = (servo.read() - goal);
    int changeAmount = abs(destination);
    if(servo.read() > goal) {
      for(int i = servo.read(); i >= goal; i--) {
        servo.write(i);
        travelTime = (changeAmount * (servoSpeed * 2));
      }
    }
    else {
        for(int i = servo.read(); i < goal; i--) {
        servo.write(goal);
        travelTime = (changeAmount * (servoSpeed * 2));
      }
    }
    orientation = goal;
    travelTime = travelTime + addDelay;
  }
};

//Servo myservo;
//Servo myservo3;
//Servo myservo4;

ServoExtended myservo(10);
ServoExtended myservo2(9);
ServoExtended myservo3(8);

int ServosControl = 4;    // Digital Arduino Pin used to control the relay
int currentServoIndex = -1;
int servoSpeedInMS = 2;
int startMillis = 0;
unsigned long endMillis = 0;
unsigned long timeoutMillis = 0;
int cascadeLength = 4;
int currentOrientation = 0;
int servoDegreesCascade[] = {180, 0, 180, 0};
boolean servosAreMoving = false;
unsigned long tailEndMillis = 0;
boolean tailIsMoving = false;

//Music
// notes in the melody:
int melody[] = {NOTE_E4,NOTE_E4,NOTE_E4,NOTE_F4,NOTE_E4,NOTE_D4,NOTE_C4,
                NOTE_E4,0,NOTE_G3,
                NOTE_G4,NOTE_G4,NOTE_G4,NOTE_A4,NOTE_G4,NOTE_F4,NOTE_E4,
                NOTE_F4,NOTE_E4,NOTE_D4,NOTE_C4,
                NOTE_B4,NOTE_A4,NOTE_G4,NOTE_E4,NOTE_B3,
                NOTE_B4,NOTE_A4,
                NOTE_G4,NOTE_G4,NOTE_E4,NOTE_F4,NOTE_G4,NOTE_F4,NOTE_E4,
                NOTE_G4,NOTE_G4,NOTE_E4,NOTE_F4,NOTE_G4,
                0, NOTE_C4,NOTE_C4,NOTE_D4,NOTE_E4,
                0, NOTE_C4,NOTE_C4,NOTE_A4,NOTE_G4,NOTE_C4,
                NOTE_E4,NOTE_D4,NOTE_C4,0};
float beats[] = {1,.66,.34,.66,.34,.66,.34,//7
                  3,.25,.75,//3
                  1,.66,.34,.66,.34,.66,.34,//7
                  1,1,1,1,//4
                  2,.66,.34,.66,.34,//5
                  2,2,//2
                  1,.66,.34,.66,.34,.66,.34,//7
                  1,.66,.34,.66,1.34,//5
                  1,1,1,1,4,//5
                  1,.66,.34,.66,.34,1,//6
                  .33,.17,3.5,8};//4
int melodyLength = 56;
int melodyIndex = -1;
int duration = 1000;
unsigned long musicEndMilis = 0;

boolean musicIsPlaying = false;

/************************************************/
void setup()
{
  myservo.attach(10);//attachs the servo on pin 9 to servo object
  myservo.servo.write(1);//back to 0 degrees 
  myservo2.attach(9);//attachs the servo on pin 9 to servo object
  myservo2.servo.write(1);//back to 0 degrees
  myservo3.attach(8);//attachs the servo on pin 9 to servo object
  myservo3.servo.write(15);//back to 0 degrees
  //myservo4.attach(7);//attachs the servo on pin 9 to servo object
  //myservo4.servo.write(0);//back to 0 degrees
  Serial.begin(9600);
  //digitalWrite(ServosControl,HIGH);
  pinMode(ServosControl, OUTPUT);
  delay(1000);//wait for a second
}
/*************************************************/
void loop()
{
      
      unsigned int uS = sonar.ping();
      Serial.println("CM");
      Serial.println(uS / US_ROUNDTRIP_CM);
        if(uS / US_ROUNDTRIP_CM == 0 && timeoutMillis != 0 && millis() >=timeoutMillis ) {
          Serial.println("nothing");
          digitalWrite(ServosControl,LOW);
            return;
          }
         else if (uS / US_ROUNDTRIP_CM == 0 && timeoutMillis == 0)  {
          timeoutMillis = millis() + 3000;
         }
         else if (uS / US_ROUNDTRIP_CM != 0){
          timeoutMillis = 0;
          digitalWrite(ServosControl,HIGH);
         }

        if(uS / US_ROUNDTRIP_CM < 20) {
          //digitalWrite(ServosControl,HIGH);
          if(musicIsPlaying) {
            if(millis() >= musicEndMilis) {
              Serial.println("musicIsPlaying");
              Serial.println(musicIsPlaying);
                musicIsPlaying = false;
              }
            }
            if(!musicIsPlaying) {
              playMusic();
              //meow();
            }
            freakOut();
            return;
          }

        if(uS / US_ROUNDTRIP_CM < 30) {
          //digitalWrite(ServosControl,HIGH);
            moveEyesSlowly();
            swayTail();
            return;
          }
        if(uS / US_ROUNDTRIP_CM != 0 && uS / US_ROUNDTRIP_CM < 90) {
          //digitalWrite(ServosControl,HIGH);
            swayTail();
            return;
          }
        //

        //freakOut();
        //moveEyesSlowly();
        //swayTail();

      

      
      
      
}

void moveServo(ServoExtended servo,int goal, int addDelay = 0) {
    
    servosAreMoving = true;
    servo.write(goal, addDelay);
    if(servo.travelTime + millis() > endMillis) {
    
    //Serial.println("travelTime");
    //Serial.println(servo.travelTime);
      Serial.println(servo.travelTime);
      endMillis = millis() + servo.travelTime;
    }
    //delay(1000);
    //delay(333);
}

void moveTail(ServoExtended servo,int goal,int addDelay = 0) {
    
    tailIsMoving = true;
    servo.write(goal, addDelay);
    if(servo.travelTime + millis() > tailEndMillis) {
      Serial.println(servo.travelTime);
      tailEndMillis = millis() + servo.travelTime;
    }
}

void moveEyesSlowly() {

    
    if(servosAreMoving) {
      if(millis() >= endMillis) {
          servosAreMoving = false;
        }
      //return;
      }
      else {
        if(myservo.servo.read() != 1) {
          moveServo(myservo, 1, 2000);
          moveServo(myservo2, 1, 2000);
        }
        else {
          moveServo(myservo, 180, 2000);
          moveServo(myservo2, 180, 2000);
        }
      }
}

void swayTail() {
    
    if(tailIsMoving) {
      if(millis() >= tailEndMillis) {
        Serial.println("tail time expired");
          tailIsMoving = false;
        }
        //return;
      }
      else {
        if(myservo3.servo.read() == 180) {
            moveTail(myservo3, 155, 400);
          }
        else if(myservo3.servo.read() == 160) {
            moveTail(myservo3, 180, 600);
          }
        else if(myservo3.servo.read() == 155) {
            moveTail(myservo3, 75, 600);
          }
        else if(myservo3.servo.read() == 75) {
            moveTail(myservo3, 90, 100);
          }
        else {
            moveTail(myservo3, 160, 300);
          }
       }
}

void freakOut() {

    if(tailIsMoving) {
      if(millis() >= tailEndMillis) {
        Serial.println("tail time expired");
          tailIsMoving = false;
        }
        //return;
      }
      else {
        if(myservo3.servo.read() == 150) {
            moveTail(myservo3, 110);
          }
        else {
            moveTail(myservo3, 150);
          }
       }
    
    if(servosAreMoving) {
      if(millis() >= endMillis) {
          servosAreMoving = false;
        }
      return;
      }
      else {
        Serial.println("servosAreMoving");
        Serial.println(servosAreMoving);
        Serial.println("myservo.servo.read()");
        Serial.println(myservo.servo.read());
        Serial.println(myservo.travelTime);
        if(myservo.servo.read() != 1) {
          moveServo(myservo, 1);
          moveServo(myservo2, 1);
        }
        else {
          moveServo(myservo, 180);
          moveServo(myservo2, 180);
        }
      }
}

void playTone(int tone1, int toneDuration = 500, float noteLength = 1) {
  musicIsPlaying = true;
  float currentNoteLengthMS = (toneDuration * noteLength);
  Serial.println("Note Length");
  Serial.println(noteLength);
  if(tone1 != 0)
    //toneAC(tone1, 10, int(currentNoteLengthMS)-30);
  
  musicEndMilis = millis() + currentNoteLengthMS;
}

void meow() {  // cat meow (emphasis ow "me")
  uint16_t i;
  playTone(1200,50);        // "m" (short)
  playTone(100,180);        // "eee" (long)
  for(i=400; i<500; i+=2)  // vary "ooo" down
     playTone(i,8);
  playTone(2200,40);        // "w" (short)
}
void playMusic() {

    if(melodyIndex == -1 || melodyIndex == melodyLength) {
        melodyIndex = 0;
    }
    playTone(melody[melodyIndex], 500, beats[melodyIndex]);
    melodyIndex ++;
    
    //delay(1000);
    //delay(333);
}

/**************************************************/
