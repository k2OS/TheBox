
//#include <VarSpeedServo.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


TinyGPS gps;
// rx = 7 (yellow), tx = 6 (white)
// vcc = black, gnd = green
SoftwareSerial nss(6,7);

/*
 * LCD RS pin to A1
 * LCD Enable pin to A0
 * LCD Vo to digital pin 3 (no - resistor to ground instead)
 * LCD D4 pin to digital pin 12
 * LCD D5 pin to digital pin 11
 * LCD D6 pin to digital pin 10
 * LCD D7 pin to digital pin 9
 * LCD R/W pin to ground
*/
LiquidCrystal lcd(A1, A0, 12, 11, 10, 9);

Servo myservo;
long previousMillis;
int interval = 1000;

int SERVOPIN = 8; 
long int servostart; // timer to keep track of the servo

// reed switch is attached to this pin - hint: back door
int BACKDOORPIN = 4;
// polulu power switch is on this pin
int POLULUPIN = 2;

// scale stuff for backdoor timer
// used both for backdoor and reset-functionality
// with scales, I am sure that the switch is triggered constantly when I start the backdoor timers
float v = 1; // init
float scalea = 0.99;
float scaleb = 0.01;
// timer for backdoor and reset 
typedef long int longjohn;  // declaring my own long int, just because we joke around with the name 'John' in Labitat.. 
// After this time, the servo will toggle open and then back to locked - will have to be prolonged to prevent accidental opening
longjohn unsigned backdoortimeout1 = 2000; 
// fter this time, the gamestatus will reset - this will need to be prolonged so it can only be triggered on external power
// and/or when the box is programmed with cable when opened.
longjohn unsigned backdoortimeout2 = 25000; 
// keeps track of when the backdoor switch is triggered                                     
longjohn unsigned backdoortimerstart;

// is the backdoor timer running?
int backdoortimerrunning = 0;
// set to 1 when the box is open
int boxopen = 0;
// set to 1 when the backdoor timer reaches backdoor timer2 and this is still 0 - is then being set to 1 to trigger the actual reset
int gamereset = 0;

longjohn unsigned mastertimerstart; // the master timer is used to see if we're timing out on the GPS signal
// mastertimer = timeout
long unsigned int timeout = 70000; // how long will we wait for the GPS (should be 1½ minute or so)?
int timeoutreached = 0;
int powermessage = 0; // keeps track of wether or not we've displayed a message after timeout and shutdown should have occured (displayed when on external power)

int debug = 1; // enables serial debug
int debug2 = 0; // additional serial debug
/*
***** storyline-array
***** I might use this .. or not. Not sure yet.
*/
char* storyline[] = {"Follow the instruc- tions from this box",
  " ",
};

/* GPS-related variables */
float flat, flon;
unsigned long age, date, time, chars = 0;
int year;
byte month, day, hour, minute, second, hundredths;
char datechar[10];
unsigned short sentences = 0, failed = 0;
// coordinates for the various locations for the puzzle box to go

static const float NNIT_LAT = 55.73558, NNIT_LON = 12.47504; // coordinates for my place of work
static const float LABI_LAT = 55.676235, LABI_LON = 12.54561; // coordinates of Labitat
static const float HOME_LAT = 55.91461, HOME_LON = 12.49487; // coordinates of home

// to prevent usage of the old Servo library, I am instead making sure, that I only talk to one of 
// Software Serial or the Servo at a time. These two flags keep check of that.
// only one can be 'attached' at the time
bool servoattached = 0;
bool gpsattached = 1;

/* declare gamestate and tasknr */
int gamestate; // RUNNING, etc.
int tasknr; // which location will we go to next?

/* setup */
void setup() 
{ 
 nss.begin(9600); // initiate SoftwareSerial, which we use to talk to the GPS
 if (debug)
   Serial.begin(115200);

 // set up the LCD's number of columns and rows: 
 lcd.begin(20, 4);

 // Print a message to the LCD.
 // to be moved when the box goes live
 // or maybe print gamestatus + welcome (back)
 lcd.setCursor(0,0);
 // stringToLcd is explained below.
 stringToLCD("Welcome!");
 delay(1000);
 stringToLCD("Initializing...");

 /*
  read Gamestate, etc. from EEPROM
  gamestate = pos 0
  tasknr = pos 1
 */
 gamestate = EEPROM.read(0);
 // if gamestate is bigger than a certain number, it has not been initiated, so we set it to '1' ('0' would cause an endless loop)
 // the manual states, that it's set to 255 if not initially set. You can usually just make a small sketch to set the
 // values instead of guessing like I do.
 if (gamestate > 200 || gamestate == 0) { gamestate = 1; EEPROM.write(0,gamestate); }

 tasknr = EEPROM.read(1);
 // if tasknr is bigger than a certain number, it has not been initiated, so we write it back - see above
 if (tasknr > 200) { tasknr = 0; EEPROM.write(1,tasknr); }

 if (debug) {
   Serial.print("Initial gamestate: ");
   Serial.println(gamestate);
 }

 /* other settings */
 // we're using the internal PULLUP-resistor
 // when the pin is pulled LOW (to GND) things happen.. (backdoor timer)
 pinMode(BACKDOORPIN,INPUT_PULLUP);
 digitalWrite(BACKDOORPIN,HIGH);
 
 // debug-LED. Should probably be disabled when we go "live" as noone can see it anyway
 pinMode(13,OUTPUT);
 digitalWrite(13,LOW);

 /* polulu-pin - pull this one high, and the battery power is cut - has no effect on external power */
 pinMode(POLULUPIN,OUTPUT);
 digitalWrite(POLULUPIN,LOW);

 // mastertimer starts here
 mastertimerstart = millis();
} 
 
void loop() { 
  // re-attach the GPS-module if it's been detached, but only if the servo is detached
  // as the servo will start twitching like crazy if we start talking serial before it detaches
   if (!servoattached && !gpsattached) { 
      nss.begin(9600);
      gpsattached = 1;
   }

   // detach the servo if timeout after movement is reached
   if (myservo.attached() && millis()-servostart>4000) { 
    myservo.detach();
    delay(10); // small delay added for safety although tests so far have been good
    servoattached = 0;
    nss.begin(9600); // the servo has been detached, now we can re-enable GPS
   } 

   if (gpsattached) {
          // feed a few times, to get a good fix, but only if attached
          feedgps(); 
          gps.f_get_position(&flat, &flon, &age);
   }

   // listen for backdoor and do various stuff 
   int sensorVal = digitalRead(BACKDOORPIN);
   v = scalea*v + scaleb*sensorVal;
   // start timer if backdoor-pin is activated
   // disable timer if v isn't under threshold
   if (v <= 0.1 && backdoortimerrunning == 0) {
     backdoortimerstart = millis();
     backdoortimerrunning = 1;
   } else if (v >= 0.1) { backdoortimerrunning = 0; boxopen = 0; }     
   // test for threshold 1 - open and close door
   // maybe I should do something fancy where I don't open and close the door, when I know I am on my way to a reset? naah.. 
   // here we pass the timer for the first backdoor that will toggle the lock
   if (backdoortimerrunning && (millis()-backdoortimerstart >= backdoortimeout1) && (millis()-backdoortimerstart <= backdoortimeout2) && !boxopen) {
     boxopen = 1;
     nss.end(); // stop talking to the GPS
     delay(250);
     gpsattached = 0; // servo will only be attached if the GPS is 'detached'
     unlockbox(); // opens the servo
     if (debug)
       Serial.println("box is opened");
     delay(5000); // I have this much time to open the box
     lockbox(); // closes the servo
   } else if (backdoortimerrunning && (millis()-backdoortimerstart >= backdoortimeout2) && !gamereset) { // test for threshold 2
    gamereset = 1;
    // reset gamestatus to '1'
    // reset tasknr to '0'
    EEPROM.write(0,1); // we're actually writing '1' as GS = 0 is just used to trigger a lock
    EEPROM.write(1,0); // tasknr = 0
    // change gamestate to 3 for shutdown (since we're already running on aux power, there should be no need to test? naa)
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Game has been reset");
    delay(1000);
    gamestate = 0; tasknr = 1;
    if (debug) {
      Serial.println("box has been reset");
      Serial.print("Gamestate changed: ");
      Serial.println(gamestate);
    }
    digitalWrite(POLULUPIN,HIGH); // try and turn off, even though I know this is in vain on aux power
    delay(100);
  } 

  if (debug) {
     Serial.print("Gamestate (pre gs0): ");
     Serial.println(gamestate);
  }


   // here we do a switch on the gamestate to decide what to do
   switch(gamestate) {
    case 0: // game has just been reset - lock the box and shut down - states have been written above
       if (debug) {
         Serial.print("Gamestate (gs0): ");
         Serial.println(gamestate);
       }
       lcd.clear();
       lcd.print("box reset and will ");
       lcd.print("lock in 3 seconds");
       if (debug) { lcd.setCursor(0,3);lcd.print(gamestate);lcd.print(",");lcd.print(tasknr);lcd.print(millis()); }
       nss.end(); // "detach" the GPS before operating the servo
       gpsattached = 0;
       delay(3000);  // the 3 second delay
       lockbox();
       digitalWrite(POLULUPIN,HIGH); // try and turn off (but it's still on aux power, I know ))
       delay(100);
       gamestate = 3; 
       break; 
    case 1: // the game is running
        if (millis()-mastertimerstart <= timeout) {
          // print current millis on the bottom of the LCD
          // timeout-millis()
          // only if we're not timed out
//          unsigned long remaindertime = timeout-millis();
          unsigned long remaindertime = timeout-millis()+mastertimerstart;
          long h,m,s,ms;
          unsigned long over;
          unsigned long elapsed=remaindertime;
          h=int(elapsed/3600000);
          over=elapsed%3600000;
          m=int(over/60000);
          over=over%60000;
          s=int(over/1000);
          ms=over%1000;
          // .. && gamestate = "running" skal der tilføjes .. eller noed
          // .. && !inbetweentasks - the countdown shows up for a tiny bit between steps, so I will add this
          // when I am actually awake
          if (millis()-previousMillis >= interval && remaindertime >= 0 && !timeoutreached) {
            lcd.setCursor(0,3);
            if (m < 10) { lcd.print("0"); }
            lcd.print(m); Serial.print(m);
            lcd.print(":"); 
            if (s < 10) { lcd.print("0"); }
            lcd.print(s); 
            previousMillis = millis();
          } else if (remaindertime <= 0) { lcd.setCursor(0,3); lcd.print("      "); }
        }
        if (millis()-mastertimerstart >= timeout) {
          if (!timeoutreached) {
            lcd.clear();
            lcd.setCursor(0,1);
            lcd.print("No signal - Good Bye");
            if (debug)
              Serial.println("Timeout reached - good bye!");
            delay(2000);
            // pull polulu high etc.
            digitalWrite(POLULUPIN,HIGH);
            // set gamestate to the limbo-state where we just wait for power to go (or the reset)
            timeoutreached = 1;
          } else if (timeoutreached && !powermessage){
            // wait a while then ask to have power removed - we have to set a flag so this step is detected alone
            gamestate = 3;
            if (debug) {
               Serial.print("Gamestate changed: ");
               Serial.println(gamestate);
            }
          }
        }
        // re-attach the GPS-module if it's been detached
        if (!servoattached && !gpsattached) { 
            nss.begin(9600);
            gpsattached = 1;
        }
        if (gpsattached) {
                // feed a few times, to get a good fix.
                feedgps(); 
                gps.f_get_position(&flat, &flon, &age);
        }        
        //if (age < 1000), then we probably have a fix
          switch(tasknr) {
            case 0: // welcome message and first mission - we're not showing this unless we actually have a GPS fix
                  if (age < 1000) { 
                      unsigned long distance = gps.distance_between(flat,flon,LABI_LAT,LABI_LON);
                      // are we within 1000m? (could probably be set lower to make it more exciting)
                      if (distance < 1000) {
                         lcd.clear();
                         stringToLCD("You made it to Labitat!"); 
                         delay(5000);
                         stringToLCD("Stand by for your next mission");
                         delay(5000);
                         tasknr++;
                         EEPROM.write(1,tasknr);
                         mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                           stringToLCD("Go to Labitat and   hack");
                           delay(10000);
                           lcd.clear();
                           lcd.setCursor(0,2);
                           lcd.print("Good Bye");
                           delay(3000);
                           digitalWrite(POLULUPIN,HIGH);
                           delay(100);
                           gamestate = 3; // switch to message if running on external power
                      }
                  }
            break; 
            case 1: // Second mission
                  if (age < 1000) { 
                      unsigned long distance = gps.distance_between(flat,flon,HOME_LAT,HOME_LON);
                      if (distance < 500) {
                         lcd.clear();
                         stringToLCD("You made it home!"); 
                         delay(10000);
                         stringToLCD("Stand by for your next mission");
                         delay(5000);
                         tasknr++;
                         EEPROM.write(1,tasknr);
                         mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                           stringToLCD("Go home");
                           delay(2000);
                           lcd.clear();
                           lcd.setCursor(0,2);
                           lcd.print("Good Bye");
                           delay(3000);
                           digitalWrite(POLULUPIN,HIGH);
                           delay(100);
                           gamestate = 3; // switch to message if running on external power
                      }
                  }
            break; 
            case 2: // Third mission
              stringToLCD("You've made it through..");
              delay(2000);
              stringToLCD("The box will open...");
              EEPROM.write(0,2); // setting gs to 2
              delay(3000);
              gamestate = 2;
            break; 
            case 3: // 4th mission
            break; 
        }
        break;
    case 2: 
         if (debug) {
           Serial.print("Gamestate (gs2): ");
           Serial.println(gamestate);
        }
        nss.end(); //stop talking to the GPS
        delay(250);
        gpsattached = 0;  
        unlockbox();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("The box is open");
        lcd.setCursor(0,2);
        lcd.print("Good Bye");
        delay(5000);

        digitalWrite(POLULUPIN,HIGH);
        delay(100);
        gamestate = 3; // switch to message if running on external power
        break;

    case 3: // we're waiting for power to be removed
     if (!powermessage) {
       delay(1000);
       lcd.clear();
       lcd.setCursor(0,1);
       lcd.print("please remove power");
       powermessage = 1;
        if (debug) {
           Serial.print("Gamestate (gs3): ");
           Serial.println(gamestate);
        }
        
       if (debug)
         Serial.println("Please remove power");

     }
     break;
   }
 
 

} 

/* 
   various functions 
*/

void lockbox() {
 servoattached = 1;
  if (!myservo.attached()) { myservo.attach(SERVOPIN); myservo.write(10); }

// myservo.slowmove(10,70);
 myservo.write(10);
 servostart = millis();
 delay(500);
 digitalWrite(13,LOW);
}

void unlockbox() {
 servoattached = 1;
  if (!myservo.attached()) { myservo.attach(SERVOPIN); }

// myservo.slowmove(90,70);
 myservo.write(90);
 servostart = millis();
 delay(500);
 digitalWrite(13,HIGH);
}

void stringToLCD(char *stringIn) {
    int lineCount = 0;
    int lineNumber = 0;
    byte stillProcessing = 1;
    byte charCount = 1;
    lcd.clear();
    lcd.setCursor(0,0);

    while(stillProcessing) {
         if (++lineCount > 20) {    // have we printed 20 characters yet (+1 for the logic)
              lineNumber += 1;
              lcd.setCursor(0,lineNumber);   // move cursor down
              lineCount = 1;
         }

         lcd.print(stringIn[charCount - 1]);

         if (!stringIn[charCount]) {   // no more chars to process?
              stillProcessing = 0;
         }
         charCount += 1;
          delay(100);
    }
}

static bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}
