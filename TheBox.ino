#include <Servo.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Time.h>

#include <MemoryFree.h>
#include <pgmStrToRAM.h>

TinyGPS gps;
// rx = 7 (yellow), tx = 6 (white)
// vcc = black, gnd = green
SoftwareSerial nss(6,7);

/*
 * LCD RS pin to A1
 * LCD Enable pin to A0
 * LCD Vo to digital pin 3 (no - resistor to vcc instead)
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
longjohn unsigned backdoortimeout1 = 2000; // 2 seconds
// after this time, the gamestatus will reset - this will need to be prolonged so it can only be triggered on external power
// and/or when the box is programmed with cable when opened.
longjohn unsigned backdoortimeout2 = 25000; // 25 seconds 
// keeps track of when the backdoor switch is triggered                                     
longjohn unsigned backdoortimerstart;

// is the backdoor timer running?
int backdoortimerrunning = 0;
// set to 1 when the box is open
int boxopen = 0;
// set to 1 when the backdoor timer reaches backdoor timer2 and this is still 0 - is then being set to 1 to trigger the actual reset
int gamereset = 0;

long int start; // timer for letting the gps-object 'feed' more than once
longjohn unsigned mastertimerstart; // the master timer is used to see if we're timing out on the GPS signal
// mastertimer = timeout
long unsigned int timeout = 70000; // how long will we wait for the GPS (should be 1½ minute or so)?
int timeoutreached = 0;
int powermessage = 0; // keeps track of wether or not we've displayed a message after timeout and shutdown should have occured (displayed when on external power)

int debug = 0; // enables serial debug + other things

/* GPS-related variables */
float flat, flon;
unsigned long age, date, time;
int Year;
byte Month, Day, Hour, Minute, Second;
const int offset = 2; // we're in UTC+2 now

// coordinates for the various locations for the puzzle box to go
static const float FAABORG_LAT = 55.094878, FAABORG_LON = 10.237732;
static const int FAABORG_THRESHOLD = 1000; // required minimum distance to Faaborg havn

static const float DAD_LAT = 55.354605, DAD_LON = 10.728463;
static const int DAD_THRESHOLD = 500; // required minimum distance to DAD 

static const float MOM_LAT = 56.279952, MOM_LON = 9.434583;
static const int MOM_THRESHOLD = 500;

// dateline to cross before the box can be opened
int cutyear = 2013;
int cutmonth = 8;
int cutday = 3;
int cuthour = 15;
int cutminute = 30;
time_t rightnow;
time_t cutoff;

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
 if (debug) {
   Serial.begin(115200);
   Serial.print(getPSTR("Free RAM = ")); // forced to be compiled into and read from Flash
   Serial.println(freeMemory(), DEC);  // print how much RAM is available. 
 }

// initiate tm
tmElements_t tm;
tm.Second = 0;
tm.Minute = cutminute;
tm.Hour = cuthour;
tm.Wday = NULL;
tm.Day = cutday;
tm.Month = cutmonth;
tm.Year = cutyear-1970;
cutoff = makeTime(tm);


 // set up the LCD's number of columns and rows: 
 lcd.begin(20, 4);

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
   Serial.print(getPSTR("Initial gamestate: "));
   Serial.println(gamestate);
   delay(1000);
   Serial.print(getPSTR("Initial tasknr: "));
   Serial.println(tasknr);
   delay(1000);
 }

 // Print a message to the LCD.
 // to be moved when the box goes live
 // or maybe print gamestatus + welcome (back)
 // for now I am making sure only to print welcome when the game is in the running state
 if (gamestate == 1) {
   // stringToLcd is explained below.
   if (tasknr == 0) { stringToLCD(getPSTR("Welcome!")); }
   else { stringToLCD(getPSTR("Welcome back!")); }
   if (debug) {
      lcd.setCursor(0,1);lcd.print(getPSTR("tasknr: "));lcd.print(tasknr);
   }
   delay(2000);
   stringToLCD(getPSTR("Getting signal..."));
 }

 /* other settings */
 // we're using the internal PULLUP-resistor
 // when the pin is pulled LOW (to GND) things happen.. (backdoor timer)
 pinMode(BACKDOORPIN,INPUT_PULLUP);
 digitalWrite(BACKDOORPIN,HIGH);
 
 // debug-LED. Should probably be disabled when we go "live" as noone can see it anyway
 pinMode(13,OUTPUT);
 if (debug) { digitalWrite(13,LOW); }

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
   if (myservo.attached() && millis()-servostart > 4000) { 
    myservo.detach();
    delay(10); // small delay added for safety although tests so far have been good
    servoattached = 0;
    nss.begin(9600); // the servo has been detached, now we can re-enable GPS
    gpsattached = 1;
   } 

   if (gpsattached) {
          // feed a few times, to get a good fix, but only if attached - we start feeding up to half a second to make sure we get a fix and 
          // date/time information
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
     if (debug) {
         Serial.println(getPSTR("backdoor timer started"));
     }
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
       Serial.println(getPSTR("box is opened"));
     delay(5000); // I have this much time to open the box - the countdown is 'frozen' during this time (but still runs)
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
    delay(3000);
    gamestate = 0;
    tasknr = 1;
    if (debug) {
      Serial.println(getPSTR("box has been reset"));
      Serial.print(getPSTR("Gamestate changed: "));
      Serial.println(gamestate);
    }
    //digitalWrite(POLULUPIN,HIGH); // try and turn off, even though I know this is in vain on aux power
    delay(100);
   } 
   if (gpsattached) {
         // feed a few times, to get a good fix, but only if attached - we start feeding up to half a second to make sure we get a fix and 
         // date/time information
         feedgps();
         gps.f_get_position(&flat, &flon, &age);
   }
   if (debug) {
//     Serial.print("Gamestate (pre gs0): ");
//     Serial.println(gamestate);
   }


   // here we do a switch on the gamestate to decide what to do
   switch(gamestate) {
    case 0: { // game has just been reset - lock the box and shut down - states have been written above
       if (debug) {
         Serial.print(getPSTR("Gamestate (gs0): "));
         Serial.println(gamestate);
       }
       stringToLCD(getPSTR("box locked and reset"));
       delay(3000);
       stringToLCD(getPSTR("Good Luck!"));
       delay(2000);
       if (debug) { Serial.println(getPSTR("gs should now change to 3")); }
       digitalWrite(POLULUPIN,HIGH); // try and turn off (but it's still on aux power, I know ))
       delay(100);
       gamestate = 3; } 
       break; 
    // THE GAME STARTS HERE FOR REAL
    case 1: // the game is running
          // the count down thingy while we're searching for signal
          // - should probably be disabled when we're changing locations.. 
          // maybe a 'showcountdown'-variable that is set to off just before the last step?
        if (millis()-mastertimerstart <= timeout) {
            // print current millis on the bottom of the LCD
            // timeout-millis()
            // only if we're not timed out
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
            if (millis()-previousMillis >= interval && remaindertime >= 0 && !timeoutreached) {
              lcd.setCursor(0,3);
              if (m < 10) { lcd.print("0"); }
              lcd.print(m); 
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
                Serial.println(getPSTR("Timeout reached - good bye!"));
              delay(2000);
              // pull polulu high etc.
              digitalWrite(POLULUPIN,HIGH);
              // set gamestate to the limbo-state where we just wait for power to go (or the reset)
              timeoutreached = 1;
            } else if (timeoutreached && !powermessage){
              // wait a while then ask to have power removed - we have to set a flag so this step is detected alone
              gamestate = 3;
              if (debug) {
                 Serial.print(getPSTR("Gamestate changed: "));
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
            // feed a few times, to get a good fix, but only if attached - we start feeding up to half a second to make sure we get a fix and 
            // date/time information
            feedgps();
            gps.f_get_position(&flat, &flon, &age);
            // this is the last run of feedgps before we check for fix etc..
            // if age is < 1000, we have a fix - so we run feedgps for another 1000ms, to make sure we have
            // date and time as well (it won't work otherwise)
            //if (age < 1000), then we probably have a fix and so we find the current time etc.
            if (age < 1000) {
              updatedatetime(); // runs for 1000ms to make sure we have date-time correct
              gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age); 
              setTime(Hour, Minute, Second, Day, Month, Year);
              adjustTime(offset * SECS_PER_HOUR);
              rightnow = now();
            }
        }
        switch(tasknr) {
            case 0: // welcome message and first mission - we're not showing this unless we actually have a GPS fix
                  if (age < 1000) { 
                      unsigned long distance = gps.distance_between(flat,flon,FAABORG_LAT,FAABORG_LON);
                      // are we within 1000m? (could probably be set lower to make it more exciting)
                      // are we within threshold?
                      if (distance < FAABORG_THRESHOLD) {
                         stringToLCD(getPSTR("You made it to...   Faaborg. Was the ice cream good?")); 
                         delay(5000);
                         stringToLCD(getPSTR("Remember to take    pictures."));
                         delay(5000);
                         stringToLCD(getPSTR("Stand by for your next mission"));
                         delay(5000);
                         tasknr++;
                         EEPROM.write(1,tasknr);
                         mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                           stringToLCD(getPSTR("Go to Faaborg and   eat an ice cream on the harbour."));
                           delay(5000);
//                           lcd.clear();
                           lcd.setCursor(0,3);
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
                      unsigned long distance = gps.distance_between(flat,flon,DAD_LAT,DAD_LON);
                      if (distance < DAD_THRESHOLD) {
                         stringToLCD(getPSTR("You've made it to.. dads house")); 
                         delay(5000);
                         stringToLCD(getPSTR("I hope the car is   still in one piece?")); 
                         delay(5000);
                         stringToLCD(getPSTR("Stand by for your   next mission"));
                         delay(5000);
                         tasknr++;
                         EEPROM.write(1,tasknr);
                         mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                           stringToLCD(getPSTR("Go and visit dad -  The one on the same island as you are on"));
                           delay(5000);
                           lcd.setCursor(0,3);
                           lcd.print(getPSTR("Good Bye"));
                           delay(5000);
                           digitalWrite(POLULUPIN,HIGH);
                           delay(100);
                           gamestate = 3; // switch to message if running on external power
                      }
                  }
            break; 

            case 2: // 3rd mission
                  if (age < 1000) { 
                      unsigned long distance = gps.distance_between(flat,flon,MOM_LAT,MOM_LON);
                      if (distance < MOM_THRESHOLD) {
                         stringToLCD(getPSTR("You've made it to.. moms house")); 
                         delay(5000);
                         stringToLCD(getPSTR("Did you take the carall this way??")); 
                         delay(5000);
                         stringToLCD(getPSTR("Stand by for your   next mission"));
                         delay(5000);
                         stringToLCD(getPSTR("You're getting closeto the end"));
                         delay(5000);
                         tasknr++;
                         EEPROM.write(1,tasknr);
                         mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                           stringToLCD(getPSTR("Go and visit mom -  The one in the same country as you are in"));
                           delay(5000);
                           lcd.setCursor(0,3);
                           lcd.print(getPSTR("Good Bye"));
                           delay(5000);
                           digitalWrite(POLULUPIN,HIGH);
                           delay(100);
                           gamestate = 3; // switch to message if running on external power
                      }
                  }
            break; 

            case 3: // 4th mission
                  if (age < 1000) { 
                    // test on the date - are we on or after the correct date, go ahead an open
                    // - otherwise tell the user to wait for the correct date
                    // we alreday have the date set in the Time module (.. somewhere in the belly of the arduino)
                    // so we need to see if the year, month and date is correct - this is done by converting to UXtime
                    if (rightnow > cutoff) {
                        
                        stringToLCD(getPSTR("Congratulations!    You should now be   married!"));
                        delay(5000);
                        stringToLCD(getPSTR("You can now show    people this box and tell them about youradventures.."));
                        delay(5000);
                        stringToLCD(getPSTR("Before you can open this box there is   something you shouldknow.."));
                        delay(5000);
                        stringToLCD(getPSTR("When I open, we will have to part ways..."));
                        delay(5000);
                        stringToLCD(getPSTR("but you will get    something much  bet-ter in return"));
                        delay(5000);
                        stringToLCD(getPSTR("Are you ready...?"));
                        delay(5000);
                        stringToLCD(getPSTR("In any case, the boxwill now open."));
                        delay(5000);
                        stringToLCD(getPSTR("The display will    blink slightly and  you will hear a buz-zing sound."));
                        delay(5000);

                        lcd.clear(); lcd.setCursor(0,0);lcd.print("Tillykke");
                        gamestate = 2;
                        EEPROM.write(0,gamestate); // setting gs to 2
                        delay(300);
                        mastertimerstart = millis(); // resetting time just in case the GPS-signal dies for a bit
                      } else {
                        stringToLCD("You can open this.. box after you are.. married");
                        delay(5000);
                        //lcd.clear();
                        lcd.setCursor(0,3);
                        lcd.print("Good Bye");
                        delay(5000);
                        digitalWrite(POLULUPIN,HIGH);
                        gamestate = 3; // switch to message if running on external power
                      }
                  }
            break; 
        }
        break;
    case 2: // the game is over - time to open the box 
         if (debug) {
           Serial.print(getPSTR("Gamestate (gs2): "));
           Serial.println(gamestate);
        }
        nss.end(); //stop talking to the GPS
        delay(250);
        gpsattached = 0;  
        unlockbox();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("The box is now open");
        lcd.setCursor(0,1);
        lcd.print("Congratulations!");
        lcd.setCursor(0,2);
        delay(3000);
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
           Serial.print(getPSTR("Gamestate (gs3): "));
           Serial.println(gamestate);
        }
        
       if (debug)
         Serial.println(getPSTR("Please remove power"));

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
 if (debug) { digitalWrite(13,LOW); }
}

void unlockbox() {
 servoattached = 1;
  if (!myservo.attached()) { myservo.attach(SERVOPIN); }

// myservo.slowmove(90,70);
 myservo.write(90);
 servostart = millis();
 delay(500);
 if (debug) { digitalWrite(13,HIGH); }
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

// this functions takes a second to complete, but is only called when the 'age' paramater returned GPS is less than 1000 
// (when we have a fix)
void updatedatetime() {
         start = millis();
         while (millis() - start < 1000) {
            feedgps();
         }
}
