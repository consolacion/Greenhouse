/* =======================================================
 meet de vochtigheid
 als minder dan ingestelde waarde, pomp aan
 meet temperatuur
 als lager dan ingestelde waarde, verwarming aan
 meet humidity
 als hoger dan ingestelde waarde zet ventilator aan
 ventilator ook aan snachts
 http://www.quickgrow.com/gardening_articles/fans_air_movement.html
 LCD commands:
 lcd.setBacklight(LED_OFF);
 lcd.noBacklight();
 lcd.clear();
 lcd.home();
 convert floats to stings: 
 float myFloat;
char buffer[5];
String s = dtostrf(myFloat, 1, 4, buffer);
But Serial.print() sends floats as strings too
 ========================================================= */
 /*
 abbreviations
 */
 
//--------------------------------------- 
/*-----( Import needed libraries )-----*/
//---------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //Malpertida If the library is in the sketch folder change #include <LiquidCrystal_I2C> to #include "LiquidCrystal_I2C" this tells the compiler to look in the sketch directory first.
#include <dht.h>
#include "RTClib.h"   //Adafruit
RTC_DS1307 rtc;

//-------------------------------
/*-----( Declare objects )-----*/
//-------------------------------
// first the humidity sensor
dht DHT;
// ------ ( declare LCD data)-------------------------
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
//
/*-----(Declare RTC objects )------*/
RTC_DS1307 RTC; //declare object

// -------------------------
// >> pin definitions
// -------------------------
// this is where you define what pins you'll be using

/*-----( Declare pins )------*/
//analogue pins
byte moisturePin=0; //read soil mositure
byte LDRPin=3;// analog pin for LDR

//digital pins
byte levelPin= 1; //set level for  irrigation
byte humidPin=2;  //Humidity sensor
byte lightPin=5;  //Switches on light
byte fanPin=6;    //Switches fan
byte pumpPin =7;  //Switches pump
byte hotPin= 8;   //Switches heater
byte buzPin=9;    //Switches buzzer
byte emptyPin=10; //digital  Pin10  guards waterlevel
byte spikePin=12; //Digital  Pin12  ->for intermittent switching of spike
byte PushButton=4;   // PushButton
byte SwitchButton=3;// Make Switch
#define DHT11_PIN humidPin

/* Variable setting   */
// variables -placeholders for various readings
byte push=0;               // contains the state of the pushbutton
byte sButton=0;            // contains the state of the throw switch
byte light;                // contains the value for LDR reading 
byte pump=0;               // contains the pump state
unsigned int moist=0;      // contains the soil moisture level
unsigned int irrigate=0;   // contains the set level for irrigation in case no potmeter
byte level=0;              // contains level of the waterlevel (HIGH or LOW)
int c;                     // contains the temperature reading
// Constants
// these constants won't change:
const int sensorMin = 40;      // LDR minimum, discovered through experiment
const int sensorMax = 1012;    // LDR maximum, discovered through experiment

//---------------- Define Special Characters-----------------
// Create a set of new characters
// 0=fan
// 1=degree sign
// 2=termometer
// 3=waterdrupp
// 4=spikes
// 5=pump
// 6=up arrow
// 7=down arrow
const uint8_t charBitmap[][8] = {
  { 
    B00000,
    B10001,
    B01010,
    B00100,
    B01010,
    B10001,
    B00000,
    B00000     }
  ,
  { 
    0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0     }
  ,
  { 
    B00100,
    B01010,
    B01010,
    B01110,
    B01110,
    B11111,
    B11111,
    B01110     }
  ,
  { 
    B00100,
    B00100,
    B01010,
    B01010,
    B10001,
    B10001,
    B10001,
    B01110    }
  ,
  { 
    B10001,
    B10001,
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001     }
  ,
  { 
B01010,
B00100,
B00100,
B00100,
B11111,
B11111,
B11111    }
  ,
  { 
    0x0, 0x4, 0xE, 0x15, 0x4, 0x4, 0x4, 0x0     }
  ,
  { 
    0x4,0x4,0x4, 0x4, 0x15, 0xE,0x4,0x0     }
};
/// -------- end creation--------------
//The following function, "setup", must always be present
void setup()
{  
  Serial.begin(115200);
  //----RTC-----
  #ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  //----end rtc-------
  //----Set sqw to 1 Hz---
Wire.beginTransmission(0x68);
Wire.write(0x07); //move pointer to SQW address
Wire.write(0x10); // sends 0×10 (hex) 00010000 (binary)
Wire.endTransmission();
//---end sqw-----
   pinMode(levelPin,INPUT);  // set level
  pinMode(humidPin,INPUT);  // measures humidity
  pinMode(emptyPin,INPUT);  // measures reservoir
  //digitalWrite(emptyPin, HIGH);       // turn on pullup resistors
  pinMode(SwitchButton, INPUT);   // make Switch
  pinMode(PushButton, INPUT);     // PushButton
  pinMode(spikePin,OUTPUT); // for alternative supply to spikes
  pinMode(pumpPin,OUTPUT);  // Output for Relay
  pinMode(fanPin, OUTPUT);  // Output for fan
  pinMode(hotPin, OUTPUT);  // Output for heater
  pinMode(lightPin, OUTPUT);// Output for light
  pinMode(buzPin, OUTPUT);  // Output for buzzer
  digitalWrite(pumpPin, LOW);// Pump off
  digitalWrite(spikePin, LOW);// moisture sensor off
  digitalWrite(fanPin,LOW);  // fan Off
  digitalWrite(hotPin,LOW);   // heater off
  digitalWrite(lightPin, LOW); // light Off
  digitalWrite(buzPin, LOW); // buzzer off
  /* Now LCD */
  //---------------Set up LCD------------------
  lcd.begin(16,2);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  //upload defined characters to LCD
  int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));
  for ( int i = 0; i < charBitmapSize; i++ )
  {
    lcd.createChar ( i, (uint8_t *)charBitmap[i] );
  }
   //---------------end upload----------------
  lcd.backlight();   
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Greenhouse");
  // ------- Quick 2 blinks of backlight  -------------
  flash(2); 
  // ------- Quick buzz--------------------------------
 // buzz(1);
  Wire.begin(); //needed for RTC, not for LCD
  RTC.begin();
  /* Set the date / time to the time this program was compiled.
   Comment this OUT, AND upload, to let the clock just run.  */
  //  RTC.adjust(DateTime(__DATE__, __TIME__));

}
/*----------------------------(end setup )---------------------*/

void loop() 
{
  DateTime now = rtc.now();  //Get the current data
 Serial.print("Het jaar is ");
 Serial.print(now.year(), DEC);
 Serial.print(" Maand = ");
 Serial.print(now.month(), DEC);
 Serial.print(" Dag = ");
 Serial.print(now.day(), DEC);
 Serial.print(" Tijd = ");
 Serial.print(now.hour(), DEC);
 Serial.print(':');
 Serial.print(now.minute(), DEC);
 Serial.print(':');
 Serial.print(now.second(), DEC);
 Serial.println();
  //------------end of RTC
  // ---------- 1. Check if there is enough water -------------  
  // check if there is water 
 //  if not sound the buzzer
 //  later in the program the pump will be disabled 
  level=  digitalRead(emptyPin);//Switch closed = empty
  if (level==0) {
    digitalWrite(buzPin, HIGH);
    delay (50);
    digitalWrite(buzPin, LOW);
    delay(500);
  }
  //------------2. Read the soil moisture content/switch pump----------------
  /*
First read the level set with P1 on the levelPin and store that in 'irrigate'
   */
  irrigate=sample(levelPin);
  /*

   Then we read the soil humidity sensor.
   We'll first have to set the spikePin to HIGH, in case that is used to feed the sensor. After the reading we set it back) 
   If the value read ('moist') is smaller than what is considered dry ('irrigate') then the pump should be switched on for a specific time. 
   This is done by indicating a higher treshhold for switching the pump off
   */
  digitalWrite(spikePin, HIGH);// put voltage on the  humidity sensor
  delay(100); //wait a short while
  moist=sample(moisturePin);  //read soil humiditySensor
  //
  digitalWrite(spikePin, LOW);
  if (moist <= irrigate){
    pump=1;
    digitalWrite(pumpPin, level); // if the reservoir is empty a 0 will be written
  }
  if (moist >= irrigate+5) {
    pump=0;
    digitalWrite(pumpPin, LOW); // prevents  Jitter
  }

  //-------------3. Read the DHT11 humidity/temp sensor-----------------
  // now we measure temperature and air humidity
  // READ DATA
  // Serial.print("DHT11, \t");
  int chk = DHT.read11(DHT11_PIN);
  switch (chk)
  {
  case DHTLIB_OK:  
    //	Serial.print("OK,\t"); 
    break;
  case DHTLIB_ERROR_CHECKSUM: 
    Serial.print("Checksum error,\t"); 
    break;
  case DHTLIB_ERROR_TIMEOUT: 
    Serial.print("Time out error,\t"); 
    break;
  default: 
    Serial.print("Unknown error,\t"); 
    break;
  }

  //-------------4. Read  LDR ----------------------------------------
  light=Map(LDRPin);
  
   /* ------------------Actions -------------------*/

  //-------------5.  DISPLAY DATA ------------------------------------
  // Serial.print(DHT.humidity,1);
  // Serial.print(",\t \t");
  //Serial.println(DHT.temperature,0);
  /*   Display data on LCD  */
  push=digitalRead(PushButton);
  if (push==1) // pushbutton not pressed
  {
    lcd.clear();
    lcd.setCursor(0, 0);  //set cursor on first line (line 0)
    //lcd.print("Temp.   : ");
    lcd.print(char(2)); //prints thermometer
    lcd.print(" ");
    lcd.print((float)DHT.temperature, 0);
    lcd.print (char(1));  // prints degree sign
    lcd.print("C");
    lcd.print(" ");
    
    Serial.print(level);
    //  Serial.print("Temp. (oC): ");
    //  Serial.println((float)DHT.temperature,2);
    //lcd.setCursor(0,1);  //set cursor on 2nd line
    //lcd.print("Humidity: ");
    lcd.print(char(3));// droplet
    lcd.print(" ");
    lcd.print((float)DHT.humidity, 0);
    lcd.print("%");
    lcd.print(" ");
    lcd.print (char(7-level));//prints up or down arrow (char 7 or 6)
    if(level+pump==2){ lcd.print(" ");
    lcd.print(char(5));
    } else { lcd.print(" ");}
    // Serial.print("Humidity (%): ");
    //Serial.println((float)DHT.humidity, 2);  
    //delay(1000);  // wait for one second and then print the  soilmoisture
    lcd.setCursor(0,1);
    lcd.print(char(4)); // spikes
    lcd.print(" ");
    lcd.print(moist); 
    lcd.print("/");
    lcd.print(irrigate);
    //lcd.print ("Irr. Level: ");
   // lcd.print(irrigate);
   // lcd.setCursor(0,0);
    //

   //lcd.print("Moisture: ");
   //lcd.print(moist); 
   // lcd.print("   ");

  }
  if (push==0)   // pushbutton pressed
  {
    lcd.clear();
    lcd.setCursor(0,0);
    //lcd.print("licht: ");
    //lcd.print(light);
    //--------------
    DateTime now = rtc.now();
    lcd.print(now.year(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.day(), DEC);
    //-----------
    lcd.print("          ");
    lcd.setCursor(0,1);
    lcd.print("licht niv.: ");
    lcd.print(analogRead(LDRPin));
    //buzz(1);
  }

 

  // ---------------5. Action on temperature ------
  //if ((DHT.temperature,0) =<20)
  c=(DHT.temperature);
//  Serial.println(c);
  if (c<=19)
    //Serial.println(c);
  {// switch on heating
    digitalWrite(hotPin,HIGH);
  }
  else
  {
    digitalWrite(hotPin,LOW);
  }
  //--------------6. Action on Humidity -----------
  if (DHT.humidity >=50)
  {
    // zet ventilator aan
    digitalWrite(fanPin, HIGH);
    lcd.setCursor(1,10);
    lcd.print(char(0)); // pomp sign
  }
  else
  {
    digitalWrite(fanPin,LOW);
    lcd.setCursor(1,10);
    lcd.print(' ');
  }

  delay(500);


  //end dht
  //end loop
}
//-------  End of Main program-----------
//
//-------- Start of functions------------
int sample(int z)
/* This function will read the Pin 'z' 5 times and take an average.
 Afterwards it will be mapped to 8 bits by dividing by 4
 Could ofcourse immediately divided by 20 but this way it is easier to follow the program
 */
{
  byte i;
  int sval = 0;
  for (i = 0; i < 5; i++){    
    sval = sval + analogRead(z);// sensor on analog pin 'z'
  }
  //sval = sval / 5;    // average
  //sval = sval / 4;    // scale to 8 bits (0 - 255)
  sval=sval / 20;
  return sval;
}
//------------- Flash the backlight a number of times--------
void flash(byte y)
{
  byte j;
  for (j=0; j<y;j++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  
  return;
}
// This function will sound the buzzer "u" times
void buzz(byte u)
{
  byte k;
  for (k=0; k<u;k++)
  {
    digitalWrite(buzPin, HIGH);
    delay(200);
    digitalWrite(buzPin, LOW);
    delay(200);
  }
  return;
}
// This function will blink an LED a number of times for a specific duration
void ledblink(int times, int lengthms, int pinnum){
  for (int x=0; x<times;x++){
    digitalWrite(pinnum, HIGH);
    delay (lengthms);
    digitalWrite(pinnum, LOW);
    delay(lengthms);
  }
}
// This function maps the LDR reading into nrs 0-3
int Map(byte sens) {
  // read the sensor:
  int sensorReading = analogRead(sens);
  // map the sensor range to a range of four options:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);
  // do something different depending on the
  // range value:
  switch (range) {
  case 0:    // 
  //  Serial.println("dark");
   // lcd.backlight();
    break;
  case 1:    // 
   // Serial.println("dim");
   // lcd.backlight();
    break;
  case 2:    // 
  //  Serial.println("medium");
   // lcd.noBacklight();
    break;
  case 3:    // 
  //  Serial.println("bright");
   // lcd.noBacklight();
    break;
  }
  return range;
}

void printDigits(int digits) { //this adds a 0 before single digit numbers
  //if (digits >= 0 && digits < 10) {
    if (digits <10) {
    lcd.write('0');
  }
  lcd.print(digits);
}

float tempConvert(float celsius)
{
  float fahrenheit = 0;
  fahrenheit = (1.8 * celsius) + 32;
  return fahrenheit; 

}
