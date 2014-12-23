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
 lcd.setBacklight(HIGH);
 lcd.setBacklight(LOW);
 lcd.clear();
 lcd.home();
 lcd.setCursor(positie, regel);
 convert floats to strings: 
 float myFloat;
 char buffer[5];
 String s = dtostrf(myFloat, 1, 4, buffer);
 But Serial.print() sends floats as strings too
 ========================================================= */
/*
 abbreviations
 */

/********************************************************************** 
*  Import needed libraries                                            *
*  LiquidCrystal_I2C is from F. Malpertida                            *
*  dht library http://arduino.cc/playground/Main/DHTLib  Rob Tillaart *
*  RTClib from Adafruit                                               *
*  If the library is in the sketch folder change #include <name> to   *
*  #include "name"                                                    *
*  this tells the compiler to look in the sketch directory first.     *
***********************************************************************/
//---------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h> //Malpertida 
#include <dht.h> 
#include "RTClib.h"   //Adafruit


/*************************************************************
*       Declare objects                                      *
*       RTC                                                  *
*       DHT                                                  *
*       LCD                                                  *
**************************************************************/
//RTC_DS1307 rtc;

RTC_DS1307 RTC; //declare object RTC
dht DHT;        //declare object DHT11
//*************************************************************
// ------ ( declare LCD data)-------------------------        *
// set the LCD address to 0x27 for a 20 chars 4 line display  *
// Set the pins on the I2C chip used for LCD connections:     *
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol     *
//*************************************************************
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
//

/*-----( Declare pins )------*/
//analogue pins
byte moisturePin=0; //read soil moisture
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

#define DHT11_PIN humidPin   // in fact a bit double
#define DS1307_I2C_ADDRESS 0x68 // each I2C object has a unique bus address, the DS1307 is 0x68

/* Variable setting   */
// variables -placeholders for various readings
volatile byte seconds;
byte push=0;               // contains the state of the pushbutton
byte sButton=0;            // contains the state of the throw switch
byte light;                // contains the value for LDR reading 
byte pump=0;               // contains the pump state
unsigned int moist=0;      // contains the soil moisture level
unsigned int irrigate=0;   // contains the set level for irrigation in case no potmeter
byte level=0;              // contains level of the waterlevel (HIGH or LOW)
int c;                     // contains the temperature reading
boolean ready_to_read=1;   // is set every 4 minutes
// Constants
// these constants won't change:
const int sensorMin = 40;      // LDR minimum, discovered through experiment
const int sensorMax = 1012;    // LDR maximum, discovered through experiment
//************************************************************
//---------------- Define Special Characters-----------------*
// Create a set of new characters                            *
// 0=fan                                                     *
// 1=degree sign                                             *
// 2=termometer                                              *
// 3=waterdruppel                                            *
// 4=spikes                                                  *
// 5=pump                                                    *
// 6=up arrow                                                *
// 7=down arrow                                              *
//************************************************************
const uint8_t charBitmap[][8] = {
  { 
    B00000,
    B10001,
    B01010,
    B00100,
    B01010,
    B10001,
    B00000,
    B00000           }
  ,
  { 
    0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0           }
  ,
  { 
    B00100,
    B01010,
    B01010,
    B01110,
    B01110,
    B11111,
    B11111,
    B01110           }
  ,
  { 
    B00100,
    B00100,
    B01010,
    B01010,
    B10001,
    B10001,
    B10001,
    B01110          }
  ,
  { 
    B10001,
    B10001,
    B11111,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001           }
  ,
  { 
    B01010,
    B00100,
    B00100,
    B00100,
    B11111,
    B11111,
    B11111          }
  ,
  { 
    0x0, 0x4, 0xE, 0x15, 0x4, 0x4, 0x4, 0x0           }
  ,
  { 
    0x4,0x4,0x4, 0x4, 0x15, 0xE,0x4,0x0           }
};
/// -------- end creation--------------
//The following function, "setup", must always be present
void setup()
{  
  
  /* **********************************************************
   ** The timer is being set up in order to have a 4 minute   *
   ** event in the ISR(TIMER1_COMPA_vect) routine for reading *
   ** of a sensor... should one wish that                     *
   ************************************************************/
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 15624;

  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);

  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);

  // enable global interrupts:
  sei();
  //---------------end timer initialisation                   *
  //***********************************************************
  Serial.begin(115200);
  //*********************************************************** 
  // Set up the RTC                                           *
  //***********************************************************
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  RTC.begin();  // Start RTC
  /*
  if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
   // following line sets the RTC to the date & time this sketch was compiled
   // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   // This line sets the RTC with an explicit date & time, for example to set
   // January 21, 2014 at 3am you would call:
   // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
   }
   */
  //above code replaced by:
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    Serial.println("RTC is older than compile time! Updating");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //----Set sqw to 1 Hz---
  // no practical function yet
  Wire.beginTransmission(0x68);
  Wire.write(0x07); //move pointer to SQW address
  Wire.write(0x10); // sends 0×10 (hex) 00010000 (binary)
  Wire.endTransmission();
  //---end sqw-----
  //*********************************************************
  // Set up the in and output pins                          *
  //*********************************************************
  pinMode(levelPin,INPUT);  // set level
  pinMode(humidPin,INPUT);  // measures humidity
  pinMode(emptyPin,INPUT);  // measures reservoir
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
  //********************************************************
  //---------------Set up LCD                              *
  //  initialize the lcd for 16 chars 2 lines              *
  //  load  the  character definitions                     *
  //  print some messages and blink the backlight          *
  //********************************************************
  lcd.begin(16,2);         
  //upload defined characters to LCD
  int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));
  for ( int i = 0; i < charBitmapSize; i++ )
  {
    lcd.createChar ( i, (uint8_t *)charBitmap[i] );
  }
  //---------------end upload----------------
  lcd.backlight();   
  // Print a message to the LCD.
  lcd.setCursor(0, 0);// topleft
  lcd.print("Greenhouse");
  // ------- Quick 2 blinks of backlight  -------------
  flash(2); 
  // ------- Quick buzz--------------------------------
  // buzz(1);
  //************************************************************ 
  // Start  RTC                                                *
  //************************************************************
  Wire.begin(); //needed for RTC, not for LCD
  RTC.begin();
  /* Set the date / time to the time this program was compiled.
   Comment this OUT, AND upload, to let the clock just run.  */
  //  RTC.adjust(DateTime(__DATE__, __TIME__));
  RTC.writeSqwPinMode(SquareWave1HZ);// double?

  //************************************************************ 
  // Check buttons at startup                                  *
  //************************************************************
  if(digitalRead(PushButton)==0){
    buzz(3);
  }

}
/*----------------------------(end setup )---------------------*/

/* *************************************************************
 *                   Main loop                                 *
 *  get and print  Date/time                                   *
 *  Check if the reservoir is full                             *
 *  Read soil humidity                                         *
 *  Switch pump                                                *
 *  Read Air Humidity and temperature                          *
 *  Read light                                                 *
 *  Display data                                               *
 *  Switch heating on or off                                   *
 *  Switch fan on or off                                       *
 ***************************************************************/
void loop() 
{
  DateTime now = RTC.now();  //Get the current data
  Serial.print(now.day(), DEC);
  Serial.print("-");
  Serial.print(now.month(), DEC);
  Serial.print("-");
  Serial.print(now.year(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute()<10)
  {
    Serial.print("0");
  }
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  //Serial.print(now.second(), DEC);
  if (now.second()<10)
  {
    Serial.print("0");
  }
  Serial.println(now.second(), DEC);
  //------------end of RTC
  /********************************************************************************
  *  ---------- 1. Check if there is enough water -------------                   *
  *  check if there is water                                                      *
  *  if not sound the buzzer                                                      *
  *  later in the program the pump will be disabled if water level too low        *    
  *********************************************************************************/
  level=  digitalRead(emptyPin);//Switch closed = empty
  if (level==0) {
    digitalWrite(buzPin, HIGH);
    delay (50);
    digitalWrite(buzPin, LOW);
    delay(500);
  }
  /*********************************************************************************
  * ------------2. Read the soil moisture content/switch pump----------------      *
  * First read the level set with P1 on the levelPin and store that in 'irrigate'  *
  * Then we read the soil humidity sensor.                                         *
  * We'll first have to set the spikePin to HIGH, in case that is used to feed     *
  * the sensor. After the reading we set it back)                                  *
  * If the value read ('moist') is smaller than what is considered dry ('irrigate')*
  * then the pump should be switched on for a specific time.                       *
  * This is done by indicating a higher treshhold for switching the pump off       *
  * Could be made conditional by testing the ready_to_read boolean                 *
  * that is set every 4 minutes in order to spare the spikes                       *
  *                                                                                *
  **********************************************************************************/
  irrigate=sample(levelPin);
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
    ready_to_read=0; // needed to read onece every 4 min
    
  }
  /*********************************************************************************
  *-------------3. test the DHT11 humidity/temp sensor-----------------            *
  **********************************************************************************/
  
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
    //   Serial.print("Time out error,\t"); 
    break;
  default: 
    Serial.print("Unknown error,\t"); 
    break;
  }
  /*********************************************************************************
  *-------------4. Read  LDR ----------------------------------------              *
  * no actions for the time being, but can be used to e.g. only irrigate at night  *
  * or to add artificial light to the day                                          *
  **********************************************************************************/
  light=Map(LDRPin);

  /* ------------------Actions -------------------*/
  /*********************************************************************************
  *-------------5.  DISPLAY DATA ------------------------------------              *
  **********************************************************************************/
  // Serial.print(DHT.humidity,1);
  // Serial.print(",\t \t");
  //Serial.println(DHT.temperature,0);
  /*   Display data on LCD  */
  push=digitalRead(PushButton);
  if (push==1) // pushbutton not pressed
  {
    lcd.clear();
    lcd.setCursor(0, 0);  //set cursor left on first line (line 0)
    lcd.print(char(2)); //prints thermometer
    lcd.print(" ");
    lcd.print((float)DHT.temperature, 0);
    lcd.print (char(1));  // prints degree sign
    lcd.print("C");
    lcd.print(" ");
    lcd.print(char(3));// droplet
    lcd.print(" ");
    lcd.print((float)DHT.humidity, 0);
    lcd.print("%");
    lcd.print(" ");
    lcd.print (char(7-level));//prints up or down arrow (char 7 or 6)
    if(level+pump==2){ 
      lcd.print(" ");
      lcd.print(char(5));
    } 
    else { 
      lcd.print(" ");
    }

    lcd.setCursor(0,1);// set cursor left on line 1 (=2nd line)
    lcd.print(char(4)); // spikes
    lcd.print(" ");
    lcd.print(moist); 
    lcd.print("/");
    lcd.print(irrigate);
    // lcd.print("=>");
    // lcd.print((float)moist/irrigate);//integer kan contain no fraction so use the cast-operator to make it float


  }
  if (push==0)   // pushbutton pressed
  {
    lcd.clear();
    lcd.setCursor(0,0);//topleft
    //--------------
    DateTime now = RTC.now();
    lcd.print(now.day(), DEC);
    lcd.print('-');
    lcd.print(now.month(), DEC);
    lcd.print('-');
    lcd.print(now.year(), DEC);
    lcd.print(" ");
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if (now.minute()<10)
    {
      lcd.print("0");
    }
    lcd.print(now.minute(), DEC);
    /* geen seconden
     lcd.print(':');
     if (now.second()<10)
     {
     lcd.print("0");
     }
     lcd.print(now.second(), DEC);
     //---end no secs
     */
    //---end rtc


    //-----------
    // lcd.print("          ");
    lcd.setCursor(0,1);
    lcd.print("licht niv.: ");
    lcd.print(analogRead(LDRPin));
    //buzz(1);
  }

 /**************************************************
  * now we measure temperature and air humidity    *                               *
  * and if necessary switch on heating or fan      *
  * temp kept at minimally 20 degrees              *
  * humidity is kept below 60%                     *
  **************************************************/
  // ---------------5. Action on temperature ------

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
  if (DHT.humidity >=60)
  {
    // zet ventilator aan
    digitalWrite(fanPin, HIGH);
    // print fan-sign only if pushbutton not pressed
    if(push==1){
      lcd.setCursor(10,1);
      lcd.print(char(0)); // fan sign
    }
  }
  else
  {
    digitalWrite(fanPin,LOW);
    // print fan-sign only if pushbutton not pressed
    if(push==1){
      lcd.setCursor(10,1);
      lcd.print(' ');
    }
  }

  delay(500);


  //end dht
  //end loop
}
//-------  End of Main program-----------
//
//-------- Start of functions------------
int sample(int z)
/**********************************************************************
* This function will read the Pin 'z' 5 times and take an average.    *
* Afterwards it will be mapped to 8 bits by dividing by 4             *
* Could ofcourse immediately divided by 20                            *
***********************************************************************/
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
/**********************************************************************
* This function willflash the backlight a number of times             *
***********************************************************************/
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
/**********************************************************************
* This function will sound the buzzer "u" times                       *
***********************************************************************/
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
/****************************************************************************
* This function will blink an LED a number of times for a specific duration *
*****************************************************************************/
void ledblink(int times, int lengthms, int pinnum){
  for (int x=0; x<times;x++){
    digitalWrite(pinnum, HIGH);
    delay (lengthms);
    digitalWrite(pinnum, LOW);
    delay(lengthms);
  }
}
/**********************************************************************
* This function maps the LDR reading into nrs 0-3                     *
***********************************************************************/
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
/**********************************************************************
* This function adds a leading zero                                   *
***********************************************************************/
void printDigits(int digits) { //this adds a 0 before single digit numbers
  //if (digits >= 0 && digits < 10) {
  if (digits <10) {
    lcd.print('0');
    Serial.print('0');
  }
  lcd.print(digits);
  Serial.print(digits);
}
/**********************************************************************
* This  function converts Celsius to Fahrenheit                       *
***********************************************************************/
float tempConvert(float celsius)
{
  float fahrenheit = 0;
  fahrenheit = (1.8 * celsius) + 32;
  return fahrenheit; 

}
/**********************************************************************
* This function writes bytes to RTC non volatile RAM                  *
***********************************************************************/

byte nvrWrite(byte a,byte lstat){
  a=a+7;
  Wire.beginTransmission(0x68);
  Wire.write(a); // move pointer to RAM address
  Wire.write(lstat);// send 1
  Wire.endTransmission();
}
/**********************************************************************
* This function reads bytes from RTC non volatile RAM                 *
***********************************************************************/
byte nvrLees(byte a){
  a=a+7;
  byte lstat;
  Wire.beginTransmission(0x68);
  Wire.write(a); // move pointer to RAM address

  Wire.endTransmission();
  Wire.requestFrom(0x68,1);
  lstat=Wire.read();
  //Serial.println(lstat);
  return lstat;
}

/**********************************************************************
* This is the timer interrupt function that sets a flag every 4 min   *
***********************************************************************/
ISR(TIMER1_COMPA_vect)
{
  seconds++;
  if(seconds == 240)
  {
    seconds = 0;
    // leest elke 240 seconden (4 minuten) een sensor
    //readSensor();
    moist=sample(moisturePin);  //read soil humiditySensor
    // of kies ervoor een flag te zetten
    ready_to_read=1;
  }
}
/**********************************************************************
* This function calculates dewpoint                                   *
***********************************************************************/
double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity/100);
  double Td = (b * temp) / (a - temp);
  return Td;
}

/**********************************************************************
* These are various other user defined symbols                        *
***********************************************************************
/*
//icon for solar panel
 {
 0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000
 };
 //icon for battery
 {
 0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111
 };
 
 // icon for power
 {
 0b00010,0b00100,0b01000,0b11111,0b00010,0b00100,0b01000,0b00000
 };
 // icon for alarm
 {
 0b00000,0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100
 };
 //icon for termometer
 {
 0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110
 };
 
 // icon for battery charge
 {
 0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100,
 };
 // icon for not charge
 {
 0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000,
 };
 
 */

