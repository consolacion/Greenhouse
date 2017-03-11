/****************************************************************
 03-04-2016 added lcd.clear on pushbutton change
 03-04-2016 added  mintemp store slaat dagelijks laagst gemeten temp op
 27-03-2016 correctie Winter_zomer routine
 27-03-2016 waarde 255 van warmtetotaalopbrengst uitgesloten
 17-03-2016 printlogWarmth toegevoegd
 16-03-2016 slaat 1 jaar stooktijd in EEPROM op
 15-03-2016 upper limit naar 21 graden
 11-03-2016 added toggle voor tomaat
 09-03-2016 addedcheck for heat on around midnight
 08-03-2016 Alarm voor losse sensor toegevoegd
 08-03=2016 Added running warmtime and count
 07-03-2016 Correctie: lsb en dst waren beide 25
 06-03-2016 Lichtschema toegevoegd
 06-03-2016 start up optie voor tomaat
 06-03-2016 hernoemd: versie 26
 05-03-2016 warmcount toegevoegd, telt aantal keren dat de verwarming aangaat
 04-03-2016 corrected totalling, corrected print of seconds in lower right corner
 01-03-2016 added totalling time heating on
 29-02-2016 added setting for tomatoes: 22-20 in day time and 20-18 at night
 15-02-2016 Schakelt nu verwarming aan op 19  en uit op 22
 13-02-2016 Added dst
 29-11-2015 Added Winsen MH-Z19 CO2 sensor
 28-11-2015 lcd.clear was removed for less flickering in LCD. Necessary area's overwritten by Spaces
 21-11-2015 added  hydroponics setting
 19-11-2015 changed into once every 4 minutes for measurement
 23-11-2015 log functie toegevoegd
 measure soil moisture
 if less than required value switch pump on
 measure temperature
 if less than required value switch on heating
 measure humidity
 if higher than desired value Switch on fan
 Consider  leaving fan on at night
 http://www.quickgrow.com/gardening_articles/fans_air_movement.html
 CO2 700-1400ppm  atmosfeer is ca 395ppm
 http://co2now.org/
 http://www.omafra.gov.on.ca/english/crops/facts/00-077.htm
 MG811 http://sandboxelectronics.com/?p=147
 http://sandboxelectronics.com/?product=mh-z16-ndir-co2-sensor-with-i2cuart-5v3-3v-interface-for-arduinoraspeberry-pi
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
 uno has 1k EEPROM
 ========================================================= */

//sudo ln /dev/ttyS0 /dev/ttyACM9
/**********************************************************************
*  Import needed libraries                                            *
*  LiquidCrystal_I2C is from F. Malpartida                            *
*  dht library http://arduino.cc/playground/Main/DHTLib  Rob Tillaart *
*  RTClib from Adafruit                                               *
*  If the library is in the sketch folder change #include <name> to   *
*  #include "name"                                                    *
*  this tells the compiler to look in the sketch directory first.     *
***********************************************************************/
//---------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h> 	// Malpartida 
#include "dht.h" 				// Rob Tillaert
#include "myRTClib.h"   			// Modified Adafruit  56 bytes NVRAM
#include <EEPROM.h>

/*************************************************************
*       Declare objects                                      *
*       RTC                                                  *
*       DHT                                                  *
*       LCD                                                  *
**************************************************************/
//RTC_DS1307 rtc;

RTC_DS1307 RTC; //declare object RTC
dht DHT;        //declare object DHT11
/**************************************************************
*           declare LCD data                                  *
*  set the LCD address to 0x27 for a 20 chars 2 line display  *
*  Set the pins on the I2C chip used for LCD connections:     *
*                     addr, en,rw,rs,d4,d5,d6,d7,bl,blpol     *
***************************************************************/
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
// Mega: D20=A4 D21=SCL

/**************************************************************
*            Declare pins )                                   *
***************************************************************/
//analogue pins
byte moisturePin = A0; 	// read soil moisture
byte levelPin = A1; 	// set level for  irrigation
byte NTCPin = A2; 		// voor NTC if used
byte LDRPin = A3; 		// analog pin for LDR
//digital pins
byte humidPin = 2; 		// Humidity sensor
byte SwitchButton = 3; 	// Make Switch
byte PushButton = 4; 	// PushButton
byte lightPin = 5; 		// Switches on light
byte fanPin = 6;  		// Switches fan
byte pumpPin = 7; 		// Switches pump
byte hotPin = 8;  		// Switches heater
byte buzPin = 9;  		// Switches buzzer
byte emptyPin = 10; 	// digital  Pin10  guards waterlevel
byte CO2Pin = 11; 		// Winsen MH-Z19 PWM pin 9  pin 7 =gnd
byte spikePin = 12; 	// Digital  Pin12  ->for intermittent switching of sensor spike
byte signalPin = 13; 	// used for signal functions LED


/**************************************************************
*             Variables                                       *
***************************************************************/
volatile byte seconds;
byte push = 0;             	// contains the state of the pushbutton
byte sButton = 0;          	// contains the state of the throw switch
byte light;                	// contains the value for LDR reading
byte pump = 0;             	// contains the pump state
unsigned int moist = 0;    	// contains the soil moisture level
unsigned int irrigate = 0; 	// contains the set level for irrigation in case no potmeter
byte level = 0;            	// contains level of the waterlevel (HIGH or LOW)
int c;                     	// contains the temperature reading
byte setLow = 19;			// lage temperastuurgrens 'boven 19' >=
byte setHigh = 22;			// hoge temperatuurgrens <=
int CO2ppmRead = 0;		   	// contains CO2 reading
boolean ready_to_read = 1; 	// is set every 4 minutes, it begins set in order to start with a measurement imemdiately
boolean hydro = 0;         	// no more moisture readings but just a 6 times a day irrigation
boolean hydroPump_flag = 0; // checks if pump was set according to a hydroponics scheme or because of measurement
boolean t_flag = 0;			// makes sure temp is written only once
boolean CO2Read_flag = 0;	// CO2 is read (1), not read(0)
boolean tomaat = 0;			// indicates tomato protocol
// de warmte tellers
unsigned long starttime = 0;
int length = 0;				// running time
int lengthtotaal = 0;		// total time
boolean lengthstat = false;
boolean countflag = 0; 		// zorgt ervoor dat er maar 1x gestart of gestopt wordt met tellen
byte warmcount = 0;			// telt aantal keren dat verwarming aangaat running number
byte warmcounttotal;		// total number of heat cycles
int mintemp = 100;				// slaat de laagst gemeten temperatuur op
boolean lastpush = 0;


byte m = 0;  	// contains the minutes, refreshed each loop
byte h = 0;  	// contains the hours, refreshed each loop
byte s = 0;  	// contains the seconds, refreshed each loop
byte mo = 0; 	// contains the month, refreshes each loop
byte j = 0;  	// contains the year, refreshed each loop
byte d = 0;	 	// contains the day (1-31)
byte dag = 0;	// contains day of week (0-6)
int doy = 0;		// contains day of year (1-366)
// Constants
// these constants won't change:
const int sensorMin = 40;      // LDR minimum, discovered through experiment
const int sensorMax = 1012;    // LDR maximum, discovered through experiment
/**
 * 					Memory use  0-23 uren
 					24 bLSB van lengthtotaal
 					25 bMSB van lengthtotaal
 					26=warmcounttotal  bevat het aantal keren dat de verwarming aanging in de afgelopen dag
 					27=warmcount telt het aantal keren dat de verwarming aanging running number
 					28=dst
 					29=LSB van running length
 					30=MSB van running length
 					25 bytes over
 */
const byte dst = 28;// moet dan maar 28 worden

byte wintertimeday = 25; 	// basevalue for day of wintertime start
byte summertimeday = 25; 	// basevalue for day of summertime start
/*                 Definitions  */
#define DHT11_PIN humidPin   	// in fact a bit double
#define DS1307_I2C_ADDRESS 0x68 // each I2C object has a unique bus address, the DS1307 is 0x68
byte incomingByte = 0;
byte meten = 243; 			//lcd char te printen bij  meten
int CO2ppmLimit = 340; 		// set  lower CO2 value
/**************************************************************
 *            Macro's/Defines                                *
 * ***********************************************************/
#define eersteRegel lcd.setCursor(0, 0)
#define tweedeRegel lcd.setCursor(0, 1)
#define pompAAN digitalWrite(pumpPin,HIGH)
#define pompUIT digitalWrite(pumpPin,LOW)
#define BAUD_RATE 115200

/*************************************************************
* ---------------- Define Special Characters-----------------*
*  Create a set of new characters                            *
*  0=fan                                                     *
*  1=degree sign                                             *
*  2=termometer                                              *
*  3=waterdruppel / voor humidity                            *
*  4=spikes       / voor soil moisture                       *
*  5=pump                                                    *
*  6=up arrow	/ reservoir vol                              *
*  7=down arrow / reservoir leeg                             *
//************************************************************/
const uint8_t charBitmap[][8] =
{
	{
		// fan  wellicht char(248)  gebruiken
		B00000,  // -----
		B10001,  // X---X
		B01010,  // -X-X-
		B00100,  // --X--
		B01010,  // -X-X-
		B10001,  // X---X
		B00000,  // -----
		B00000   // -----
	}
	,
	{
		// degree wellicht char(223)  gebruiken
		B00110,  // --XX-
		B01001,  // -X--X
		B01001,  // -X--X
		B00110,  // --XX-
		B00000,  // -----
		B00000,  // -----
		B00000,  // -----
		B00000   // -----
		//	0x6, 0x9, 0x9, 0x6, 0, 0, 0, 0
	}
	,
	{
		// termometer
		B00100,  // --X--
		B01010,  // -X-X-
		B01010,  // -X-X-
		B01110,  // -XXX-
		B01110,  // -XXX-
		B11111,  // XXXXX
		B11111,  // XXXXX
		B01110   // -XXX-
	}
	,
	{
		// druppel
		B00100,  // --X--
		B00100,  // --X--
		B01010,  // -X-X-
		B01010,  // -X-X-
		B10001,  // X---X
		B10001,  // X---X
		B10001,  // X---X
		B01110   // -XXX-
	}
	,
	{
		// moisture sensor
		0b00100,  // --X--
		0b01110,  // -XXX-
		0b01010,  // -X-X-
		0b01010,  // -X-X-
		0b01010,  // -X-X-
		0b01010,  // -X-X-
		0b01010,  // -X-X-
		0b00000   // -----
	}
	,
	{
		//pomp
		B01010,  // -X-X-
		B00100,  // --X--
		B00100,  // --X--
		B00100,  // --X--
		B11111,  // XXXXX
		B11011,  // XX-XX
		B11111   // XXXXX
	}
	,
	{
		// up arrow - reservoir full
		0x0, 0x4, 0xE, 0x15, 0x4, 0x4, 0x4, 0x0
		// -----
		// --X--
		// -X-X-
		// X-X-X
		// --X--
		// --X--
		// --X--
		// -----
	}
	,
	{
		// down arrow - reservoir empty
		0x4, 0x4, 0x4, 0x4, 0x15, 0xE, 0x4, 0x0
		// -----
		// --X--
		// --X--
		// --X--
		// X-X-X
		// -X-X-
		// --X--
		// -----

	}
};
//-------- end creation--------------

void setup()
{

	/************************************************************
	 *  The timer is being set up in order to have a 4 minute   *
	 *  event in the ISR(TIMER1_COMPA_vect) routine for reading *
	 *  of a sensor... should one wish that                     *
	 *  ofcourse it is also possible to do a check on the       *
	 *  minutes dividable by 4  ( min % 4==0)                   *
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
	Serial.begin(BAUD_RATE);
	/************************************************************
	*  Set up the RTC                                           *
	*************************************************************/

	/*********************************************************
	* Set up the in and output pins                          *
	**********************************************************/
	pinMode(levelPin, INPUT); 		// set level
	pinMode(humidPin, INPUT); 		// measures humidity
	pinMode(emptyPin, INPUT); 		// measures reservoir
	pinMode(SwitchButton, INPUT);   // make Switch
	pinMode(PushButton, INPUT);     // PushButton
	pinMode(spikePin, OUTPUT); 		// for alternative supply to spikes
	pinMode(pumpPin, OUTPUT); 		// Output for Relay
	pinMode(fanPin, OUTPUT);  		// Output for fan
	pinMode(hotPin, OUTPUT);  		// Output for heater
	pinMode(lightPin, OUTPUT);		// Output for light
	pinMode(buzPin, OUTPUT);  		// Output for buzzer
	digitalWrite(pumpPin, LOW);		// Pump off
	digitalWrite(spikePin, LOW);	// moisture sensor off
	digitalWrite(fanPin, LOW); 		// fan Off
	digitalWrite(hotPin, LOW);  	// heater off
	digitalWrite(lightPin, LOW); 	// light Off
	digitalWrite(buzPin, LOW);
	// buzzer off
	/* Now LCD */
	/*********************************************************
	 *               Set up LCD                              *
	 *   initialize the lcd for 16 chars 2 lines             *
	 *  load  the  character definitions                     *
	 *  print some messages and blink the backlight          *
	 *********************************************************/
	lcd.begin(20, 4);
	//upload defined characters to LCD
	int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));
	for ( int i = 0; i < charBitmapSize; i++ )
	{

		lcd.createChar ( i, (uint8_t *)charBitmap[i] );// zijn al geladen
	}
	//---------------end upload----------------
	lcd.backlight();
	// Print a message to the LCD.
	lcd.setCursor(0, 0);// topleft
	lcd.print("Greenhouse");
	Serial.println("Greenhouse");
	lcd.setCursor(0, 1);

	

	//************************************************************
	// Check buttons at startup                                  *
	//************************************************************


}
/*----------------------------(end setup )---------------------*/

/***************************************************************
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

	/****************************************************
	*                     DST                          *
	* **************************************************/

	
					// hbyte running warmtime
		



	/******************************************************
	 *              Lichtschema                           *
	 * ****************************************************/


	/*******************************************************/

	//-------------------------------------------end DST
	if (Serial.available() > 0)
	{
		incomingByte = Serial.read();
		switch (incomingByte)
		{
			case 63://?
		//		printLogTemp();
				break;
			case 108: //l
		//		logTemp(c);
				break;
			case 109: //m
				Serial.print(mintemp);
				break;
			case 84: //T
			
				break;
			case 116: //t
			
				break;
			case 87: //W klok1 uur achteruit wintertijd
			
				break;
			case 119: // w Klok 1 uur vooruit zomertijd
			
				break;
			case 120://x
			
				break;

			default:
				break;
		}
	}
	/********************************************************************************
	 *             Klok gelijkzet routine                                           *
	 *             1- vat leeg      digitalRead(emptyPin)=0; //pin10
	 *             2- grond vochtig >200 sample(moisturePin) A0
	 *             3- hydroponics mode (digitalRead(SwitchButton) == 0) //pin d3
	 *             4- pushbutton=+10 min digitalRead(PushButton);//d4
	 * *****************************************************************************/
	
	
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


	/* ------------------Actions -------------------*/
	/*********************************************************************************
	*             5.  DISPLAY DATA                                                   *
	* Pushbutton not pressed: show measurements                                      *
	* Pushbutton pressed: show time                                                  *
	**********************************************************************************/

		//	lcd.clear();
		lcd.setCursor(0, 0);  //set cursor left on first line (line 0)
		//lcd.print(char(2)); //prints thermometer
		lcd.print("Temperatuur ");
		lcd.print((float)DHT.temperature, 0);
		lcd.print (char(1));  // prints degree sign
		lcd.print("C");
		lcd.setCursor(0,1);
	//	lcd.print(char(3));// droplet
		lcd.print("Vochtigheid  ");
		lcd.print((float)DHT.humidity, 0);
		lcd.print("%");
		

	/**************************************************
	 * now we measure temperature and air humidity    *                               *
	 * and if necessary switch on heating or fan      *
	 * temp kept at minimally 20 degrees              *
	 * humidity is kept below 60%                     *
	 **************************************************/
	// ---------------5. Action on temperature ------

	c = (DHT.temperature);

	if (c <= setLow)

	{
		// switch on heating
		digitalWrite(hotPin, HIGH);
		}
	if (c >= setHigh)
	{
		digitalWrite(hotPin, LOW);
			}
	//--------------6. Action on Humidity -----------
	if (DHT.humidity >= 60)
	{
		// Switch on Fan
		digitalWrite(fanPin, HIGH);
		// print fan-sign only if pushbutton not pressed
		if(push == 1)
		{
			lcd.setCursor(10, 1);
			lcd.print(char(0)); // fan sign
		}
	}
	else
	{
		digitalWrite(fanPin, LOW);
		// print fan-sign only if pushbutton not pressed
		if(push == 1)
		{
			lcd.setCursor(10, 1);
			lcd.print(' ');
		}
	}


//}

	/**********************************************************************
	 *            If probe loose sound alarm and switch off heating       *
	 * ********************************************************************/

}
//-------  End of Main program-----------
//
//-------- Start of functions------------

/**********************************************************************
* This function adds a leading zero on Serial port                    *
***********************************************************************/
void printDigits(int digits)   //this adds a 0 before single digit numbers
{
	//if (digits >= 0 && digits < 10) {
	if (digits < 10)
	{
		//	lcd.print('0');
		Serial.print('0');
	}
	//lcd.print(digits);
	Serial.print(digits);
}
/**********************************************************************
* This function adds a leading zero on LCD                            *
***********************************************************************/
void printDigitLCD(int digits)   //this adds a 0 before single digit numbers
{
	//if (digits >= 0 && digits < 10) {
	if (digits < 10)
	{
		lcd.print('0');

	}
	lcd.print(digits);

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
* This is the timer interrupt function that sets a flag every 4 min   *
***********************************************************************/
ISR(TIMER1_COMPA_vect)
{
	seconds++;
	if(seconds == 240)
	{
		seconds = 0;
		// leest elke 240 seconden (4 minuten) een sensor
		// readSensor();
		// moist=sample(moisturePin);  //read soil humiditySensor
		// of kies ervoor een flag te zetten
		ready_to_read = 1;

	}
}
/**********************************************************************
* This function calculates dewpoint                                   *
***********************************************************************/
double dewPointFast(double celsius, double humidity)
{
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity / 100);
	double Td = (b * temp) / (a - temp);
	return Td;
}



//===========================================================================

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
//---------------------------------------
byte customChar[8] = {
	0b00100,  // --X--
	0b01110,  // -XXX-
	0b01010,  // -X-X-
	0b01010,  // -X-X-
	0b01010,  // -X-X-
	0b01010,  // -X-X-
	0b01010,  // -X-X-
	0b00000   // -----
};
//--------------------------------------------------------------

 */
int CO2ppm()
/*********************************
 *  Winsen MH-Z19                *
 *  ppm=2000*(Th-2)/(TH+TL-4)    *
 *  TH+TL=1000                   *
 * *******************************/
{
	int ppm = pulseIn(CO2Pin, HIGH);
	ppm = (2 * (ppm - 2));
	return ppm;
}



// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{

	return( (val / 10 * 16) + (val % 10) );
}
// Combine 2 bytes into an integer
int bytesToInteger(byte lsb, byte msb)
{
	return (msb << 8 | lsb);
}


