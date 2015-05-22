

/* Chip McClelland - Cellular Data Logger
BSD license, Please keep my name and the required Adafruit text in any redistribution

Requirements: 
  - Account on Ubidots.  http://www.ubidots.com
  - Adafruit FONA GPRS Card - Listed below
  - I used a 3.3V 8MHz Arduino Pro Mini from Sparkfun - https://www.sparkfun.com/products/11114
  - I also used the Sparkfun MMA8452 Accelerometer - https://www.sparkfun.com/products/12756
  - There are a couple switches, pots and LEDs on my Carrier board -  https://oshpark.com/shared_projects/ygCgpmMP
  - In V5, I added a DS1339 I2C Real Time Clock to make reporting more regular

 I made use of the Adafruit Fona library and parts of the example code
 /*************************************************** 
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA 
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963

  These displays use TTL Serial to communicate, 2 pins are required to 
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
// Includes and Defines for the Libraries
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <DSRTCLib.h>
#include "i2c.h"               // not the wire library, can't use pull-ups
#include <Wire.h>
#include <avr/sleep.h>        // For Sleep Code
#include <avr/power.h>
#define FONA_RX 4
#define FONA_TX 5
#define FONA_RST 6
#define FONA_KEY 7
#define FONA_PS 8
#define CARRIER 1  //  0 for T-Mobile and 1 for Vodafone
#define MMA8452_ADDRESS 0x1D  // Our Accelerometer I2C Address set with open jumper SA0


// Pin Constants
const int int2Pin = 3;         // This is the interrupt pin that registers taps
const int int1Pin = 2;         // This is ths RTC interrupt pin
const int ledPin = 9;          // led connected to digital pin 4
const int SensitivityPot = A0; // Potentiometer used to adjust sensitivity
const int DelayPot=A1;         //  Used to adjust the time between events

// Instantiate all our functions
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);  // instantiate the fona Software Serial
Adafruit_FONA fona = Adafruit_FONA(&fonaSS, FONA_RST);     // Model the library
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
DS1339 RTC = DS1339(int1Pin,0);          // Instantiate the RTC on Pin 2 and interrupt 0 

//Program Execution Variables
int tries = 0;                 // keep track of connection attepmts
volatile boolean TransmitFlag = 0;      // Makes sure we send data
int ledState = LOW;            // variable used to store the last LED status, to toggle the light
String Location = "";          // Will build the Location string here
uint16_t vbat;                 // Battery voltage in milliVolts
int KeyTime = 2000;            // Time needed to turn on the Fona
unsigned long TimeOut = 20000; // How long we will give an AT command to comeplete
unsigned long TransmitRetry = 60000;  // How long will we wait to retransmit
unsigned long LastSend = 0;   // Last Time we sent data in milliseconds
unsigned long LastBump=0;      // Need to make sure we don't count again until debounce
int PersonCount = 0;           // Count the number of PIR hits in reporting interval
uint8_t n=0;                   // Fona connected status variable
uint16_t returncode;           // Fona function return codes
int RetryMode = 1;             // Ensures we send an initial set of data immediately
int retries=0;                 // Number of times we need to retransmit data
int ConnectRetryLimit = 50;    // Number of times Arduino will try to connect in GetConnected

// Accelerometer
const byte SCALE = 2;          // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 3;       // output data rate (kHZ) - 0=800kHz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
int InputValue = 0;            // Raw sensitivity input
byte Sensitivity = 0x00;       // Hex variable for sensitivity
int DelayTime = 0;             // Modification of delay time (in tenths of a second added to the 1 second standard delay
int threshold = 100;           // threshold value to decide when the detected sound is a knock or not
int debounce=500;             // need to debounce the sensor - figure bikes are spaced at
static byte source;           // Accelerometer interrupt register
volatile boolean knock=0;      // Flag to set if we detect a knock

void wakeUpNow()              // here the accelerometer interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  // If int2 goes high, either p/l has changed or there's been a single/double tap
   source = readRegister(0x0C);  // Read the interrupt source reg.
   knock = 1;                    // Set the knock flag
}

void nap()
{
  // Here is where we wake up from sleep based on the RTC alarm
  //  Serial.print(F("."));
  //    RTC.clear_interrupt();
  // TransmitFlag = 1;
}

void setup() {
//  byte c;                        // Initialize Ports and Serial
  Serial.begin(19200);
  RTC.start(); // ensure RTC oscillator is running, if not already
  pinMode(int1Pin, INPUT);      // RTC Interrupt pins
  digitalWrite(int1Pin, HIGH);
  pinMode(ledPin, OUTPUT);       // "Bump" LED
  pinMode(FONA_KEY,OUTPUT);      // This is the pin used to turn on and off the Fona
  pinMode(FONA_PS,INPUT);        // Need to declare pins for the Teensy
  digitalWrite(FONA_PS,HIGH);    //  Let's turn on the pull-up
  pinMode(int2Pin, INPUT);       // This is the pin the accelerometer will use to wake the Arduino
  digitalWrite(int2Pin, HIGH);   // Internal pull-up

  // Initialize the FONA
  if(digitalRead(FONA_PS)) {
    TurnOffFona();             //  Start with the Fona off
  }
  TurnOnFona();              // Then turn it on - helps to make sure stame state every time
  if(!InitializeFona()) {    // Initialize the Fona
    Serial.println(F("Initialization Failed"));
    BlinkForever;
  }
  else Serial.println(F("Initialization Succeeded"));
  ReadRSSI();
  // Now get and format Location
  char replyGSM[64];          // this is a large buffer for the GSM location
  Serial.println(F("Connected to GPRS - getting location"));  
  tries = 0;                         // Reset the tries counter  
  if (!fona.getGSMLoc(&returncode, replyGSM, 250)) {
    Serial.println(F("Failed!"));
    BlinkForever;                    // If we can't get the location - turn off and ask for a reboot
  }
   if (returncode == 0) {            // Return code 0 means that the get location call worked
    Serial.println(replyGSM);
    String replystring = String(replyGSM);  // Here is what we get: -78.799919,35.898777,2014/12/13,17:49:28
    Location = "{\"lat\":" + replystring.substring(11,16) + ",\"lng\":" + replystring.substring(0,6) + "}";
    int commaCount = 0;
    String tempString = "";   
    for (int i=0; i<strlen(replyGSM); i++) {
      if (replyGSM[i] == ',') {
        commaCount++;
      }
      if (commaCount == 2) {
        tempString = replystring.substring(i+1,i+5);
        int year = tempString.toInt();
        tempString = replystring.substring(i+6,i+8);
        int month = tempString.toInt();
        tempString = replystring.substring(i+9,i+11);
        int day = tempString.toInt(); 
        tempString = replystring.substring(i+12,i+14);
        int hour = tempString.toInt(); 
        tempString = replystring.substring(i+15,i+17);
        int minute = tempString.toInt(); 
        tempString = replystring.substring(i+18,i+20);
        int second = tempString.toInt(); 
        set_time(year,month,day,hour,minute,second);  // Set's the time based on GSMLoc Results
        break;
      }
    }
  } 
  else {
    Serial.print(F("Fail code #")); Serial.println(returncode);
    BlinkForever();
  }
  
 // Not working but would like to come back to this someday
 // if(!RTC.time_is_set()) // If time set does not work then set error code
 // {
 //   Serial.println(F("Clock did not set! Check that its oscillator is working."));
 //   BlinkForever();
 // }
 
  
  // Now read the battery level for diagnostics
  if (! fona.getBattVoltage(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
  }
  if (!Send2ubidotsCount()) {                   // Sends Right Away to test connection
    Serial.println(F("Initial Send Failed - Reboot"));
    BlinkForever();
  }
  GetDisconnected();                           // Disconnect from GPRS
  TurnOffFona();                               // Turn off the FONA
  // Print out the free memory for diagnostics
  Serial.print(F("Free Ram Setup: "));
  Serial.println(freeRam());
   // Read the Pots for Sensitivity and Delay - Note this is done once at startup - need to press reset to take effect if changed
  InputValue = analogRead(SensitivityPot);
  InputValue = map(InputValue,0,1023,0,16);  // This value is used to set the Sensitivity (can go to 127 but not this application)
  Sensitivity = byte(InputValue);
  Serial.print(F("Input set to ="));
  Serial.println(Sensitivity);
  InputValue = analogRead(DelayPot);
  InputValue = map(InputValue,0,1023,0,2000);  // This value is used to set the delay in msec that will be added to the 1 second starting value
  debounce = debounce + InputValue;
  Serial.print(F("Debounce delay (in mSec) set to ="));
  Serial.println(debounce);
  // disable ADC since the only analog reading is the input sensitivity - saves power
  ADCSRA = 0; 
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readRegister(0x0D);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    initMMA8452(SCALE, dataRate);  // init the accelerometer if communication is OK
    Serial.println(F("MMA8452Q is online..."));
  }
  else
  {
    Serial.print(F("Could not connect to MMA8452Q: 0x"));
    Serial.println(c, HEX);
    BlinkForever();
  }
  // Attach the inerrupts for the Accelerometer and the Watch Dog Timer
  attachInterrupt(1, wakeUpNow, LOW);  // use interrupt 1 (pin 3) and run function wakeUpNow when pin 3 goes LOW
  set_hourly_alarm();                  // Sets an alarm to go off at the top of every hour
  attachInterrupt(0, nap, FALLING);
  RTC.enable_interrupt();
}


void loop() 
{
  if (!digitalRead(int1Pin))   { // This checks to see if it is time to send data or if we need to retry
    Serial.println(F("Alarm"));
    read_time();
    RTC.clear_interrupt();
    if (PersonCount > 0) {
      TransmitFlag = 1;
    }
  }
  if (TransmitFlag) {  // If we need to transmit
    if (LastSend + TransmitRetry <= millis()) {  // And we are not in a Retry Mode
      TurnOnFona();                    // Turn on the module
      CheckForBump();                  // This is where we look to see if we should count a bump
      if(GetConnected()) {             // Connect to network and start GPRS
        CheckForBump();                // This is where we look to see if we should count a bump
        if (Send2ubidotsCount()) {     // Send data to Ubidots
          TransmitFlag = 0;             // Sent count - we are happy
          CheckForBump();                // This is where we look to see if we should count a bump
          Send2ubidotsBatt();            // Send Battery Data
          CheckForBump();                // This is where we look to see if we should count a bump    
          Send2ubidotsRetries();         // Send Retry Data
        }
        else {
          LastSend = millis();        // reset the resend clock
        }
      }
      CheckForBump();                // This is where we look to see if we should count a bump    
      GetDisconnected();             // Disconnect from GPRS
      CheckForBump();                // This is where we look to see if we should count a bump
      TurnOffFona();                 // Turn off the module
    }
  }
  else  {
    if (LastBump + debounce <= millis())  { // Need to stay awake for debounce period - no sleep on retransmit
      Serial.print(F("Free Ram Bump: ")); // Print out the free memory for diagnostics
      Serial.println(freeRam());
      Serial.println(F("Entering Sleep mode"));
      delay(100);     // this delay is needed, the sleep function will provoke a Serial error otherwise!!
      sleepNow();     // sleep function called here
    }
  }
  CheckForBump();   // This is where we look to see if we should count a bump
}


void TurnOnFona()
{
  Serial.print(F("Turning on Fona: "));
  while(!digitalRead(FONA_PS)) 
  {
    digitalWrite(FONA_KEY,LOW);
    unsigned long KeyPress = millis();
    while(KeyPress + KeyTime >= millis()) {}
    digitalWrite(FONA_KEY,HIGH);
    delay(100);
  }
  fona.begin(4800);
  Serial.println(F("success!"));
}

boolean InitializeFona()
{
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));   // See if the FONA is responding
  if (! fona.begin(4800)) {                                    // make it slow so its easy to read!
    Serial.println(F("Couldn't find FONA"));
    return 0;                                                  // Initialization Failed
  }
  Serial.println(F("FONA is OK"));
  GetConnected();                                              // Connect to network and start GPRS
  while (!fona.enableGPRS(true))  {
    tries ++;
    Serial.print(F("Failed to turn on GPRS Attempt #"));
    Serial.println(tries);
    delay(500);
    if (tries >= 20) {
      Serial.println(F("GPRS failed to initalize"));
      return 0;                                                  // Initialization Failed
    }
  }
  return 1;                                                      // Initalization Succeeded
}

void ReadRSSI()
{
  // read the RSSI
  uint8_t n = fona.getRSSI();
  int8_t r;
  
  Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  Serial.print(r); Serial.println(F(" dBm"));
}

boolean GetConnected() 
{
  tries = 0;
  do 
  {
    tries ++; 
    n = fona.getNetworkStatus();  // Read the Network / Cellular Status
    Serial.print(F("Network status Try #")); 
    Serial.print(tries);
    Serial.print(F(" - "));
    Serial.print(n);
    Serial.print(F(": "));
      if (n == 0) Serial.println(F("Not registered"));
      if (n == 1) Serial.println(F("Registered (home)"));
      if (n == 2) Serial.println(F("Not registered (searching)"));
      if (n == 3) Serial.println(F("Denied"));
      if (n == 4) Serial.println(F("Unknown"));
      if (n == 5) Serial.println(F("Registered roaming"));
 
    delay(500); 
    if (tries >= ConnectRetryLimit) 
    {
      Serial.println(F("Failed to connect to the network"));
      TurnOffFona();
      return 0;
    }
  } while (n != 1 && n != 5);
  tries = 0;
  Serial.print(F("Start task and set APN: "));
  if (!SendATCommand("AT+CSTT=\"internetd.gdsp\"",'O','K')) {
    Serial.println(F("Failed to set APN"));
    return 0;
  }
  Serial.print(F("Bring up wireless connection: "));
  if (! SendATCommand("AT+CIICR",'O','K')) {
    if(! SendATCommand("",'O','K')) {
      if(! SendATCommand("",'O','K')) {
        SendATCommand("AT+CIPCLOSE",'O','K');
        SendATCommand("AT+CIPSHUT",'O','K');
        Serial.println(F("Failed to bring up the wireless connection"));
        retries++;
        return 0;
      }
    }
  }
  Serial.print(F("Get local IP address: "));
  SendATCommand("AT+CIFSR",'.','.');   // No good way to check this one as IP addresses can all be different
  return 1;
}

void GetDisconnected()
{
  fona.enableGPRS(false);
  Serial.println(F("GPRS Serivces Stopped"));
}

void TurnOffFona()
{
  Serial.print(F("Turning off Fona: "));
  while(digitalRead(FONA_PS)) 
  {
    digitalWrite(FONA_KEY,LOW);
    unsigned long KeyPress = millis();
    while(KeyPress + KeyTime >= millis()) {}
    digitalWrite(FONA_KEY,HIGH);
    delay(100);
  }
  Serial.println(F("success!"));
}

// This has been my problem child function - debugging items commented out
boolean SendATCommand(char Command[], char Value1, char Value2) 
{
  char replybuffer[64];          // this is a large buffer for replies
  int count = 0;
  int complete = 0;
  int location = 0;
  unsigned long commandClock = millis();                      // Start the timeout clock
  fona.println(Command);
  Serial.println(Command);
  while(!complete && millis() <= commandClock + TimeOut)         // Need to give the modem time to complete command 
  {
    while(!fona.available() &&  millis() <= commandClock + TimeOut);
    while(fona.available()) {                                 // reading data into char array 
      replybuffer[count]=fona.read();                       // writing data into array
      count++;
      if(count == 63) break;
    }
    Serial.print(F("count="));
    Serial.print(count);
    Serial.print(F(" - Reply: "));
    for (int i=0; i < count; i++) {
      if (replybuffer[i] != '\n') Serial.write(replybuffer[i]);
    }
    Serial.println("");                           // Uncomment if needed to debug
    for (int i=0; i < count; i++) {
      if(replybuffer[i]==Value1 && replybuffer[i+1]==Value2 || replybuffer[i]==Value1 && Value2 == Value1) {
        complete = 1;
        location = i;
        break;
      }
    }
    Serial.print(F("Time = "));
    int elapsed = int((millis()-commandClock)/1000);
    Serial.print(elapsed);
    Serial.print(F(" found: "));
    if (complete) {
      if (replybuffer[location] != '\n') Serial.write(replybuffer[location]);
      if (replybuffer[location+1] != '\n') Serial.write(replybuffer[location+1]);
     }
    else {
      Serial.print(F("nothing"));
    }
    Serial.println("");
    location = 0;
    count = 0;
    CheckForBump();   // This is where we look to see if we should count a bump
  }
  if (complete ==1) return 1;                            // Returns "True"  - "False" sticks in the loop for now
  else return 0;
}

boolean Send2ubidotsCount() 
{ 
  int num;
  String le;
  String var;
  int succeeded;
  // Send Person Count Information and GPS context
  var="{\"value\":"+ String(PersonCount) + ", \"context\":"+ Location + "}"; //value is the sensor value
  num=var.length();                                                          // How long is the payload
  le=String(num);                                                            //this is to calcule the length of var
  Serial.print(F("Start the connection to Ubidots: "));
  if (SendATCommand("AT+CIPSTART=\"tcp\",\"things.ubidots.com\",\"80\"",'C','T')) {
    Serial.println(F("Connected"));
  }
  Serial.print(F("Begin to send data to the remote server: "));
  if (SendATCommand("AT+CIPSEND",'\n','>')) {
    Serial.println(F("Sending"));
  }
  fona.println(F("POST /api/v1.6/variables/xxxxxxxxxxxxxx/values HTTP/1.1")); // Replace with your ID variable
  fona.println(F("Content-Type: application/json"));
  fona.print(F("Content-Length: "));
  fona.println(le);
  fona.println(F("X-Auth-Token: xxxxxxxxxxxxxx")); //in here, you should replace your Token
  fona.println(F("Host: things.ubidots.com"));
  fona.println();
  fona.println(var);
  fona.println();
  fona.println((char)26); //This terminates the JSON SEND with a carriage return
  Serial.print(F("Send JSON Package: "));
  if (SendATCommand("",'C','R')) { // The 201 code "CREATED" from Ubidots means it was successfully uploaded
    Serial.println(F("Sent Person Count"));
    PersonCount = 0;
    succeeded = 1;
  }
  else {
    Serial.println(F("Send Timed out, will retry at next interval"));
    retries++;
    succeeded = 0;
  }
  Serial.print(F("Close connection to Ubidots: ")); // Close the connection
  if (SendATCommand("AT+CIPCLOSE",'G','M')) {
    Serial.println(F("Closed"));
  }
  if (succeeded == 1) return 1;
  else return 0;
}
// Send Battery Information
void Send2ubidotsBatt() 
{
  int num;
  String le;
  String var;
  if (! fona.getBattVoltage(&vbat)) vbat= 4200;
  var="{\"value\":"+ String(vbat) + "}";              //value is the sensor value
  num=var.length();
  le=String(num);                                     //this is to calcule the length of var
    Serial.print(F("Start the connection to Ubidots: "));
  if (SendATCommand("AT+CIPSTART=\"tcp\",\"things.ubidots.com\",\"80\"",'C','T')) {
    Serial.println(F("Connected"));
  }
  Serial.print(F("Begin to send data to the remote server: "));
  if (SendATCommand("AT+CIPSEND",'\n','>')) {
    Serial.println(F("Sending"));
  }
  fona.println(F("POST /api/v1.6/variables/xxxxxxxxx/values HTTP/1.1")); // Replace with your ID variable
  fona.println(F("Content-Type: application/json"));
  fona.print(F("Content-Length: "));
  fona.println(le);
  fona.println(F("X-Auth-Token: xxxxxxxxxxx")); //in here, you should replace your Token
  fona.println(F("Host: things.ubidots.com"));
  fona.println();
  fona.println(var);
  fona.println();
  fona.println((char)26); //This terminates the JSON SEND with a carriage return
  Serial.print(F("Send JSON Package: "));
  if (SendATCommand("",'C','R')) { // The 201 code "CREATED" from Ubidots means it was successfully uploaded
     Serial.println(F("Sent Battery"));
  }
  else {
    Serial.println(F("Send Timed out, will retry at next interval"));
  }
 Serial.print(F("Close connection to Ubidots: ")); // Close the connection
 if (SendATCommand("AT+CIPCLOSE",'G','M')) {
    Serial.println(F("Closed"));
  }
}

void Send2ubidotsRetries() 
{
  int num;
  String le;
  String var;
  var="{\"value\":"+ String(retries) + "}";              //value is the sensor value
  num=var.length();
  le=String(num);                                     //this is to calcule the length of var
    Serial.print(F("Start the connection to Ubidots: "));
  if (SendATCommand("AT+CIPSTART=\"tcp\",\"things.ubidots.com\",\"80\"",'C','T')) {
    Serial.println(F("Connected"));
  }
  Serial.print(F("Begin to send data to the remote server: "));
  if (SendATCommand("AT+CIPSEND",'\n','>')) {
    Serial.println(F("Sending"));
  }
  fona.println(F("POST /api/v1.6/variables/xxxxxxxxxx/values HTTP/1.1")); // Replace with your ID variable
  fona.println(F("Content-Type: application/json"));
  fona.print(F("Content-Length: "));
  fona.println(le);
  fona.println(F("X-Auth-Token: xxxxxxxxx")); //in here, you should replace your Token
  fona.println(F("Host: things.ubidots.com"));
  fona.println();
  fona.println(var);
  fona.println();
  fona.println((char)26); //This terminates the JSON SEND with a carriage return
  Serial.print(F("Send JSON Package: "));
  if (SendATCommand("",'C','R')) { // The 201 code "CREATED" from Ubidots means it was successfully uploaded
     Serial.println(F("Sent Retries"));
  }
  else {
    Serial.println(F("Send Timed out, will retry at next interval"));
  }
 Serial.print(F("Close connection to Ubidots: ")); // Close the connection
 if (SendATCommand("AT+CIPCLOSE",'G','M')) {
    Serial.println(F("Closed"));
  }
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(0x0E, fsr >> 2);  
  else
    writeRegister(0x0E, 0);

  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));  
 
  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
  writeRegister(0x21, 0x55);  // 1. single taps only on all axes
  // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
  writeRegister(0x23, Sensitivity);  // 2. x thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x24, Sensitivity);  // 2. y thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x25, Sensitivity);  // 2. z thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x26, 0xFF);  // 3. Max time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeRegister(0x27, 0x64);  // 4. 1000ms (at 100Hz odr) between taps min, this also depends on the data rate
  writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max

  // Set up interrupt 1 and 2
  writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
  writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
  writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c & ~(0x01));
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c | 0x01);
}


// Read a single byte from address and return it as a byte
byte readRegister(uint8_t address)
{
  byte data;

  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
  i2cWaitForComplete();
  i2cReceiveByte(TRUE);
  i2cWaitForComplete();

  data = i2cGetReceivedByte();	// Get MSB result
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);	// Disable TWI
  sbi(TWCR, TWEN);	// Enable TWI

  return data;
}

// Writes a single byte (data) into address
void writeRegister(unsigned char address, unsigned char data)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendByte(data);
  i2cWaitForComplete();

  i2cSendStop();
}

void CheckForBump()
{
  if (digitalRead(int2Pin)==0)    // If int2 goes LOW (inverted), either p/l has changed or there's been a single/double tap
  {
    source = readRegister(0x0C);  // Read the interrupt source reg.
    if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
      if (millis() >= LastBump + debounce)
      {
        PersonCount++;                    // Increment the PersonCount
        LastBump = millis();              // Reset last bump timer
        Serial.print(F("Count: "));
        Serial.print(PersonCount);
        byte source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it - Finish with Accel before talking to clock
        ledState = !ledState;              // toggle the status of the ledPin:
        digitalWrite(ledPin, ledState);    // update the LED pin itself
        RTC.readTime();                    // update RTC library's buffers from chip
        Serial.print(F(" at "));
        printTime(0);                      // Adds time to the Bump event
      }
      else {
        byte source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it
      }
    }   
  }
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep mode: SLEEP_MODE_PWR_DOWN
     * 
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    /* Now it is time to enable an interrupt. We do it here so an 
     * accidentally pushed interrupt button doesn't interrupt 
     * our running program. if you want to be able to run 
     * interrupt code besides the sleep function, place it in 
     * setup() for example.
     * 
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
     * 
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    attachInterrupt(1,wakeUpNow, LOW);   // use interrupt 1 (pin 3) for the accelerometer - inverted
    attachInterrupt(0, nap, FALLING);    // Interrupt 0 (pin 2) for the Real Time Clock

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(1);      // disables interrupts so the 
    detachInterrupt(0);      // wakeUpNow code will not be executed 
                             // during normal running time.
}

void set_time(int year,int month, int day,int hour, int minute,int second)
// In this function we will set the clock based on the data returned form the GSMLoc function
// The data looks like this: "-78.799919,35.898777,2014/12/13,17:49:28"
// Since the Longitude could be shorter or longer, will use the commas for parsing the string
{
  Serial.println(F("Setting Time Based on GSMLoc Results"));
 
  // set initially to epoch
  RTC.setSeconds(second);
  RTC.setMinutes(minute);
//  RTC.setMinutes(58);   // For Testing
  RTC.setHours(hour);
  RTC.setDays(day);
  RTC.setMonths(month);
  RTC.setYears(year);
  RTC.writeTime();
  read_time();
}

void read_time() 
{
  Serial.print (F("The current time is "));
  RTC.readTime(); // update RTC library's buffers from chip
  printTime(0);
  Serial.println();
}

void set_hourly_alarm()
{
  // Test basic functions (time read and write)
  Serial.print ("The current time is ");
  RTC.readTime(); // update RTC library's buffers from chip
  printTime(0);
  Serial.println();
  Serial.println(F("Writing alarm to go off at the top of every hour"));
  Serial.print(F("Read back: "));
  RTC.setSeconds(0);
  RTC.setMinutes(0);
  RTC.setHours(0);
  RTC.setDays(0);
  RTC.setMonths(0);
  RTC.setYears(0);
  RTC.setAlarmRepeat(EVERY_HOUR); // There is no DS1339 setting for 'alarm once' - user must shut off the alarm after it goes off.
  RTC.writeAlarm();
  delay(500);
  RTC.readAlarm();
  printTime(1);  
 }
 
void set_minute_alarm()
{
  // Test basic functions (time read and write)
  Serial.print (F("The current time is "));
  RTC.readTime(); // update RTC library's buffers from chip
  printTime(0);
  Serial.println();
  Serial.println(F("Writing alarm to go off at the top of every minute"));
  RTC.setSeconds(0);
  RTC.setMinutes(0);
  RTC.setHours(0);
  RTC.setDays(0);
  RTC.setMonths(0);
  RTC.setYears(0);
  RTC.setAlarmRepeat(EVERY_MINUTE); // There is no DS1339 setting for 'alarm once' - user must shut off the alarm after it goes off.
  RTC.writeAlarm();
  delay(500);
  RTC.readAlarm();
  Serial.print(F("Read back: "));
  printTime(1);
  Serial.println();
}

void printTime(byte type)
{
  int mins; int secs;
  // Print a formatted string of the current date and time.
  // If 'type' is non-zero, print as an alarm value (seconds thru DOW/month only)
  // This function assumes the desired time values are already present in the RTC library buffer (e.g. readTime() has been called recently)

  if(!type)
  {
    Serial.print(int(RTC.getMonths()));
    Serial.print(F("/"));  
    Serial.print(int(RTC.getDays()));
    Serial.print(F("/"));  
    Serial.print(RTC.getYears());
  }
  else
  {
    //if(RTC.getDays() == 0) // Day-Of-Week repeating alarm will have DayOfWeek *instead* of date, so print that.
    {
      Serial.print(int(RTC.getDayOfWeek()));
      Serial.print(F("th day of week, "));
    }
    //else
    {
      Serial.print(int(RTC.getDays()));
      Serial.print(F("th day of month, "));      
    }
  }
  
  Serial.print(F("  "));
  Serial.print(int(RTC.getHours()));
  Serial.print(F(":"));
  mins = int(RTC.getMinutes());
  if (mins < 10) Serial.print(F("0")); 
  Serial.print(mins);
  Serial.print(F(":"));
  secs = int(RTC.getSeconds());  
  if (secs < 10) Serial.print(F("0"));
  Serial.println(secs);
}


// Debugging code, to check usage of RAM
// Example Call: Serial.println(freeRam());
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void BlinkForever()
{
  Serial.println(F("Error - Reboot"));
  TurnOffFona();
  while(1) {
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);
  }
}

