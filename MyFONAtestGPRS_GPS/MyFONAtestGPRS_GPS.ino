/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!

Open up the serial console on the Arduino at 115200 baud to interact with FONA

Note that if you need to set a GPRS APN, username, and password scroll down to
the commented section below at the end of the setup() function.
*/


/* this verstion will give you all supplumentary data such as voltage of battery strength of gprs signal etc.

Carlos Sandoval 12/5/2015
casa7199@colorado.edu 
7209377819
*/

#include "Adafruit_FONA.h"

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

void setup() {
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
 
  // Print module IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }


  // Optionally configure a GPRS APN, username, and password.

  fona.setGPRSNetworkSettings(F("fast.t-mobile.com"), F(""), F(""));

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

// enable GPRS

fona.enableGPRS(true);
 delay(5000);
 

}

  
void loop() {
  int dustDensity=45;
  int temp=54;
  int hum=75;
  int lon=123.07;
  int lat=23.32;
    
  
  
 


    /*********************************** GPS (SIM808 only) */
//
//        // turn GPS on
//        if (!fona.enableGPS(true))
//          Serial.println(F("Failed to turn on"));
//    
//      
//        int8_t stat;
//        // check GPS fix
//        stat = fona.GPSstatus();
//        if (stat < 0)
//          Serial.println(F("Failed to query"));
//        if (stat == 0) Serial.println(F("GPS off"));
//        if (stat == 1) Serial.println(F("No fix"));
//        if (stat == 2) Serial.println(F("2D fix"));
//        if (stat == 3) Serial.println(F("3D fix"));
//        
//      
//    
//    // takes 10 sec to get fix to actually be able to output correct GPS location
//    delay(15000); 
//
//
//        // check for GPS location
//        char gpsdata[120];
//        fona.getGPS(0, gpsdata, 120);
//        Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
//        Serial.println(gpsdata);
//      
//
// 

    /*********************************** GPRS */


        // turn GPRS on
        if (!fona.enableGPRS(true))
          Serial.println(F("Failed to turn on"));

// Data to post
//http://www.pymecontrol.cl/littledevices/solarbot.php?dust=23&temp=34&hum=52


      
        // Post data to website

        //********************************Change data type to String**********************
  
    char dustDensity_str[5];
    dtostrf(dustDensity,4,1,dustDensity_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char temp_str[4];
    dtostrf(temp,3,1,temp_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char hum_str[3];
    dtostrf(hum,3,1,hum_str);  //4 is mininum width, 3 is precision; float value is copied onto buff

    char long_str[3];
    dtostrf(lon,3,1,long_str);  //4 is mininum width, 3 is precision; float value is copied onto buff


    char lat_str[3];
    dtostrf(lat,3,1,lat_str);  //4 is mininum width, 3 is precision; float value is copied onto buff
    
    int ldata=strlen(dustDensity_str)+strlen(hum_str)+strlen(temp_str)+strlen(long_str)+strlen(lat_str); // Length of string of all combined data
    
    char data_str[60];

    sprintf(data_str,"dust=%s&temp=%s&hum=%s&lon=%s&lat=%s",dustDensity_str,temp_str,hum_str,long_str,lat_str);

    printf("Now sending %s...",data_str);
    Serial.println(data_str);
//*********************END**********************************************
        
        uint16_t statuscode;
        int16_t length;
       // char url[80];
        char data[80];

//**************dummy data variable*******************
   //int value=23;
   
  char da[]="carloslengthofdatasldkfjsldkfjsldkfjds";
  
  
//itoa function converts integer into null-terminated string. 
//It can convert negative numbers too. The standard definition of itoa function is give below:-
 
//char* itoa(int num, char* buffer, int base) 

//  
//  itoa(value,data2,10);
//  
//  
//  
//  int ldata = strlen(data2);
//
//  char data[ldata];
//  sprintf(data,"%s",data2);

//*************data end************************


//******************URL string*************************
char url1[]="www.pymecontrol.cl/littledevices/solarbot.php?";

int lurlfull = strlen(url1)+strlen(data_str); 
char url[lurlfull];

sprintf(url,"%s%s",url1,data_str);
//testin printing
Serial.print(url);
        flushSerial();
        Serial.print(F("http://"));
        Serial.println(url);
        Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        
        
        Serial.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
        
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_POST_end();
      }
    /*****************************************/

  
  
  
  
  // flush input
  flushSerial();
  while (fona.available()) {
    Serial.write(fona.read());
  }

}// end void loop

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
