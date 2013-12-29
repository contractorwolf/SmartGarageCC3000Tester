

/*************************************************** 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
 /*
This example does a test of the TCP client capability:
  * Initialization
  * Optional: SSID scan
  * AP connection
  * DHCP printout
  * DNS lookup
  * Optional: Ping
  * Connect to website and print out webpage contents
  * Disconnect
SmartConfig is still beta and kind of works but is not fully vetted!
It might not work on all networks!
*/
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include <aJSON.h>

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10


// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIV2); // you can change this clock speed

#define WLAN_SSID       "contractor"           // cannot be longer than 32 characters!
#define WLAN_PASS       "strife56"

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
//#define WEBSITE      "www.google.com"
//#define WEBPAGE      "/+jameswolf"

//#define WEBSITE      "www.adafruit.com"
//#define WEBPAGE      "/testwifi/index.html"



#define WEBSITE      "www.smartgarageopener.com"
#define WEBPAGE      "/device_sensors/1.json"
//#define WEBPAGE      "/device_sensors/"

//#define WEBSITE      "www.cnn.com"
//#define WEBPAGE      "/WORLD/?hpt=sitenav"

//#define WEBSITE      "www.npr.org"
//#define WEBPAGE      "/rss/rss.php?id=1001"



//#define WEBSITE      "edition/services"
//#define WEBPAGE      "/services/rss/"

//#define WEBSITE      "spreadsheets.google.com"
//#define WEBPAGE      "/feeds/list/0Av2v4lMxiJ1AdE9laEZJdzhmMzdmcW90VWNfUTYtM2c/2/public/basic?alt=json-in-script"


//#define WEBSITE      "ajax.googleapis.com"
//#define WEBPAGE      "/ajax/services/search/news?v=1.0&q=barack%20obama"




//ajax.googleapis.com/ajax/services/search/news?v=1.0&q=barack%20obama
//www.cnn.com/WORLD/?hpt=sitenav
//spreadsheets.google.com/feeds/list/0Av2v4lMxiJ1AdE9laEZJdzhmMzdmcW90VWNfUTYtM2c/2/public/basic?alt=json-in-script
//www.npr.org/rss/rss.php?id=1001
//rss.cnn.com/rss/edition.rss
//www.npr.org/rss/rss.php?id=1001


/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically on startup)
*/
/**************************************************************************/

uint32_t ip;
Adafruit_CC3000_Client www;



//led indicator pins
int ledConnected = 9;
int ledGND = 8;
int ledPower = 7;


int loopCount = 0;

unsigned long  lastRead = millis();

void setup(void)
{
    // initialize the digital pin as an output.
  pinMode(ledConnected, OUTPUT);  
  pinMode(ledGND, OUTPUT);  
  pinMode(ledPower, OUTPUT); 
  
  //set all led pins to gnd
  digitalWrite(ledGND, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(ledConnected, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(ledPower, LOW);    // turn the LED off by making the voltage LOW
  
  
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000 v0.1!\n")); 

  Serial.print("Free RAM: "); 
  Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  


  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  digitalWrite(ledPower, HIGH);   // turn the LED on, connected

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); 
  Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  // Optional: Do a ping test on the website
  /*
  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");  
  replies = cc3000.ping(ip, 5);
  Serial.print(replies); Serial.println(F(" replies"));
  */  

  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */

   Serial.println(F("\r\nSETUP COMPLETE"));
   delay(3000);
}

void loop(void)
{
    loopCount++;
    Serial.print(F("\r\nLOOP: "));
    Serial.println(loopCount);
    Serial.print(F("\r\n"));
          
    Serial.print("Free RAM: "); 
    Serial.println(getFreeRam(), DEC);
   
    if(!www.connected()){
      www = cc3000.connectTCP(ip, 80); 
      //possilly wait here, flash red led
      Serial.println("NOT CONNECTED, RECONNECTING");
    }
    
    displayConnectionDetails();
  
    if (www.connected()) {
      
      //http request
      www.fastrprint(F("GET "));
      www.fastrprint(WEBPAGE);
      www.fastrprint(F(" HTTP/1.1\r\n"));
      www.fastrprint(F("Host: ")); 
      www.fastrprint(WEBSITE); 
      www.fastrprint(F("\r\n"));
  
      //other headers
      www.fastrprint(F("User-Agent: Arduino\r\n"));  
      www.fastrprint(F("Accept: *\r\n")); 
      www.fastrprint(F("Content-Type: application/json\r\n"));  
      www.fastrprint(F("Keep-Alive: 100\r\n"));  
      www.fastrprint(F("Connection: keep-alive\r\n"));  
      //end headers
      
      www.fastrprint(F("\r\n"));
    
      www.println();
    } else {
      Serial.println(F("Connection failed"));    
      return;
    }
    
    
    //wraps incoming data received
    Serial.println(F("\r\nBEGINRECEIVE-----------------------------------"));
    
    // Read data until either the connection is closed, or the idle timeout is reached. 
    lastRead = millis();
   

   
   
   
    //while connected and not timed out   
    String serverjson = "";

   
    char json[200];

    
    
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) 
    {
       bool listen = false;  

      digitalWrite(ledConnected, HIGH);   // turn the LED on, receiving data
      while (www.available()) {
        
        
        //read and display
        char c = www.read();
        if(c == '{'){
          listen = true;  
        }

        
        
        if(listen){
          serverjson += c;
          //Serial.print(c);
        }
        
        if(c == '}'){
          listen = false;  
        }
        
        //end read and display
        
        
        lastRead = millis();
      }
    }
    
    //show timeout values
    Serial.println(www.connected());
    Serial.println(millis() - lastRead);
    Serial.println(IDLE_TIMEOUT_MS);
    
    
    
    
    //not sure if close here is necessary, 
    //maybe keep open? need to lookup
    www.close();
    
    Serial.print(F("\r\nJSON RECEIVED: "));
    Serial.println(serverjson);
    

    
    
    
    Serial.println(F("\r\nENDRECEIVE-----------------------------------"));
    
    

    
    //JSON PARSEING
    //JSON: {"closed":1,"created_at":"2013-11-26T23:00:28Z","id":1,"serial":"865976g897g78fuy7y","updated_at":"2013-11-26T23:00:35Z"}
    //serverjson.toCharArray(json, 200);
    //aJsonObject *root = aJson.createObject();  
    //root = aJson.parse(json);
    //char* string = aJson.print(root);
    //Serial.print(F("\r\nReverse JSON: "));
    //Serial.println(string);
    
    
    
    //aJsonObject *serial = aJson.getObjectItem(root, "closed");
    //Serial.println(serial->valuestring);
    //aJson.delete(root);
    
    
    
    
    digitalWrite(ledConnected, LOW);   // turn the LED off, no longer receiving data
     
    //20 seconds between calls
    delay(2000);
    
    /* You need to make sure to clean up after yourself or the CC3000 can freak out */
    /* the next time your try to connect ... */
    //Serial.println(F("\n\nDisconnecting"));
    //cc3000.disconnect();
    //digitalWrite(ledPower, LOW);   // turn the LED off, no longer connected
  
}







//SETUP FUNCTIONS, maybe not needed, good examples of whats possible
//*************************************************************************
//*************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
  uint8_t valid, rssi, sec, index;
  char ssidname[33]; 

  index = cc3000.startSSIDscan();

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

