#include <Arduino.h>
#include <ArduinoOTA.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <OneButton.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include "Button.h"
#include "Sensor.h"
#include "MyTimers.h"
#include "Secrets.h"
//#include "AnimChar.h"
using namespace std;

void connectToWiFi();
void talkToClient();
void setupOTA();
void loopOTA();
void setupTelnet();
void loopTelnet();

void checkCirculationTimer();
void checkButtons();
void checkSensors();
void processTemps();
void checkKeepPumping();
void determinePumping();
void animateCharacters();
void operateHotTub();
void checkSafeties();
void allOff();
void checkBlowerTimeOut();
void printTargetTemp();
void printTubTemp();
void printUnitTemp();
void printHeating();
void printPumping();
void printBlowing();
void printCirc();
void printStandby();
void printTemplate(); 
void resetUpdateTimer();
void computerMusic();

const int tempSensorWhitePin = 34;    // <--digital read
const int tempSensorBlackPin = 35;    // <--digital read
const int fuseSensorPin = 39;         // <--digital read --yellow
const int flowSensorPin = 36;         // <--digital read --green connector in unit
const int onboardLEDPin = 2 ;               
const int oneWireTempSensorsPin = 32;                             // from #define ONE_WIRE_BUS 2  --  https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/
const int beepPin = 25;                                             //PWM out for beeper
const int heaterPin = 17;                                           //good
const int pumperPin = 16;                                           //good
const int blowerPin = 4;                                            //good
const int lightsPin = 15;                                           //good
const int tempDnButtonPin = 18;                                     //good
const int tempUpButtonPin = 19;                                     //good
const int blowerButtonPin = 33;                                     //HAD TO MOVE FROM 5 TO 33 DO TO INSTANT "HIGH", good now
const int standbyButtonPin = 23;                                    //good
const int lcdSDAPin = 21;                                           //good
const int lcdSCLPin = 22;                                           //good
const int debounceTime = 50;

Button tempUpButton(tempUpButtonPin, debounceTime, beepPin); 
Button tempDnButton(tempDnButtonPin, debounceTime, beepPin);
Button blowerButton(blowerButtonPin, debounceTime, beepPin);
Button standbyButton(standbyButtonPin, debounceTime, beepPin);
OneWire oneWireTempSensors(oneWireTempSensorsPin);            // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature tubTempSensor(&oneWireTempSensors);         // Pass our oneWire reference to Dallas Temperature sensor 
LiquidCrystal_I2C lcd(0x27,20,4);
WiFiServer server(80);

/* Animate Char definitions
//AnimChar bubbleChar(bubbleCharByte[8][8], 8);   
//
// CLASS IDENTIFIER UNDEFINED, WHY
//
vector<vector<byte>>  heaterCharAllFrames = {  {0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000},
                                              {0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000},
                                              {0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010},
                                              {0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101},
                                              {0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010},
                                              {0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000},
                                              {0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010, 0b00000, 0b01000},
                                              {0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100} };
vector<byte>          heaterChar = heaterCharAllFrames[0];
vector<vector<byte>>  pumperCharAllFrames = {  {0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000},
                                              {0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000},
                                              {0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010},
                                              {0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101},
                                              {0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010},
                                              {0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000},
                                              {0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010, 0b00000, 0b01000},
                                              {0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100} };
vector<byte>          pumperChar = pumperCharAllFrames[0]; 
vector<vector<byte>>  blowerCharAllFrames = {  {0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000},
                                              {0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000},
                                              {0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010},
                                              {0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101},
                                              {0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010},
                                              {0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000},
                                              {0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010, 0b00000, 0b01000},
                                              {0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100} };
vector<byte>          blowerChar = blowerCharAllFrames[0]; 
vector<vector<byte>>  lightsCharAllFrames = {  {0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000},
                                              {0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000},
                                              {0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010},
                                              {0b00010, 0b00000, 0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101},
                                              {0b00000, 0b01000, 0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010},
                                              {0b01000, 0b10100, 0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000},
                                              {0b10100, 0b01000, 0b00000,	0b00010, 0b00101, 0b00010, 0b00000, 0b01000},
                                              {0b01000, 0b00000, 0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100} };
vector<byte>          lightsChar = lightsCharAllFrames[0]; 
*/

bool fuseSensor;
bool flowSensor;
bool heating = false;
bool pumping = false;
bool pumpFromCirc = false;
bool pumpFromHeat = false;
bool pumpFromKeepPumping = false;
bool blowing = false;
bool lighting = false;
bool standby = false;
bool lastHeating = false;
bool lastPumping = false;
bool lastBlowing = false;
bool lastLighting = false;
bool lastStandby = false;
bool reachedTargetTemp = false;
bool connected = false;
#define HOSTNAME "HotTub"
#define SER SerialAndTelnet

float currentUnitTemp = 0.0;
float currentTubTemp = 0.0;
float targetTemp = 102.0;
const float lowerTempDeviation = 0.75;
const float upperTempDeviation = 0.5;
const float tempMin = 95.0;
const float tempMax = 105.0;
const int second = 1000;
const int minute = 60 * second;
const int hour = 60 * minute;

Timers circulationTimer;
Timers keepPumpingTimer;
Timers blowerTimeOutTimer;
Timers flowSensorBuffer;
Timers updateTimer;

int pumpOffTime = 7 * hour;
int pumpOnTime = 30 * minute;
int circMinutesLeft = 0; 
int circStartTime = 0;
const int keepPumpingTime = 10 * minute;
const int blowerTimeOutTime = 30 * minute;
const int flowSensorBufferTime = 10 * second;
const int updateTime = 2 * second;
unsigned long currentTime = millis();
unsigned long previousTime = 0; 
const long timeoutTime = 1000;

String header;
String output26State = "off";
String output27State = "off";

//Timers frameTimer;
//const int frameTime = 42;						// 24 fps -> 1000ms/24fps  == ~42 ms
//int frame = 0;
//const int totalFrames = 8;

/*--------------------SETUP--------------------*/

void setup() {
  tone(beepPin, 4000, 50);
  noTone(beepPin);
  delay(100);
  tone(beepPin, 4000, 50);
  noTone(beepPin);
  delay(100);
  tone(beepPin, 4000, 50);
  noTone(beepPin);
  delay(100);
  tone(beepPin, 4500, 250);
  noTone(beepPin);
  delay(500);

  Serial.begin(9600);
  Serial.print("hot tubbin' and debuggin'");
  //pinMode(tempSensorWhitePin, INPUT);
  //pinMode(tempSensorBlackPin, INPUT);
  //pinMode(fuseSensorPin, INPUT_PULLDOWN);
  //pinMode(flowSensorPin, INPUT_PULLDOWN);
  pinMode(oneWireTempSensorsPin, INPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(pumperPin, OUTPUT);
  pinMode(blowerPin, OUTPUT);
  pinMode(lightsPin, OUTPUT);
  pinMode(lcdSDAPin, OUTPUT);
  pinMode(lcdSCLPin, OUTPUT);
  pinMode(beepPin, OUTPUT);
  pinMode(onboardLEDPin, OUTPUT);
  digitalWrite(onboardLEDPin, LOW);

  lcd.init();                             // initialize the lcd 
  lcd.backlight();  
  lcd.clear();
  lcd.setCursor(5,0);                     // print splash
  lcd.print("Welcome to");
  lcd.setCursor(2,1);
  lcd.print("Roger's Hot Tub!");
  lcd.setCursor(1,3);
  lcd.print("I hope it works...");
  tempUpButton.init();
  tempDnButton.init();
  blowerButton.init();
  standbyButton.init();
  delay(second);

  connectToWiFi();
  server.begin();
  setupTelnet();
  setupOTA();
  
  tubTempSensor.begin();
  tubTempSensor.setResolution(9);       // int between 9 and 12:    9->94ms ... 12->750ms
  
  circulationTimer.start(pumpOffTime);    // initialize  timers
  updateTimer.start(second);
  keepPumpingTimer.start(second);
  delay(second);
  
  lcd.clear();
  printTemplate();
  printTargetTemp();
}

/*--------------------LOOP--------------------*/

void loop() { 
  loopTelnet();
  loopOTA();
  talkToClient();
  checkButtons();                             // sets targetTemp, blowing, and lighting, and standby
  if (!standby) {                             // not in standby     
    if (updateTimer.available())              // if time to update, then...
    {     
      checkCirculationTimer();                // sets pumpFromCirc T/F
      checkBlowerTimeOut();                   // sets blowing false if unattended for blowerTime
      checkSensors();                         // gets values from sensors
      processTemps();                         // looks at sensor values and set heating and pumpFromHeat T/F
      checkKeepPumping();                     // set pumpFromKeepPumping true if recently heating
      determinePumping();                     // set pumping from pumpFrom[...]
    }
  } 
  operateHotTub();                          // sets relay pins H/L
  //checkSafeties();                        // check fuses flow sensors etc...
  resetUpdateTimer();
  lastStandby = standby;
}

/*--------------------CORE FUNCTIONS--------------------*/

void checkButtons() {
  if (tempUpButton.check())  {                               //TEMP UP SHORT - plus one degree
    if (standby == true) { 
      standby = false;
      printTemplate();
      printHeating();
      printPumping();
      printBlowing();
      printCirc(); 
    }      
    if (targetTemp < tempMax)     { targetTemp = targetTemp + 1.0; }
    printTargetTemp();
  } 
  
  if (tempDnButton.check())  {                               //TEMP DOWN SHORT - minus one degree
    if (standby == true) { 
      standby = false;
      printTemplate(); 
      printHeating();
      printPumping();
      printBlowing();
      printCirc(); 
    }      
    if (targetTemp > tempMin)     { targetTemp = targetTemp - 1.0; }
    printTargetTemp();
  }

  if (blowerButton.check())  {                               //BLOW SHORT - toggle blowing
    if (standby == true) { 
      standby = false;
      printTemplate(); 
      printHeating();
      printPumping();
      printBlowing();
      printCirc(); 
    }                                     
    blowing = !blowing; 
    if (blowing)                  { blowerTimeOutTimer.start(blowerTimeOutTime); }
    printBlowing();
  }  
  
  if (standbyButton.check()) { 
    standby = !standby;     
    if (standby) 
    {
      allOff();
      printStandby();
      computerMusic();
    } else {
      printTemplate(); 
      printHeating();
      printPumping();
      printBlowing();
      printCirc();   
    }
  }
} 

void checkCirculationTimer() {
  if (circulationTimer.available()) {                 //Time For Pumping?
    circStartTime = millis();
    circulationTimer.stop();
    if (pumpFromCirc) {                               //is pumping, turn off
      pumpFromCirc = false;
      circulationTimer.start(pumpOffTime);            //start off timer
    } else {                                          //is not pumping, turn on               
      pumpFromCirc = true;
      circulationTimer.start(pumpOnTime);             //start on timer
    }
  }
  printCirc();
}

void checkBlowerTimeOut() {
  if (blowing==true) {
    if (blowerTimeOutTimer.available())
    {
      blowing = false;
      printBlowing();
    }
  }
}

void checkSensors() {
  tubTempSensor.requestTemperatures();
  currentTubTemp = tubTempSensor.getTempFByIndex(0);
  currentUnitTemp = tubTempSensor.getTempFByIndex(1);
  printTubTemp();
  printUnitTemp();
}

void processTemps() {
  // 1: anytime current above target + upperD -> [off, reached]
  if (currentTubTemp >= targetTemp + upperTempDeviation) { 
    reachedTargetTemp = true;
    heating = false;
    pumpFromHeat = false;
  }
  // 2: target previously reached but current dropped below target - lowerD -> [on, not reached]
  else if (reachedTargetTemp && currentTubTemp <= targetTemp - lowerTempDeviation) {
    reachedTargetTemp = false;
    heating = true;
    pumpFromHeat = true;
    keepPumpingTimer.stop();
    keepPumpingTimer.start(keepPumpingTime);
  }
  // 3: not yet reached and current below target + upperD -> [on, no change to reached]
  else if (!reachedTargetTemp && currentTubTemp < targetTemp + upperTempDeviation) {
    heating = true;
    pumpFromHeat = true;
    keepPumpingTimer.stop();
    keepPumpingTimer.start(keepPumpingTime);
  }
  //4: off, but current is not below target - lowerD -> [off, no change to reached]
  else {
    heating = false;
    pumpFromHeat = false;
  }
}

void checkKeepPumping() {
  if(keepPumpingTimer.available()) {                     // if keepPumpingTimer is still running
    pumpFromKeepPumping = false;                         // continue pumping (to not burn out still hot heater)
  }
  else {
    pumpFromKeepPumping = true;
  }
}

void determinePumping() {
  if (pumpFromCirc || pumpFromHeat || pumpFromKeepPumping) {
    pumping = true; 
  }
  else {
    pumping = false;
  }
}

void operateHotTub() {
  if (heating!=lastHeating) {
    if (heating) { 
      //digitalWrite(heaterPin, HIGH);      //DEBUG - DON'T HEAT IF DRY 
      printHeating();
    } else {
      digitalWrite(heaterPin, LOW); 
      printHeating();
    }
    lastHeating = heating;
  }
  if (pumping!=lastPumping) {
    if (pumping) { 
      //digitalWrite(pumperPin, HIGH);      //DEBUG - DON'T PUMP IF DRY 
      printPumping();
    } else {
      digitalWrite(pumperPin, LOW); 
      printPumping();
    }
    lastPumping = pumping;
  }
  if (blowing!=lastBlowing) {
    if (blowing) {
      digitalWrite(blowerPin, HIGH); 
      printBlowing();
    } else {
      digitalWrite(blowerPin, LOW); 
      printBlowing();
    }
    lastBlowing = blowing;
  }
  if (lighting!=lastLighting) {
    if (lighting) {
      digitalWrite(lightsPin, HIGH);
    } else {
      digitalWrite(lightsPin, LOW); 
    }
    lastLighting = lighting;
  }
}

void checkSafeties() {
  /*
  if (flowSensor.getValue() == 999999  && !flowSensorBuffer.available())
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("flow sensor override");
    lcd.setCursor(0,1);
    lcd.print("value = ");
    lcd.setCursor(8,1);
    lcd.print(flowSensor.getValue());
    allOff();
    while (true) { delay(50); }
  }
  if (fuseSensor.getValue() == 999999)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("fuse sensor override");
    lcd.setCursor(0,1);
    lcd.print("value = ");
    lcd.setCursor(8,1);
    lcd.print(fuseSensor.getValue());
    allOff();
    while (true) { delay(50); }
  }
  */
}

void resetUpdateTimer() {
  if (updateTimer.available())
  {
    updateTimer.stop();
    updateTimer.start(updateTime);
  }
}

/* animate characters
void animateCharacters() {
  if(frameTimer.available()) {
    if (frame < totalFrames)    { frame++;   }
    else                        { frame = 0; }
    heaterChar = heaterCharAllFrames[frame];
    pumperChar = pumperCharAllFrames[frame];
    blowerChar = blowerCharAllFrames[frame];
    lightsChar = lightsCharAllFrames[frame];
    frameTimer.stop();
    frameTimer.start(frameTime);
  }
}
*/

/*--------------------IOT--------------------*/

void connectToWiFi() {
  lcd.clear();
  lcd.setCursor(0,0);                     
  lcd.print("Connecting to:");
  delay(second / 2);

  lcd.setCursor(0,1);
  lcd.print(SECRET_SSID);
  delay(second / 2);

  WiFi.begin(SECRET_SSID, SECRET_PASS);
  int i = 0;
  Timers timeoutTimer;
  int timeoutTime = 30 * second;
  timeoutTimer.start(timeoutTime);
  while (WiFi.status() != WL_CONNECTED && !timeoutTimer.available()) {    //while not connected yet and not timed out
    delay(50);
    lcd.setCursor(i, 2);
    lcd.print(".");
    i++;
    if (i == 20) { 
      i = 0; 
      lcd.setCursor(0,2);
      lcd.print("                    ");
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
  lcd.setCursor(0, 2);
  lcd.print("WiFi connected!     ");
  lcd.setCursor(0, 3);
  lcd.print("RSSI:               "); 
  lcd.setCursor(6, 3); 
  lcd.println(WiFi.RSSI());
  delay(second);
  lcd.setCursor(0,3);
  lcd.print("IP:                 "); 
  lcd.setCursor(4, 3); 
  lcd.println(WiFi.localIP());
  delay(second);
  } else {
    lcd.setCursor(0, 2);
    lcd.print("WiFi not connected! ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    delay(second);
  }
}

void talkToClient() {
  WiFiClient client = server.available();   // Listen for incoming clients
  Serial.println("client:");
  Serial.println(client);
  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("new client:");
    Serial.println(client);
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            //mine--------------------------\/\/\/
            if (header.indexOf("GET /blower/on") >= 0) {
              Serial.println("blower on via webserver");
              blowing = true;
            }
            if (header.indexOf("GET /blower/off") >= 0) {
              Serial.println("blower off via webserver");
              blowing = false;
            }
            //mine--------------------------/\/\/\
            
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(2, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(2, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(99, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(99, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");


            //mine--------------------------\/\/\/
            if (blowing == true) {
              client.println("<p>Bubbles: On</p>"); 
            } else {
              client.println("<p>Bubbles: Off</p>"); 
            } 
            if (blowing == true) {
              client.println("<p><a href=\"/blowing/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/blowing/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 

            if (standby == true) {
              client.println("<p>Standby:</p>");
              client.println("<p>Hot Tub Off</p>"); 
            } else {
              client.println("<p>Hot Tub On</p>"); 
            } 
            if (standby == true) {
              client.println("<p><a href=\"/standby/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/standby/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 



            //mine--------------------------/\/\/\
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void setupOTA()
{
  ArduinoOTA.onStart([]() {
    Serial.println(F("Start"));
    tone(beepPin, 4100, 50);
    noTone(beepPin);
    delay(100);
    tone(beepPin, 4200, 50);
    noTone(beepPin);
    delay(100);
    tone(beepPin, 4300, 50);
    noTone(beepPin);
    delay(100);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Over The Air Update!");
    lcd.setCursor(0,1);
    lcd.print("Progess:");
    
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F(" \r\nEnd"));
    tone(beepPin, 4300, 50);
    noTone(beepPin);
    delay(100);
    tone(beepPin, 4500, 250);
    noTone(beepPin);
    delay(250);
    lcd.setCursor(0,3);
    lcd.print("Complete!");
    delay(500);
    lcd.clear();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    lcd.setCursor((progress / (total / 20)), 2);
    lcd.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
  });
  ArduinoOTA.begin();
}

void loopOTA() {
  ArduinoOTA.handle();
}

void setupTelnet() {
  
}

void loopTelnet() {
  
}

/*--------------------TASK HANDLING--------------------*/


/*--------------------UTILITIES--------------------*/

void allOff() {
  heating = false;
  pumping = false;
  blowing = false;
  lighting = false;
  digitalWrite(heaterPin, LOW);
  digitalWrite(pumperPin, LOW);
  digitalWrite(blowerPin, LOW);
  digitalWrite(lightsPin, LOW);
}

void printTargetTemp() {
    lcd.setCursor(5,0);
    if (targetTemp>=100) { lcd.print(targetTemp, 1); }
    else {
      lcd.print(" ");
      lcd.setCursor(6,0); 
      lcd.print(targetTemp, 1);
    }
}
void printTubTemp() {
  lcd.setCursor(5,1);
  if (currentTubTemp>=100) { lcd.print(currentTubTemp, 1); }
  else {
    lcd.print(" ");
    lcd.setCursor(6,1); 
    lcd.print(currentTubTemp, 1);
  }
}
void printUnitTemp() {
  lcd.setCursor(5,2);
  if (currentUnitTemp>=100) { lcd.print(currentUnitTemp, 1); }
  else {
    lcd.print(" ");
    lcd.setCursor(6,2); 
    lcd.print(currentUnitTemp, 1);
  }
}
void printHeating() {
  lcd.setCursor(16,0);
  if (heating) {
    lcd.print("Heat");
  } else {
    lcd.print("    ");
  }
}
void printPumping() {
  lcd.setCursor(16,1);
  if (pumping) {
    lcd.print("Pump");
  } else {
    lcd.print("    ");
  }
}
void printBlowing() {
  lcd.setCursor(13,2);
  if (blowing) {
    lcd.print("Bubbles");
  } else {
    lcd.print("       ");
  }
}
void printCirc() {
  lcd.setCursor(0,3);
  long time;
  int hr;
  int min;
  String displayTime;
  if (pumping) {
    if (pumpFromHeat || pumpFromKeepPumping) 
    {
      lcd.print("Circ While Heating  ");
    } 
    else if (!pumpFromHeat && pumpFromKeepPumping) 
    {
      lcd.print("Circ While Hot for   ");
      time = (circStartTime + keepPumpingTime) - millis();
      min = (time % hour) / minute;
      displayTime = (String(min) + "m");
      lcd.setCursor(19,3);
      lcd.print(displayTime);
    } 
    else if (!pumpFromHeat && !pumpFromKeepPumping && pumpFromCirc) 
    {
      lcd.print("Circulate For       ");
      time = (circStartTime + pumpOnTime) - millis();
      hr = time / hour;
      min = (time % hour) / minute;
      if (hr > 0)
      {
        displayTime = (String(hr) + "h:" + String(min) + "m");
      }
      else
      {
        displayTime = (String(min) + "m");
      }
      lcd.setCursor(14,3);
      lcd.print(displayTime);
    }
  } 
  else
  { 
    lcd.print("Pump Idle for       ");
    time = (circStartTime + pumpOffTime) - millis();
    hr = time / hour;
    min = (time % hour) / minute;
    displayTime = (String(hr) + "h:" + String(min) + "m");
    lcd.setCursor(14,3);
    lcd.print(displayTime);
  }
}
void printStandby() {
  if (standby) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("      Standby:      ");
    lcd.setCursor(0,1);
    lcd.print("    Hot Tub Off!    ");
    lcd.setCursor(0, 2); 
    if(WiFi.status() == WL_CONNECTED) {
      lcd.print("WiFi connected!     ");
      lcd.setCursor(0, 3);
      lcd.print("RSSI:               "); 
      lcd.setCursor(6, 3); 
      lcd.print(WiFi.RSSI());
      delay(second);
      lcd.setCursor(0, 3);
      lcd.print("IP:                 "); 
      lcd.setCursor(4, 3); 
      lcd.println(WiFi.localIP());
    } else if (WiFi.status() != WL_CONNECTED) {
      lcd.print("WiFi not connected! ");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    }
  }
}
void printTemplate() {
  lcd.clear();
  lcd.setCursor(0,0);                     
  lcd.print(" Set:               ");
  lcd.setCursor(0,1);
  lcd.print("Temp:               ");
  lcd.setCursor(0,2);
  lcd.print("Unit:               ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  printTargetTemp();
  printTubTemp();
  printUnitTemp();
  printCirc();
}

void computerMusic() {
  float noteMin = 100;
  float noteMax = 100;
  float durMin = 500;
  float durMax = 500;
  float note;
  float duration;
  while (durMax >= 10) {
    noteMin = noteMin * 1.01;
    noteMax = noteMax * 1.25;
    durMin = durMin * 0.7;
    durMax = durMax * 0.8;
    note = random(noteMin, noteMax);
    duration = random(durMin, durMax);
    
    tone(beepPin, note, duration);
    delay(duration);
    noTone(beepPin);
  }
  while (durMax <= 200) {
    noteMin = noteMin / 1.01;
    noteMax = noteMax / 1.25;
    durMin = durMin / 0.7;
    durMax = durMax / 0.8;
    note = random(noteMin, noteMax);
    duration = random(durMin, durMax);
    tone(beepPin, note, duration);
    delay(duration);
    noTone(beepPin);
  }
  while (durMax >= 10) {
    noteMin = noteMin * 1.01;
    noteMax = noteMax * 1.25;
    durMin = durMin * 0.7;
    durMax = durMax * 0.8;
    note = random(noteMin, noteMax);
    duration = random(durMin, durMax);
    tone(beepPin, note, duration);
    delay(duration);
    noTone(beepPin);
  }
  while (durMax <= 100) {
    noteMin = noteMin / 1.01;
    noteMax = noteMax / 1.25;
    durMin = durMin / 0.7;
    durMax = durMax / 0.8;
    note = random(noteMin, noteMax);
    duration = random(durMin, durMax);
    tone(beepPin, note, duration);
    delay(duration);
    noTone(beepPin);
  }
  int durMod = 0;
  while (noteMax < 5000) {
      while (durMax >= 10) {
      noteMin = noteMin * 1.01;
      noteMax = noteMax * 1.25;
      durMin = durMin * 0.7;
      durMax = durMax * 0.8;
      durMod = (durMin / 2);
      note = random(noteMin, noteMax);
      duration = random(durMin, durMax);
      tone(beepPin, note, duration);
      delay(duration - durMod);
      noTone(beepPin);
    }
    while (durMax <= 100) {
      noteMin = noteMin * 1.01;
      noteMax = noteMax * 1.25;
      durMin = durMin / 0.7;
      durMax = durMax / 0.8;
      durMod = (durMin / 2);
      note = random(noteMin, noteMax);
      duration = random(durMin, durMax);
      tone(beepPin, note, duration);
      delay(duration - durMod);
      noTone(beepPin);
    }
  }
}

/* lcd template
01234567890123456789
 Set:100.0      Heat
Temp:100.0      Pump
Unit:100.0   Bubbles
Pump while heating
*/
