/*
   Copyright (c) 2015, Majenko Technologies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

 * * Neither the name of Majenko Technologies nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include "DHT.h"
#include "Timer.h"
#include <IRsend.h>
#include <IRutils.h>

// DHT Sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
const int DHTPin = 2;
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);
static char celsiusTemp[7];
static char hcelsiusTemp[7];
static char fahrenheitTemp[7];
static char humidityTemp[7];
static float h = 0;
static float t = 0;
static float hic = 0;

Timer timer;


uint16_t RECV_PIN = 14;
uint16_t CAPTURE_BUFFER_SIZE = 1024;
IRrecv irrecv(RECV_PIN, CAPTURE_BUFFER_SIZE);
IRsend irsend(4);  // An IR LED is controlled by GPIO pin 4 (D2)
int khz = 38; // 38kHz carrier frequency for the NEC protocol

#define IR_STATUS_NOT_STARTED 0
#define IR_STATUS_SUCC 1
#define IR_STATUS_OVERFLOW 2
#define IR_STATUS_MANY 3
#define IR_STATUS_STARTED 4
#define IR_STATUS_TIMEOUT 5

unsigned int last_ir_job_status = IR_STATUS_NOT_STARTED;
unsigned int cur_ir_index = 0;
unsigned long last_ir_sent_time = 999999; //don't wait for first time

decode_results results;  // Somewhere to store the results
irparams_t save;         // A place to copy the interrupt state while decoding.

struct IRData {
  int size;
  uint16_t data[200];
};

#define IR_LOW_IDX 0
#define IR_HIGH_IDX 1
#define IR_POWER_ON_IDX 2
#define IR_POWER_OFF_IDX 3

#define IR_MODES 4

#define IR_HIGH_SET (1 << IR_HIGH_IDX)
#define IR_LOW_SET (1 << IR_LOW_IDX)
#define IR_POWER_ON_SET (1 << IR_POWER_ON_IDX)
#define IR_POWER_OFF_SET (1 << IR_POWER_OFF_IDX)
#define IR_READY (IR_HIGH_SET | IR_LOW_SET)

String array_codes[IR_MODES] = { "low", "high", "power on", "power off" };

#define STARTED (1<<IR_MODES)

struct Settings {
  int magic;
  int status;
  unsigned int ir_delay; // threshold before sending IR again
  unsigned int min_ir_size; // threshold before saving IR 
  float low_temp;
  float high_temp;
};

int MAGIC = 0xDEA0;
Settings settings;

void reset_settings() {
    settings.magic = MAGIC;
    settings.status = 0;
    settings.low_temp = 23;
    settings.high_temp = 25;
    settings.ir_delay = 60;
    settings.min_ir_size = 20;
    EEPROM.put(0, settings);
    EEPROM.commit();
}


bool set_tempreture(float* temp, char* temp_str) {
  strcpy(temp_str,"Failed");
  *temp = NAN; // set NaN
  int i = 10;
  while (isnan(*temp) && i > 0) {
    i--;
    *temp = dht.readTemperature();
    //Serial.print("\nset temp: ");
    //Serial.print(*temp);
    if (isnan(*temp)) {
      delay(2000);
    }
    else {
      dtostrf(*temp, 6, 2, temp_str);
    }
    
  }
  return !isnan(*temp);
}

bool set_humidity(float* humidity, char* hum_str) {
  strcpy(hum_str, "Failed");         
  *humidity = NAN; // set NaN
  int i = 10;
  while (isnan(*humidity) && i > 0) {
    i--;
    *humidity = dht.readHumidity();
    //Serial.print("\nset humidity: ");
    //Serial.print(*humidity);
    if (isnan(*humidity)) {
      delay(2000);
    }
    else {
      dtostrf(*humidity, 6, 2, hum_str);
    }
  }
  return !isnan(*humidity);

}

void dotemp() {
  strcpy(humidityTemp, "Failed");         
  if (set_tempreture(&t, celsiusTemp) && set_humidity(&h, humidityTemp)) {
     hic = dht.computeHeatIndex(t, h, false);       
     dtostrf(hic, 6, 2, hcelsiusTemp);                 
  }
  if (settings.status & STARTED) {
    if(isnan(hic)) {
      Serial.println(F("invalid temp reading"));
      return;
    }
    if(abs(millis() - last_ir_sent_time) < (settings.ir_delay * 1000)) {
      Serial.println(F("ir threshold wait:"));
      return;
    }
    if (hic > settings.high_temp) {
      Serial.println(F("high temp detected"));
      send_ir(IR_LOW_IDX); // send low temp IR
    }
    else if (hic < settings.low_temp) {
      Serial.println(F("high temp detected"));
      send_ir(IR_HIGH_IDX); // send high temp IR
    }
  }
}


const char *ssid = "esp8266";
const char *password = "";
String HtmlHeader = "<html><head><title>ESP8266</title><style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head><body>";

ESP8266WebServer server ( 80 );

const int led = 13;

void handleRoot() {
  char temp[800];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf ( temp, 800,

             "<html>\
  <head>\
    <title>ESP8266 smart IR DHT</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Auto IR DHT ESP8266!</h1><br>\
    %s\
    <br><p>Uptime: %02d:%02d:%02d</p>\
  </body>\
</html>",
             printHelp().c_str(), hr, min % 60, sec % 60
           );
  server.send ( 200, "text/html", temp );
}

void handleWifi() {
  if (server.hasArg("ssid")) {
    String ssid = server.arg("ssid");
    String pass = "";
    if (server.hasArg("pass")) {
      pass = server.arg("pass");
    }
    String message = "New Wifi ssid :";
    message += ssid;
    message += ", password: ";
    message += pass;
    server.send ( 200, "text/plain", message );
    WiFi.begin(ssid.c_str(), pass.c_str());
    return;
  }

  String message = "Wifi settings\n\n";
  message += "AP IP address ";
  message += WiFi.softAPIP().toString();
  message += "\nWifi status: ";
  message += WiFi.status();
  message += "\nIP:";
  message += WiFi.localIP().toString();
  int n = WiFi.scanNetworks();
  if (n == 0)
    message += "no networks found\n";
  else
  {
    message += "\n";
    message += n;
    message += " networks found\n";
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      message += (i + 1);
      message += (": ");
      message += (WiFi.SSID(i));
      message += (" (");
      message += (WiFi.RSSI(i));
      message += (")");
      message += ((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      message += "\n";
    }
  }
  server.send ( 200, "text/plain", message );

}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
    result += ':';
  }
  return result;
}

void setup ( void ) {
  Serial.begin(9600);
  irrecv.enableIRIn();  // Start the receiver
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ready");

  Serial.println("esp8266ing started");
  
  EEPROM.begin(4096);
  EEPROM.get(0, settings);
  if(settings.magic != MAGIC) {
   Serial.println("first boot - no settings");
   reset_settings();
  }

  WiFi.mode(WIFI_AP_STA);
  int ret;

  uint8_t timeout = 20; // 20 * 500 ms = 10 sec time out
  while ( ((ret = WiFi.status()) != WL_CONNECTED) && timeout-- ) {
    Serial.print(".");
    delay(500);
  }

  // connected ? disable AP, client mode only
  if ((ret = WiFi.status()) == WL_CONNECTED) {
    Serial.println("connected!");
    Serial.print("STA IP address ");
    Serial.println(WiFi.localIP().toString());
    // not connected ? start AP
  } else {
    Serial.println("Configuring access point...");
    uint8_t  MAC_softAP[6];
    uint8_t* mac  = WiFi.softAPmacAddress(MAC_softAP);
    String APName = "ESP-IR-" + macToStr(mac);
    WiFi.softAP(APName.c_str(), "");
    WiFi.softAPIP();
    Serial.print("AP IP address ");
    Serial.println(WiFi.softAPIP().toString());
  }

  Serial.println("esp8266ing loop");
  if ( MDNS.begin ( "esp8266" ) ) {
    Serial.println ( "MDNS responder started" );
  }

  server.on ( "/wifi", handleWifi );
  server.on ( "/", handleRoot );
  server.on ( "/set", handleSet );
  server.on ( "/set_ir", handleSetIR );
  server.on ( "/send_ir", handleSendIR );  
  server.on ( "/ir_job_status", handleIRJob );
  server.on ( "/get", handleGet );
  server.on("/temp", [](){  
    String webString="<html><head><meta http-equiv='refresh' content='20'/></head><body>Temperature:<br>Humidity: "+String(h)+" %<br>temp: "+String(t)+" C<br>heat index: "+String(hic)+" C";
    server.send(200, "text/html", webString);
  });
  server.on("/reset", [](){  
    reset_settings();
    String webString = HtmlHeader + "settings reset";
    server.send(200, "text/html", webString);
  });
  server.on("/start", [](){  
    String webString = HtmlHeader;
      if (settings.status & (IR_READY)) {
        webString += F("started");
        settings.status |= STARTED;
      } else {
        webString += F("can't start, status:");
        webString += settings.status;
      }
    server.send(200, "text/html", webString);
  });
  server.on("/stop", [](){  
    String webString = HtmlHeader + "stopped!";
    settings.status &= ~(STARTED);
    server.send(200, "text/html", webString);
  });
  server.on("/save", [](){  
    String webString = HtmlHeader + "settings saved!";
    EEPROM.put(0, settings);
    EEPROM.commit();
    server.send(200, "text/html", webString);
  });
  server.onNotFound ( handleRoot );
  server.begin();
  Serial.println ( "HTTP server started" );
  int afterEvent = timer.every(10000, dotemp);
  dht.begin();
  dotemp();

}

void loop ( void ) {
  server.handleClient();
  timer.update();
  if (last_ir_job_status == IR_STATUS_STARTED) {
    Serial.println ( "handle set IR request" );
    set_ir(cur_ir_index);
  }
}

String printHelp() {
  String help = "<br><br><b>usage:</b><br><a href='/wifi'> /wifi</a>(?ssid=[ssid](&pass=[pass]))  --  get or set wifi config";
  help += "<br><br> /set?(ir_delay=[secs])(&high_temp=[temp])(&low_temp=[temp])(&ir_size=[min IR size])  -- set global settings";
  help += "<br><br><a href='/get'> /get</a> -- get global settings";
  help += "<br><br><a href='/set_ir'> /set_ir</a> -- set IR code";
  help += "<br><br><a href='/send_ir'> /send_ir</a> -- send IR code";
  help += "<br><br><a href='/temp'> /temp</a> -- get current temperature";
  help += "<br><br><a href='/start'> /start</a> -- start auto IR";
  help += "<br><br><a href='/stop'> /stop</a> -- start auto IR";
  help += "<br><br><a href='/save'> /save</a> -- save settings to EEPROM";
  
  return help;
}

void handleGet() {
  String message = HtmlHeader + "<br>";
  message += "ir_delay: ";
  message += settings.ir_delay ;
  message += "ir_min_size: ";
  message += settings.min_ir_size;
  message += "</br>high temp: ";
  message += settings.high_temp;
  message += "</br>low temp: ";  
  message += settings.low_temp;
  if (settings.status & STARTED) {
    message += "</br><p style='color:#00FF00'>IR blaster started</p>";  
  } else {
    message += "</br>IR blaster stopped";
  }
  for (int i=0;i<IR_MODES;i++){
    if (settings.status & 1<<i) {
      message += "</br>IR " + array_codes[i] +" set<br>";
      IRData data;
      EEPROM.get(sizeof(settings) + (sizeof(data) * i), data);
      message += dumpCode(data.data, data.size);  

    } else {
      message += "</br><p style='color:#FF0000'>IR " + array_codes[i] +" not set</p>";  
    }
  }
  message += "</body></html>";
  server.send ( 200, "text/html", message );

}

void handleIRJob() {
  String message = HtmlHeader + "Set IR<br>";
  switch(last_ir_job_status) {
    case IR_STATUS_NOT_STARTED:
      message += "no pending IR job";
      break;
    case IR_STATUS_SUCC:
      message += "last IR job succeded";
      last_ir_job_status = IR_STATUS_NOT_STARTED;
      break;
    case IR_STATUS_OVERFLOW:
      message += "last IR job transmission size too big, need to change code constant!!!";
      break;
    case IR_STATUS_MANY:
      message += "last IR job got too many IR transmissions, undefined behaviour";
      break;
    case IR_STATUS_TIMEOUT:
      message += "last IR job didn't received transmission";
      break;
    case IR_STATUS_STARTED:
      message += "last IR job still pending, send IR code";
      break;
    default:
      message += "unknown program error code: ";
      message += last_ir_job_status;
      break;
  }
  message += "</body></html>";
  server.send ( 200, "text/html", message );
  return;
  
}

void handleSetIR() {
  Serial.println ( "handleSetIR" );
  String message = HtmlHeader;
  if (!server.hasArg("idx")) {  
    message += "Set IR<br>";
    for (int i=0;i<IR_MODES;i++){
      message += "<a href='/set_ir?idx=";
      message += i;
      message += "'>set " + array_codes[i] + " ir</a><br>";
    }    
    message += "</body></html>";
    server.send ( 200, "text/html", message );
    return;
  }
  int idx = server.arg("idx").toInt();
  message += "waiting for IR transmission, timeout:<span id='mycounter'></span>.<br><a href='/ir_job_status'>status</a>";
  message += "<script>i = 20;function onTimer() { document.getElementById('mycounter').innerHTML = i;i--;if (i < 0) {window.location.href = '/ir_job_status';}else {setTimeout(onTimer, 1000);}}; onTimer();</script>";
  server.send ( 200, "text/html", message );
  last_ir_job_status = 0;
  cur_ir_index = idx;
  last_ir_job_status = IR_STATUS_STARTED;
  //set_ir(idx);
}

void handleSendIR() {
  String message = HtmlHeader;
  if (!server.hasArg("idx")) {  
    message += "Send IR<br>";
    for (int i=0;i<IR_MODES;i++){
      message += "<a href='/send_ir?idx=";
      message += i;
      message += "'>send " + array_codes[i] + " ir</a><br><br>";
    }    
    message += "</body></html>";
    server.send ( 200, "text/html", message );
    return;
  }
  int idx = server.arg("idx").toInt();
  message += HtmlHeader + "IR Sent</body></html>";
  server.send ( 200, "text/html", message );
  send_ir(idx);
}


void handleSet() {
  String message = HtmlHeader + "setting args:<br>";
  String unknown = "";
  bool set = false;
  bool wrong = false;
  for (uint8_t i = 0; i < server.args(); i++) {

    if (server.argName(i) == "ir_delay") {
        message += "<br>ir_delay old: ";
        message += settings.ir_delay;
        message += " new: ";
        settings.ir_delay = server.arg(i).toInt();
        message += settings.ir_delay;
        set = true;
    } else if (server.argName(i) == "ir_size") {
        message += "<br>ir_size old: ";
        message += settings.min_ir_size;
        message += " new: ";
        settings.min_ir_size = server.arg(i).toInt();
        message += settings.min_ir_size;
        set = true;
    } else if (server.argName(i) == "high_temp") {
        message += "<br>high_temp old: ";
        message += settings.high_temp;
        message += " new: ";
        settings.high_temp = server.arg(i).toFloat();
        message += settings.high_temp;
        set = true;
    } else if (server.argName(i) == "low_temp") {
        message += "<br>low_temp old: ";
        message += settings.low_temp;
        message += " new: ";
        settings.low_temp = server.arg(i).toFloat();
        message += settings.low_temp;
        set = true;
    } else {
        wrong = true;
        unknown += "<br><p style='color:#FF0000'> unknown arg: " + server.argName(i) + ": " + server.arg(i) + "</p>";
    }
  }
  message += unknown;
  if(wrong || (!set) ) {
    message += printHelp();
  }
  message += "</body></html>";
  server.send ( 200, "text/html", message );
}

String dumpCode(volatile uint16_t* data, int size)
{
  // Start declaration
  String res = "unsigned int  ";          // variable type
  res += "rawData[";                // array name
  res += size - 1, DEC;  // array size
  res += "] = {";                   // Start declaration

  // Dump data
  for (int i = 1;  i < size;  i++) {
    res += (data[i] * USECPERTICK);
    if ( i < size-1 ) res += ","; // ',' not needed on last one
    if (!(i & 1))  res += " ";
  }

  // End declaration
  res +=  "};"; 
  return res;
}

void send_ir(int idx) {
      IRData data;
      EEPROM.get(sizeof(settings) + (sizeof(data) * idx), data);
      irsend.sendRaw(data.data, data.size, khz);  
      last_ir_sent_time = millis();
}

int set_ir(int idx) {
  decode_results  results;
  unsigned long start_time = millis();
  Serial.println(F("waiting for IR signal"));  
  do {
    if (irrecv.decode(&results)) {  // Grab an IR code
      if(results.overflow) {
        Serial.println(F("IR overflow"));
        last_ir_job_status = IR_STATUS_OVERFLOW;
        irrecv.resume();
        return IR_STATUS_OVERFLOW;
      }
      Serial.println(dumpCode(results.rawbuf, results.rawlen));
      if(results.rawlen < settings.min_ir_size) {
        Serial.println(F("IR too small"));
        irrecv.resume();
        delay(0);
        continue;
      }
      IRData data;
      data.size = results.rawlen;
      memcpy(data.data, (char*)results.rawbuf, (results.rawlen * sizeof(uint16_t)));
      Serial.println((sizeof(settings) + (sizeof(data) * idx)));
      EEPROM.put(sizeof(settings) + (sizeof(data) * idx), data); // store IR in EEPROM
      delay(0);
      settings.status |= 1 << idx; // set ir status stored
      EEPROM.put(0, settings);
      EEPROM.commit();
      delay(0);
      irrecv.resume();
      Serial.println(F("IR signal stored"));  
      int more = 0;
      while (irrecv.decode(&results)) {  // if more IR coming
        Serial.println(dumpCode(results.rawbuf, results.rawlen));
        more = 1;
        delay(0);
        irrecv.resume();
      }
      if (more) {
        Serial.println(F("more IR received, undefined behaviour"));
        last_ir_job_status = IR_STATUS_MANY;
        return IR_STATUS_MANY;
      }
      last_ir_job_status = IR_STATUS_SUCC;
      Serial.println(F("return success"));  
      return 0;
    }
    delay(0);
  } while(millis() < (start_time + 20000));
  Serial.println(F("IR signal timeout"));  
  last_ir_job_status = IR_STATUS_TIMEOUT;
  return IR_STATUS_TIMEOUT;
}

