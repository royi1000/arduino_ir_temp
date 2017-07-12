
#include "DHT.h"
#include "Timer.h"
#include <IRremote.h>
#include <EEPROM.h>

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


unsigned long last_ir_sent_time = 0;
int recvPin = 6;
IRrecv irrecv(recvPin);
IRsend irsend;
int khz = 38; // 38kHz carrier frequency for the NEC protocol

// DHT Sensor
const int DHTPin = 7;
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);

Timer timer;

//unsigned int  C20LEVEL1[187] = {3050,2900, 950,950, 1000,950, 1050,1900, 1950,900, 950,950, 1000,950, 1000,950, 1050,900, 1000,950, 1050,900, 1000,1900, 1950,1900, 1900,950, 1050,900, 1000,950, 950,1000, 950,1000, 1000,950, 950,950, 1000,950, 1000,950, 1050,900, 1000,950, 950,1000, 950,950, 1000,950, 1050,900, 1000,950, 1000,1900, 1900,1000, 3100,2850, 1050,900, 950,950, 1000,1950, 1950,900, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 950,1950, 1900,1950, 1950,900, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 1000,950, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 1000,950, 950,950, 1000,1950, 1950,900, 3150,2800, 1050,900, 950,1000, 950,1950, 1950,900, 1000,950, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,1950, 1950,1900, 1950,900, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 1000,950, 950,1000, 950,950, 1000,950, 1000,1950, 1950,850, 4000};  // UNKNOWN E3832232
//unsigned int  C30LEVEL1[193] = {3100,2800, 950,1000, 950,1000, 1000,1950, 1900,950, 1000,950, 1000,950, 1000,950, 1000,950, 1000,950, 1000,1900, 1050,950, 1000,900, 950,1000, 1850,1000, 950,1000, 1000,900, 950,1000, 1000,950, 1000,950, 1000,950, 1000,950, 1000,950, 1000,950, 1000,950, 950,1000, 950,950, 1050,950, 1000,900, 1050,950, 1000,1900, 1950,950, 3000,2950, 950,1000, 1000,900, 950,2000, 1850,1000, 950,1000, 1000,900, 950,1000, 1000,950, 950,950, 1050,1900, 1000,950, 1000,950, 950,1000, 1950,900, 1050,950, 1000,900, 950,1000, 1000,950, 1000,950, 1000,950, 950,1000, 1000,900, 950,1000, 1000,950, 1000,950, 1000,950, 950,950, 1000,950, 950,1000, 1000,1950, 1900,950, 3000,2950, 950,1000, 950,1000, 950,1950, 1950,950, 950,950, 1000,950, 1050,900, 950,1000, 1000,950, 1000,1950, 950,950, 950,1000, 1000,950, 1950,950, 1000,950, 1000,950, 950,1000, 950,950, 950,1000, 1000,950, 950,950, 1000,950, 950,1000, 1000,900, 1000,950, 1000,950, 950,1000, 950,950, 950,1000, 1000,1900, 1950,900, 4050};  // UNKNOWN 7C6C147C
//unsigned int  POWER[187]    = {3100,3800, 1900,1000, 1000,1850, 1000,950, 1950,900, 1100,850, 1050,850, 1050,900, 1100,850, 1050,850, 1050,1850, 1150,800, 2000,850, 1000,950, 1100,800, 1100,850, 1000,950, 1100,800, 1150,850, 1050,850, 1050,900, 1100,850, 1050,850, 1050,900, 1150,800, 1050,850, 1150,850, 1050,850, 1050,900, 950,1950, 2050,850, 3150,3750, 2050,800, 1150,1750, 1000,950, 2100,800, 1000,900, 1150,800, 1000,950, 1100,800, 1050,900, 1150,1750, 1100,850, 2000,850, 1100,850, 1050,900, 950,1000, 1000,900, 1150,800, 1050,900, 1100,850, 1050,850, 1100,850, 1100,850, 1050,850, 1100,850, 1100,850, 1050,900, 1050,850, 1150,800, 1100,1800, 2050,800, 3150,3800, 1950,900, 1100,1850, 1050,900, 2000,800, 1150,800, 1100,850, 1100,850, 1050,900, 1000,900, 1150,1800, 1050,900, 1900,950, 1150,800, 1100,850, 950,1200, 1150,850, 1150,750, 1050,900, 1050,850, 1150,800, 1100,850, 1150,800, 950,950, 1100,850, 1100,850, 1050,900, 1100,850, 1050,850, 1150,1800, 2050,800, 4050};  // UNKNOWN 5B12B3C9

static char celsiusTemp[7];
static char hcelsiusTemp[7];
static char fahrenheitTemp[7];
static char humidityTemp[7];
static float h = 0;
static float t = 0;
static float hic = 0;

struct IRData {
  int size;
  unsigned int data[RAWBUF];
};

#define IR_HIGH_SET 1
#define IR_LOW_SET 2
#define IR_POWER_SET 4
#define IR_READY (IR_HIGH_SET | IR_LOW_SET | IR_POWER_SET)
#define STARTED 8

#define IR_LOW_IDX 0
#define IR_HIGH_IDX 1
#define IR_POWER_IDX 2

struct Settings {
  int magic;
  int status;
  int ir_delay; // threshold before sending IR again
  float low_temp;
  float high_temp;
};

int MAGIC = 0xDEA0;

Settings settings;

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
    if(millis() - last_ir_sent_time < (settings.ir_delay * 1000)) {
      Serial.println(F("ir threshold wait"));
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
void reset_settings() {
    settings.magic = MAGIC;
    settings.status = 0;
    settings.low_temp = 24;
    settings.high_temp = 26;
    settings.ir_delay = 60;
    EEPROM.put(0, settings);
}

void setup() {
  Serial.begin(115200);
  strcpy(celsiusTemp,"Failed");
  strcpy(hcelsiusTemp,"Failed");
  strcpy(humidityTemp, "Failed");         
  
  EEPROM.get(0, settings);
  if(settings.magic != MAGIC) {
   Serial.println("first boot - no settings");
   reset_settings();
  }

  int afterEvent = timer.every(10000, dotemp);
  
  irrecv.enableIRIn();  // Start the receiver

    // initialize the DHT sensor
  dht.begin();
  dotemp();
  //Initialize serial and wait for port to open:
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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
        return 2;
      }
      IRData data;
      data.size = results.rawlen;
      memcpy(data.data, (char*)results.rawbuf, (results.rawlen * sizeof(unsigned int)));
      EEPROM.put(sizeof(settings) + (sizeof(data) * idx), data);     
      Serial.println(F("IR signal stored"));  
      return 0;
    }

  } while(millis() < (start_time + 60000));
  Serial.println(F("IR signal timeout"));  
  return 1;
}

void input_handle() {
    String s = Serial.readStringUntil('\n');
    if (s.startsWith(F("start"))) {
      if (settings.status & (IR_READY)) {
        Serial.println(F("starting"));
        settings.status |= STARTED;
      } else {
        Serial.print(F("can't start, status:"));
        Serial.println(settings.status);
      }
    }
    else if (s.startsWith(F("stop"))) {
      Serial.println(F("stopped"));
      settings.status &= ~(STARTED);
    }
    else if (s.startsWith(F("set ir low"))) {
      set_ir(IR_LOW_IDX);
    }
    else if (s.startsWith(F("set ir high"))) {
      set_ir(IR_HIGH_IDX);
    }
    else if (s.startsWith(F("set ir power"))) {
      set_ir(IR_POWER_IDX);
    }
    else if (s.startsWith(F("set temp high"))) {
      s.replace(F("set temp high "), "");
      settings.high_temp = s.toFloat();
      Serial.print(F("high temperature saved: "));
      Serial.println(settings.high_temp);
    }
    else if (s.startsWith(F("set temp low"))) {
      s.replace(F("set temp low "), "");
      settings.low_temp = s.toFloat();
      Serial.print(F("low temperature saved: "));
      Serial.println(settings.low_temp);
    }
    else if (s.startsWith(F("save settings"))) {
      EEPROM.put(0, settings);     
      Serial.println(F("settings saved"));
    }
    else if (s.startsWith(F("get temp"))) {
        if (set_tempreture(&t, celsiusTemp) && set_humidity(&h, humidityTemp)) {
           hic = dht.computeHeatIndex(t, h, false);       
           dtostrf(hic, 6, 2, hcelsiusTemp);                 
        }
        Serial.print(F("temperature: \nhumidity: "));
        Serial.print(h);
        Serial.print(F("% heat index: "));
        Serial.print(hic);
        Serial.print(F(" C temp: "));
        Serial.print(t);
        Serial.println(" C");
    }
    else if (s.startsWith(F("send power"))) {
        Serial.println(F("sending power signal"));
        send_ir(IR_POWER_IDX);
    }
    else if (s.startsWith(F("send low"))) {
        Serial.println(F("sending low signal"));
        send_ir(IR_LOW_IDX); 
    }
    else if (s.startsWith(F("send high"))) {
        Serial.println(F("sending high signal"));
        send_ir(IR_HIGH_IDX);
    }
    else if (s.startsWith(F("reset settings"))) {
        Serial.println(F("reset settings to defaults"));
        reset_settings();
    }
    else {
      Serial.println(F("unknown command, allowed:\nstart\nstop\nset ir <low|high|power>\nset temp <low|high> [temp]\nsave settings\nget temp\nsend <power|low|high>"));
    }

}

void loop() {
  timer.update();
  if (Serial.available()) {
    input_handle();
  }
}
