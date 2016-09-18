#include <Wire.h>
#include <radio.h>
#include <RDA5807M.h>
#include "RDSParser.h"
#include "U8glib.h"
#include <EEPROM.h>

#define DIGIT_FONT    u8g_font_helvR18n
//#define RUS_FONT      u8g_font_cronx1h
#define RUS_FONT      u8g_font_orgv01
#define FIX_BAND      RADIO_BAND_FM

// Nokia 5110 display connection
const byte lcd_CE       = 10;
const byte lcd_RST      = 8;
const byte lcd_DC       = 9;
const byte lcd_DIN_MOSI = 11;
const byte lcd_CLK      = 13;

const byte BACKLIGHT_PIN = 5;                   // PWM control for the screen backlight
const byte BATTERY_PIN   = A0;                  // Used to measure the battery voltage
const byte LIGHT_SENSOR  = A1;                  // Photo sensor to measure ambient light

// Rotary encoder
const byte R_MAIN_PIN = 2;                      // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 6;                      // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 3;                      // Rotary Encoder push button pin

const uint8_t battery_bitmap[] PROGMEM = {
  0b01111111, 0b11111000,
  0b10000000, 0b00000100,
  0b10001000, 0b01000111,
  0b10001000, 0b01000101,
  0b10001000, 0b01000111,
  0b10000000, 0b00000100,
  0b01111111, 0b11111000,
};

const uint8_t Lhighlight_bitmap[] PROGMEM = {
  0b00011000,
  0b00011100,
  0b00011110,
  0b00011111,
  0b00011110,
  0b00011100,
  0b00011000
};

const uint8_t Rhighlight_bitmap[] PROGMEM = {
  0b00011000,
  0b00111000,
  0b01111000,
  0b11111000,
  0b01111000,
  0b00111000,
  0b00011000
};

//------------------------------------------ backlight of the LCD display (lite version) ----------------------
#define H_LENGTH 8                                  // Sensor history length
class BL {
  public:
    BL(byte sensorPIN, byte lightPIN) {
      sensor_pin = sensorPIN;
      led_pin    = lightPIN;
    }
    void init(void);                                // Initialize the data
    void adjust(void);                              // Automatically adjust the brightness
  
  private:
    byte sensor_pin;                                // Light sensor pin
    byte led_pin;                                   // Led PWM pin
    uint32_t checkMS;                               // Time in ms when the sensor was checked
    byte brightness;                                // The backlight brightness
    byte new_brightness;                            // The baclight brightness to set up
    int queue[H_LENGTH];                            // Statistics of the sensor values
    byte h_indx;                                    // Current history index
    void put(int item);                             // Save the sensor value to the history
    int average(void);                              // Calculate average value for internal history
    const byte default_brightness = 128;            // Default brightness of backlight
    const byte dayly_brightness = 150;              // Dayly brightness of backlight
    const uint16_t b_night = 400;                   // light sensor value of the night
    const uint16_t b_day = 700;                     // light sensor value of the day light
    const uint16_t period = 200;                    // The period in ms to check the photeregister
    const byte nightly_brightness = 50;             // The display brightness when it is dark
    const byte max_dispersion = 15;                 // The maximum dispersion of the sensor to change the brightness  
};

void BL::init(void) {

  pinMode(led_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  int light = analogRead(sensor_pin);
  for (byte i = 0; i < H_LENGTH; ++i) queue[i] = light;
  brightness = new_brightness = default_brightness;
  checkMS = 0;
  adjust();
  h_indx = 0;
}

void BL::put(int item) {
  queue[h_indx++] = item;
  if (h_indx >= H_LENGTH) h_indx = 0;
}

int BL::average(void) {
  long sum = 0;
  for (byte i = 0; i < H_LENGTH; ++i) sum += queue[i];
  sum += H_LENGTH >> 1;                             // round the average
  sum /= H_LENGTH;
  return (int)sum;
}
  
void BL::adjust(void) {

  if (new_brightness != brightness) {
    if (new_brightness > brightness) ++brightness; else --brightness;
    analogWrite(led_pin, brightness);
    delay(5);
  }

  if (millis() < checkMS) return;
  checkMS = millis() + period;
 
  int light = analogRead(sensor_pin);
  put(light);
  light = average();
  if (light < b_night) {
    new_brightness = nightly_brightness;
    return;
  }

  if (light > b_day) {
    new_brightness = 0;
    return;
  }

  new_brightness = map(light, b_night, b_day, nightly_brightness, dayly_brightness);
}

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    volatile byte mode;                         // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 2000) { pt = 0; buttonPIN = ButtonPIN; overPress = timeout_ms; }
    bool buttonPressLong(void) {
      unsigned long now_t = millis();
      return ((pt > 0) && (now_t - pt > shortPress) && (now_t - pt < overPress));
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void cnangeINTR(void);
    bool buttonTick(void);
  private:
    const unsigned int shortPress = 900;
    unsigned int overPress;                     // Maxumum time in ms the button can be pressed
    volatile unsigned long pt;                  // Time in ms when the button was pressed (press time)
    byte buttonPIN;                             // The pin number connected to the button
};

void BUTTON::cnangeINTR(void) {                 // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                 // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1;  // short press
        else mode = 2;                          // long press
      pt = 0;
    }
  }
}

bool BUTTON::buttonTick(void) {                // Check the button state, called each time in the main loop

  bool keyUp = digitalRead(buttonPIN);         // Read the current state of the button
  unsigned long now_t = millis();
  if (!keyUp) {                                // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt > 0) {
      pt = 0;
      return true;                              // Button pressed once
    }
  }
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false) {
      min_pos = low; max_pos = upp;
      if (!write(initPos)) initPos = min_pos;
      increment = fast_increment = inc;
      if (fast_inc > increment) fast_increment = fast_inc;
      is_looped = looped;
    }
    void set_increment(byte inc) { increment = inc; }
    byte get_increment(void) { return increment; }
    bool write(int16_t initPos) {
      if ((initPos >= min_pos) && (initPos <= max_pos)) {
        pos = initPos;
        return true;
      }
      return false;
    }
    int16_t read(void) { return pos; }
    void cnangeINTR(void);
  private:
    const uint16_t overPress = 1000;
    int32_t min_pos, max_pos;
    volatile uint32_t pt;                           // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                      // Time in ms when the value was changed
    volatile bool channelB;
    volatile int32_t pos;                           // Encoder current position
    byte mPIN, sPIN;                                // The pin numbers connected to the main channel and to the socondary channel
    bool is_looped;                                 // Weither the encoder is looped
    byte increment;                                 // The value to add or substract for each encoder tick
    byte fast_increment;                            // The value to change encoder when in runs quickly
    const uint16_t fast_timeout = 300;              // Time in ms to change encodeq quickly
};

void ENCODER::cnangeINTR(void) {                    // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                       // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ class RDS_RADIO, combines the radio and RDS parcer --------------
class RDS_RADIO : public RDA5807M, public RDSParser {
  public:
    RDS_RADIO() {}
    void  seekUp(bool Auto)   { RDA5807M::seekUp(Auto);   resetRDS(); }
    void  seekDown(bool Auto) { RDA5807M::seekDown(Auto); resetRDS(); }
    char* rdsData(void)       { return rds_name; }
    void  rdsServiceCB(char *name);
    void  setFrequency(uint16_t freq);
    void  radioInit(void);

  private:
    void resetRDS(void);                            // Reset RDS data
    char *rds_name;                                 // The pointer to the rds data
    void utf8rus(char* str);                        // Convert Russian utf8 code to the win encoding
};

void RDS_RADIO::utf8rus(char* str)  {
  byte i, k, n;

    byte d = 0;
    k = strlen(str); i = 0;
   
    while (i < k) {
      n = str[i]; i++;
   
      if (n >= 0xC0) {
        switch (n) {
          case 0xD0: {
            n = str[i]; i++;
            if (n == 0x81) { n = 0xA8; break; }
            if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
            break;
          }
          case 0xD1: {
            n = str[i]; i++;
            if (n == 0x91) { n = 0xB8; break; }
            if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
            break;
          }
        }
      }
      str[d++] = n;
    }
    str[d] = '\0';
}

void RDS_RADIO::rdsServiceCB(char *name) {
  utf8rus(name);
  rds_name = name;
}

void RDS_RADIO::resetRDS(void) {
  rds_name = 0;
  RDA5807M::clearRDS();
  RDSParser::init();
}
  
void RDS_RADIO::setFrequency(uint16_t freq) {
  RDA5807M::setFrequency(freq);
  resetRDS();
}

void RDS_RADIO::radioInit(void) {
  RDA5807M::init();
  RDSParser::init();
}

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM is 16 bytes long and it has the following format:
  uint32_t ID                           each time increment by 1
  uint16_t stations[5]                  list of the last 5 stations 
  byte CRC                              the checksum
*/
class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      rAddr = wAddr = 0;
      eLength = 0;
      nextRecID = 0;
    }
    void init();
    bool load(void);
    bool isValid(void)            { return is_valid; }
    uint16_t getStation(byte index);                // Return saved station frequence by the index
    bool saveStation(uint16_t st);                  // write updated config into the EEPROM
    
  private:
    uint16_t stations[4];                           // The list of last 4 stations. The list must fit to the screen
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                                 // The flag indicates that data can be saved
    bool is_valid;                                  // Weither tha data was loaded
    uint16_t rAddr;                                 // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                 // Address in the EEPROM to start write new record
    uint16_t eLength;                               // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                             // next record ID
    const byte record_size = 16;                    // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {          // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

uint16_t CONFIG::getStation(byte index) {
  uint16_t res;
  if (index < 4) res = stations[index];
  return res;
}

bool CONFIG::saveStation(uint16_t  st) {
  byte i = 0;
  for ( ; i < 4; ++i) {
    if (stations[i] == st) break;                   // This station is already in the list
  }
  if (i >= 4) i = 3;                                // This station is new one
  for (char j = i-1; j >= 0; --j)                   // shift station list one item down
    stations[j+1] = stations[j];
  stations[0] = st;                                 // Put new entry to the top

  return save();                                    // Save new data into the EEPROM
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)stations;
  for (byte i = 0; i < 4*sizeof(uint16_t); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                            // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

bool CONFIG::load(void) {
  is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[record_size];

  for (byte i = 0; i < record_size; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < 4*sizeof(uint16_t) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                              // To avoid empty fields
  if (summ == Buff[record_size-1]) {                    // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[i];
    }
    recID = ts;
    byte i = 4;
    memcpy(stations, &Buff[4], 4*sizeof(uint16_t));
    return true;
  }
  return false;
}

//------------------------------------------ class MAIN SCREEN -----------------------------------------------
class MSCR {
  public:
    MSCR(U8GLIB_PCD8544* pU8g, RDS_RADIO* Radio, ENCODER* Encoder, byte batteryPIN, CONFIG* Cfg) {
      pD       = pU8g;
      pRadio   = Radio;
      pEnc     = Encoder;
      batt_pin = batteryPIN;
	    pCfg     = Cfg;
    }
    void init(void);                   			    // Initialize the internal data
    void main(void);                            // The main screen loop
    void push(void);                            // Rotary button has been pressed shortly
    void menu(void);                            // Rotary button has been pressed for long time
  private:
    void setFrequency(void);                    // set new frequency to the radio
    void selectHistory(void);                   // Select the station from the history list
    U8GLIB_PCD8544* pD;                         // The pointer to the screen instance
    RDS_RADIO*      pRadio;                     // Pointer to the Radio instance
    ENCODER*        pEnc;                       // Pointer to the Encoder instance
	  CONFIG*         pCfg;						            // Pointer to the configuration instance
    uint16_t        freq;                       // Current frequency or the one to be set
    uint32_t        rds_update_time;            // The time in ms to update RDS data
    uint32_t        radio_save_time;            // The time in ms to save current station into EEPROM
    uint32_t        update_time;                // The time in ms to update the screen
    bool            fm_seek;                    // Weither to seek stations automatically or use fine tune
    bool            select_history;             // select from the history mode
    byte            batt_pin;                   // The battery sensor PIN
    RADIO_INFO      info;                       // Radio information structure
    const uint32_t  period = 2000;              // The period in ms to redraw the screen
    const uint16_t  low_station = 7600;         // Radio stations diappasone [76 MHz - 108 MHz]
    const uint16_t  upp_station = 10800;
    const byte      radio_step = 10;            // Increment the frequence by 0.1 MHz
    const byte      fast_step  = 1;
    const uint16_t batt_low  = 830;             // Sensor readings for low battery
    const uint16_t batt_high = 920;             // Sensor readings for high battery
};

void MSCR::init(void) {
  pinMode(batt_pin, INPUT);
  update_time = 0;
  rds_update_time = radio_save_time = 0;        // radio_update_time set to zero means not save the station config in future
  fm_seek = true;                               // By default seek to the next station
  select_history = false;
  freq = pCfg->getStation(0);
  setFrequency();
  pEnc->reset(freq, low_station, upp_station, radio_step, fast_step, true);
}

void MSCR::setFrequency(void) {
  if (!freq) freq = low_station;
  uint16_t t = freq - 10;
  if (t < low_station) t = freq + 10;
  pRadio->setFrequency(t);
  delay(300);
  pRadio->setFrequency(freq);
  pRadio->getRadioInfo(&info);
}

void MSCR::main(void) {

  if (select_history) {                         // Pick up the station from the history
    selectHistory();
    return;
  }

  // Synchronize frequence from the Radio
  int16_t pos = pRadio->getFrequency();
  if (pos != freq) {
    freq = pos;
    pEnc->write(freq);
    update_time = 0;
  }

  // If the rotary encoder has been changed, select new radio station
  uint32_t nowMS = 0;
  pos = pEnc->read();
  if ((uint16_t)pos != freq) {
    if (fm_seek) {
      if (pos > freq)
        pRadio->seekUp(true);
      else
        pRadio->seekDown(true);
    } else {
      freq = (uint16_t)pos;
      pRadio->setFrequency(freq);
    }
    update_time = 0;
    pRadio->getRadioInfo(&info);

    nowMS = millis();
    rds_update_time = nowMS + 100;
    radio_save_time = nowMS + 60000;            // Set time to save current station to the history 
  }
  
  // Update RDS information
  nowMS = millis();
  if (info.rssi && (nowMS >= rds_update_time)) {      // if RDS is active
    pRadio->checkRDS();
    rds_update_time = nowMS + 20;
  }

  if (radio_save_time && (nowMS >= radio_save_time)) {
    pCfg->saveStation(freq);
    radio_save_time = 0;                        // The station information is already saved in EEPROM
  }
 
  if (nowMS > update_time) {                    // Update the screen
    char radio_freq[6];
    pRadio->getRadioInfo(&info);  
    sprintf(radio_freq, "%3d.%1d", freq / 100, (freq % 100) / 10);
    int level = info.rssi + 4;
    if (level > 32) level = 32;
    level = map(level, 0, 32, 0, 16);

    int b = analogRead(batt_pin);
    if (b > batt_high) b = batt_high;
    if (b < batt_low)  b = batt_low;
    byte battery = map(b, batt_low, batt_high, 0, 13);
  
    pD->firstPage();
    do {
      pD->setFont(DIGIT_FONT);
      byte width = pD->getStrPixelWidth(radio_freq);
      byte lpos = 42 - width/2;
      pD->drawStr(lpos, 34, radio_freq);

      pD->drawTriangle(0, 7, level, 7, level, 7 - (level / 2));
      pD->drawBitmapP(84-16, 0, 2, 7, battery_bitmap);
      pD->drawBox(84-16+1, 1, battery, 5);

      // Display RDS info
      pD->setFont(RUS_FONT);
      char *rds = pRadio->rdsData();
      if (rds) {
        int len = pD->getStrPixelWidth(rds);
        pD->drawStr(42-len/2, 48, pRadio->rdsData());
      }
      if (fm_seek) pD->drawStr(30, 8, F("auto"));
    } while(pD->nextPage());
    update_time = millis() + period;
  }
}

void MSCR::push(void) {                         // short button press
  if (select_history) {                         // the station has been selected from the history
    freq = pCfg->getStation(freq);              // In the function call freq holds the station index in the list
	  setFrequency();
    pEnc->reset(freq, low_station, upp_station, radio_step, fast_step, true);
    select_history = false;
  } else {
    fm_seek = !fm_seek;
  }
  update_time = 0;
}

void MSCR::menu(void) {                         // long button press
  if (!select_history) {                        // switch to the main mode
    freq = 0;
    pEnc->reset(freq, 0, 3, 1, 1, true);
	  select_history = true;
    radio_save_time = millis() + 60000;         // Save new stations configuration in 1 minute
    update_time = 0;                            // Force to refresh the screen
  }
}

void MSCR::selectHistory(void) {                // Show the history list and the selection marks
  
  uint16_t pos = pEnc->read();
  if ((uint16_t)pos != freq) {                  // In the history mode freq holds station index in the list
    freq = pos;
	  update_time = 0;                            // Force to refresh the screen
  }

  if (millis() < update_time) return;

  char radio_freq[6];

  pD->firstPage();
  do {
    pD->setFont(RUS_FONT);
    for (byte i = 0; i < 4; ++i) {              // Only four lines of text can fit the display
      uint16_t s = pCfg->getStation(i);
      sprintf(radio_freq, "%3d.%1d", s / 100, (s % 100) / 10);
      byte width = pD->getStrPixelWidth(radio_freq);
      pD->drawStr(42 - width/2, i*12+9, radio_freq);
    }
	// Draw the marks
    pD->drawBitmapP(16,    freq*12, 1, 7, Lhighlight_bitmap);
    pD->drawBitmapP(84-24, freq*12, 1, 7, Rhighlight_bitmap);
  } while(pD->nextPage());
  
  update_time = millis() + period;
}

// ===================================== End of the classes definition ==============================================

U8GLIB_PCD8544 u8g(lcd_CLK, lcd_DIN_MOSI, lcd_CE, lcd_DC, lcd_RST);       
RDS_RADIO radio;
ENCODER rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON  rotButton(R_BUTN_PIN);
BL bckLight(LIGHT_SENSOR, BACKLIGHT_PIN);
CONFIG cfg;

MSCR mainScreen(&u8g, &radio, &rotEncoder, BATTERY_PIN, &cfg);

void rdsHandler(uint16_t b1, uint16_t b2, uint16_t b3, uint16_t b4) {
  radio.processData(b1, b2, b3, b4);
}

void rdsServiceNameCB(char *name) {
  radio.rdsServiceCB(name);
}

void setup() {
  u8g.setColorIndex(1); // pixel on

  bckLight.init();

  radio.radioInit();                            // Initialize the Radio
  radio.setBand(RADIO_BAND_FMWORLD);
  radio.setVolume(5);
  radio.setMono(false);
  radio.setBassBoost(false);
  radio.setMute(false);
  radio.setSoftMute(true);
  radio.attachReceiveRDS(rdsHandler);

  radio.attachServicenNameCallback(rdsServiceNameCB);

  rotEncoder.init();
  rotButton.init();
  
  cfg.init();
  cfg.load();
  mainScreen.init();

  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,   CHANGE);
}

void rotEncChange(void) {
  rotEncoder.cnangeINTR();
}

void rotPushChange(void) {
  rotButton.cnangeINTR();
}

// ======================== The Main Loop ====================================
void loop() {

  if (rotButton.mode) {
    if (rotButton.mode == 1) {                  // Short Button press
      mainScreen.push();
    } else {                                    // Long Button press
      mainScreen.menu();
    }
    rotButton.mode = 0;
  }
  mainScreen.main();
  bckLight.adjust();                            // Automatically adjust backlight of the screen
}

