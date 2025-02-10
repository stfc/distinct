/********************************************************************
 ****                                                            ****
 ****                 DISTINCT aquaculture sensors               ****
 ****                                                            ****
 **** https://www.linkedin.com/pulse/iot-big-data-productive-safe-sustainable-aquaculture-miying-yang/
 ****                                                            ****
 **** Multisensor code supporting five different aquaculture     ****
 **** sensors, sending data through serial (USB)                 ****
 ****                                                            ****
 **** Code primarily based on https://github.com/DFRobot/        ****
 **** License is LGPL in accordance with upstream code.          ****
 ****                                                            ****
 **** Refactoring & new features by jens.jensen@stfc.ac.uk       ****
 **** Version 0.2a                                               ****
 ****                                                            ****/
#define DISTINCT_VERSION (0x00010300UL)
/****                                                            ****
 **** Brief changelog at end                                     ****
 ********************************************************************/

/** Analog PIN assignment setup **/

/** PH and EC are for pH value and EC value.  If a sensor is absent, the
 * ADC input will float and generate (probably) nonsense data - but you
 * will get data... */

#define TURB_PIN A0
#define PH_PIN A1
#define EC_PIN A2
#define DO_PIN  A3
#define ORP_PIN A4
#define TDS_PIN A5
// Temperature input is digital (DS18S20 1wire digital thermometer)
#define TMP_PIN 2

// LED is digital and controls the Arduino's on-board LED
#define LED_PIN 13

// 10 days
#define ORP_CALIBRATION_INTERVAL_MILLI 864000000UL
#define DO_CALIBRATION_INTERVAL_MILLI 864000000UL

// one second between measurements/printing
#define MEASURE_INTERVAL_MILLI 1000UL;

// Max number of flashes to accommodate
#define LED_FLASH_MAX 10

// Ticks (milliseconds) between LED on/off
#define LED_FLASH_TICK 100


#include "DFRobot_PH.h"
#include "DFRobot_EC10.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <OneWire.h>


DFRobot_PH ph;
DFRobot_EC10 ec;


/* Vcc (5000 mV) for Arduino.  Some Sketches had different values for this */
#define VREF 5000.0
#define OFFSET 10        // "zero drift voltage" for ORP

#define SCOUNT  30           // number of sample points for averaging

// The address calData is stored in the EEPROM.  This should be out of range
// of the addresses used by the libraries.
#define CalDataAddress 0x18
// Magic value
#define CalDataMagic 0xD15712C7UL

// Impossible temperature -1000^C returned as error value
#define ERRORTEMP                   (-1000.0f)


/** Calibration mode : whether we have been asked to calibrate a sensor
 * - which will temporarily disable measurements */
enum cal_mode_t {
		 CAL_MODE_NONE = 0,
		 CAL_MODE_ORP,
		 CAL_MODE_DO,
		 CAL_MODE_EC,
		 CAL_MODE_PH,
		 CAL_MODE_TEST
};


/** State of cailbration
  * See also function nextCalState 
  */

enum cal_state_t {
		  //* Enter the state for calibration
		  CAL_STATE_INIT = 0,
		  //* Calibration has been advertised, waiting
		  CAL_STATE_WAIT,
		  //* Measuring
		  CAL_STATE_MEASURING,
		  //* Ready to save (or exit)
		  CAL_STATE_FINAL
};


/** Data structure (definition);
 *
 * Every value has a dual float/ulong implementation in order to sanity check
 * values read off the EEPROM (both are four bytes in length).
 */

typedef union { float f; unsigned long d; } floatlong;

/** Calibration data.
 * Note that EC and pH calibrations are not available directly to the sketch.
 * The libraries have independent EEPROM write routines:
 * 0x00-0x03 pH 7
 * 0x04-0x07 pH 4
 * 0x0F-0x12 EC
 */
struct cal_data_t {
  unsigned long magic;
  unsigned long timestamp;
  unsigned long version;
  floatlong doSatVolt;
  floatlong doSatTemp;
};

// from EC
    float _ecvalue;
    float _ecvalueRaw;
    float _kvalue;
    float _voltage;
/** Main data structure
 * Most sensors keep a short array of values and average or median the values.
 */

struct data_t {
  struct cal_data_t calData;
  int orpArray[SCOUNT];
  int tdsArray[SCOUNT];
  int doArray[SCOUNT];
  float turb;
  float temp;
  float pH;
  float ec;
  float orp;
  float dox;
  float tds;
  //* Calibration - which sensor to calibrate
  cal_mode_t calMode_;
  //* State of calibration
  cal_state_t calState_;
  //* Number of measurements to calibrate when calState_ == CAL_STATE_MEASURE
  int calMmts_;
  /** Array index and number of valid entries; stepped in parallel for all arrays */
  byte indx_, imax_;
} sensor_data;





const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
                14.46, 14.22, 13.82, 13.44, 13.09,
                12.74, 12.42, 12.11, 11.81, 11.53,
                11.26, 11.01, 10.77, 10.53, 10.30,
                10.08, 9.86,  9.66,  9.46,  9.27,
                9.08,  8.90,  8.73,  8.57,  8.41,
                8.25,  8.11,  7.96,  7.82,  7.69,
                7.56,  7.43,  7.30,  7.18,  7.07,
                6.95,  6.84,  6.73,  6.63,  6.53,
                6.41
};




/** Status indicator, for the LED
 * This structure contains the bit numbers and also indicate how many times the LED flashes
 * to indicate this status.
 */

enum status_t {
  STATUS_AOK = 0,
  /** ORP is not calibrated */
  STATUS_ORPCAL = 2,
  /** Failure of temperature sensor serial bus */
  STATUS_TMPSER = 3,
  /** No USB serial outward */
  STATUS_NOUSB = 4,
  // Need DO calibration
  STATUS_DOCAL = 5
};


struct status_led_t {
    /** Current status, OR of all the flags */
    unsigned long mask;
    /** Ticks for the next display event (milliseconds) */
    unsigned long tick;
} global_status;



// Given non-empty array and its length, return the median value
int getMedianNum(int[], int);

// Return the average value from an array
float getAvgArray(int const *const, int);

// Read the temperature; returning ERRORTEMP if something is wrong
// Also updates the STATUS_TMPSER flag in global_status
float readTemperature();

// Read an analog pin, returning the value in mV
float readAnalogMilli(int);

//* parse a command string returned on the UART (actually USB) serial
void uartParse(char const *, unsigned int, data_t *, status_led_t *);

//* Calibration routine for DO
void doCalibration(cal_data_t *);

//* Increment for cal_state_t
cal_state_t nextCalState(cal_state_t);

//* Read values from EEPROM
void readCalData(cal_data_t *, status_led_t *);

//* Write calibration data values to EEPROM
void writeCalData(cal_data_t *, status_led_t *);

//* Reading serial bus data from USB
// Parameters are the buffer and the length minus one (as in the longest possible string)
boolean serialDataAvailable(char *, const unsigned int);

/** Main function for performing all measurements; updates sensor_data and global_status structures */
void measureAll(data_t *, status_led_t *);

/** Output all values to USB in JSON format */
void printAll(data_t *, status_led_t *);

/* Update status LED, intended to be called every 50ms
 *  This call also updates the timer value (global_status.tick), readying for the next tick
 */

void display(status_led_t *);



void
setup()
{
  // Data is sent onward through USB
  Serial.begin(115200);
  // Give serial (= USB) a bit of time if it's present
  if(!Serial) delay(50);
  // Sensor data initialisation
  sensor_data.indx_ = sensor_data.imax_ = 0;
  sensor_data.turb = sensor_data.temp = 0.0f;
  // Status data initialisation
  sensor_data.calMode_ = CAL_MODE_NONE;
  sensor_data.calState_ = CAL_STATE_INIT;
  global_status.mask = 0;
  global_status.tick = millis()+100;
  // All six ADCs are used for input
  int pins[] = { TURB_PIN, PH_PIN, EC_PIN, DO_PIN, ORP_PIN, TDS_PIN, TMP_PIN, -1 };
  int *pin = pins;
  while( *pin >=0 )
    pinMode(*pin++, INPUT);

  // The on-board LED is used for status
  pinMode(LED_PIN, OUTPUT);

  readCalData(&sensor_data.calData, &global_status);
  /** If you do not have the appropriate sensor, the Arduino will  hang
   ** at these commands **/

  ph.begin();
  ec.begin();
}




void
loop()
{
  static unsigned long measure = millis() + MEASURE_INTERVAL_MILLI;
  // XXX: Handle the 49.7 day rollover?
  static char receiveBuffer[30];    // store the serial command from USB
  // String buffer hack to communicate with ph and ec modules
  char hack[6];

  if(millis() > measure) {
    measure += MEASURE_INTERVAL_MILLI;
    measureAll(&sensor_data, &global_status);

    switch(sensor_data.calMode_) {
    case CAL_MODE_NONE:
        printAll(&sensor_data, &global_status);
      break;
    /*
  case CAL_MODE_ORP:
    Serial.println(">>> ORP calibration mode <<<");
    break;
    */
    case CAL_MODE_DO:
      switch(sensor_data.calState_) {
      case CAL_STATE_INIT:
        Serial.println(">>> DO calibration mode <<<");
        Serial.println(">>> Please put the probe in saturated oxygen water, then press return (or type EXIT to quit)");
        // Advance to next state to prevent re-advertising
        sensor_data.calState_ = CAL_STATE_WAIT;
      break;
      case CAL_STATE_WAIT:
        // Need to perform this many measurements to completely flush buffer
        sensor_data.calMmts_ = SCOUNT;
        break;
      case CAL_STATE_MEASURING:
        if(sensor_data.calMmts_ > 0) {
	        Serial.print("Calibrating..."); Serial.print(sensor_data.calMmts_); Serial.println("");
	        --sensor_data.calMmts_;
        } else {
	        Serial.println("Finished calibrating; give command SAVE or EXIT");
	        sensor_data.calState_ = CAL_STATE_FINAL;
        }
        break;
      case CAL_STATE_FINAL:
        break;
      }
      break;
    case CAL_MODE_TEST:
      switch(sensor_data.calState_) {
      case CAL_STATE_INIT:
        Serial.println(">>> TEST calibration mode <<<");
        Serial.println(">>> press return to proceed <<<");
        // Advance to next state to prevent re-advertising
        sensor_data.calState_ = CAL_STATE_WAIT;
      break;
      case CAL_STATE_WAIT:
        // Need to perform this many measurements to completely flush buffer
        sensor_data.calMmts_ = SCOUNT;
        break;
      case CAL_STATE_MEASURING:
        if(sensor_data.calMmts_ > 0) {
	        Serial.print("Calibrating..."); Serial.print(sensor_data.calMmts_); Serial.println("");
	        --sensor_data.calMmts_;
        } else {
	        Serial.println("Finished calibrating; give command SAVE or EXIT");
	        sensor_data.calState_ = CAL_STATE_FINAL;
        }
        break;
      case CAL_STATE_FINAL:
        break;
      }
      break;
    case CAL_MODE_EC:
      switch(sensor_data.calState_) {
      case CAL_STATE_INIT:
        Serial.println(">>> EC calibration mode <<<");
        Serial.println(">>> Press return to calibrate <<<");
        sensor_data.calState_ = CAL_STATE_WAIT;
        break;
      case CAL_STATE_WAIT:
        break;
      case CAL_STATE_MEASURING:
        strncpy(hack, "CALEC", 6); hack[5] = '\0';
        ec.calibration(readAnalogMilli(EC_PIN), sensor_data.temp, hack);
        sensor_data.calState_ = CAL_STATE_FINAL;
        break;
      case CAL_STATE_FINAL:
      break;
      }
    case CAL_MODE_PH:
      switch(sensor_data.calState_) {
      case CAL_STATE_INIT:
        Serial.println(">>> PH calibration mode <<<");
        Serial.println(">>> Press return to calibrate <<<");
        sensor_data.calState_ = CAL_STATE_WAIT;
        break;
      case CAL_STATE_WAIT:
        break;
      case CAL_STATE_MEASURING:
        strncpy(hack, "CALPH", 6); hack[5] = '\0';
        ph.calibration(readAnalogMilli(PH_PIN), sensor_data.temp, hack);
        sensor_data.calState_ = CAL_STATE_FINAL;
        break;
      case CAL_STATE_FINAL:
        break;
      }
      break;
    }
  }
  if(Serial) {
    bitClear(global_status.mask, STATUS_NOUSB);
    if(serialDataAvailable(receiveBuffer, 29)) {
      // If we are calibrating, a CRLF says move to next state
      if(sensor_data.calMode_ && (*receiveBuffer == 0x0A || *receiveBuffer == 0x0D)) {
        sensor_data.calState_ = nextCalState(sensor_data.calState_);
        if( sensor_data.calState_ >= CAL_STATE_FINAL ) {
          sensor_data.calMode_ = CAL_MODE_NONE;
          sensor_data.calState_ = CAL_STATE_INIT;
        }
      } else {
        // Not in calibration OR a non-empty command received
        uartParse(receiveBuffer, 30, &sensor_data, &global_status);
      }
      measure = millis() + MEASURE_INTERVAL_MILLI;
    }
  } else {
    bitSet(global_status.mask, STATUS_NOUSB);
  }

  // Call LED update
  if(millis() >= global_status.tick)
    display(&global_status);
}

/*
boolean
serialDataAvailable(char *buf, const unsigned int len)
{
  char *p = buf, c;
  memset(buf, 0, len+1);
  unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 ) {
    if (millis() - receivedTimeOut > 500U) {
      memset(buf, 0, len+1);
      p = buf;
    }
    receivedTimeOut = millis();
    c = Serial.read();
    if (c == '\n' || (p-buf) == len) {
      strupr(buf);
      return true;
    } else {
      *p++ = c;
    }
  }
  return false;
}
*/
boolean
serialDataAvailable(char *buf, const unsigned int len)
{
  if(Serial.available() > 0) {
    memset(buf, 0, len+1);
    strncpy(buf, Serial.readString().c_str(), len);
    return true;
  }
  return false;
}


void
uartParse(char const *buffer, unsigned int len, data_t *data, status_led_t *)
{
  const size_t bufsize = 10;
  char cmdbuf[bufsize];
  if(!strncmp(buffer, "SAVE", 4)) {
    switch(data->calMode_) {
    case CAL_MODE_EC:
      strncpy(cmdbuf, "EXITEC", bufsize); cmdbuf[bufsize-1] = '\0';
      ec.calibration(readAnalogMilli(EC_PIN), data->temp, cmdbuf);
      break;
    case CAL_MODE_PH:
      strncpy(cmdbuf, "EXITPH", bufsize); cmdbuf[bufsize-1] = '\0';
      ph.calibration(readAnalogMilli(PH_PIN), data->temp, cmdbuf);
      break;
    }
    Serial.println("Finished calibration");
    data->calMode_ = CAL_MODE_NONE;
    return;
  }

  if(!strncmp(buffer, "EXIT", 4)) {
    Serial.println("Exiting calibration");
    data->calMode_ = CAL_MODE_NONE;
    return;
  }
  if(strncmp(buffer, "CAL", 3)) {
    Serial.println("Unknown command; expected \"CAL\" or \"EXIT\" or \"SAVE\"");
    return;
  }
  if(!strncmp(buffer, "CAL DO", 6)) {
    data->calMode_ = CAL_MODE_DO;
    return;
  }
  /*
  if(!strncmp(buffer, "CAL ORP", 3)) {
    data->calMode_ = CAL_MODE_ORP;
    return;
  }
  */
  if(!strncmp(buffer, "CAL EC", 6)) {
    Serial.println(F(">>>Please put the probe into the 12.88ms/cm buffer solution<<<"));

    strncpy(cmdbuf, "ENTEREC", bufsize); cmdbuf[bufsize-1] = '\0';
    ec.calibration(readAnalogMilli(EC_PIN), data->temp, cmdbuf);
    data->calMode_ = CAL_MODE_EC;
    return;
  }
  if(!strncmp(buffer, "CAL PH", 6)) {
    strncpy(cmdbuf, "ENTERPH", bufsize); cmdbuf[bufsize-1] = '\0';
    ph.calibration(readAnalogMilli(PH_PIN), data->temp, cmdbuf);
    data->calMode_ = CAL_MODE_PH;
    return;
  }
  if(!strncmp(buffer, "CAL TEST", 8)) {
    data->calMode_ = CAL_MODE_TEST;
    return;
  }
  if(!data->calMode_)
    Serial.println("Usage: \"CAL DO\", \"CAL PH\", \"CAL EC\"");
}


int
getMedianNum(int bArray[], int iFilterLen) 
{
    int bTab[iFilterLen];
    for (int i = 0; i<iFilterLen; i++) {
      bTab[i] = bArray[i];
    }
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
      for (i = 0; i < iFilterLen - j - 1; i++) {
        if (bTab[i] > bTab[i + 1]) {
          bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
          bTab[i + 1] = bTemp;
       }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}


/* This function is needed because enums are not incrementable.
 *  Despite being integers - a bit C++ish.
 *  Do make sure it matches the definition though.
 */

cal_state_t
nextCalState(cal_state_t c)
{
  switch(c) {
  case CAL_STATE_INIT:
    return CAL_STATE_WAIT;
  case CAL_STATE_WAIT:
    return CAL_STATE_MEASURING;
  case CAL_STATE_MEASURING:
    return CAL_STATE_FINAL;
  }
  return CAL_STATE_FINAL;
}


void
readCalData(cal_data_t *calData, status_led_t *status)
{
  EEPROM.get(CalDataAddress, *calData);  
  if( calData->magic != CalDataMagic ) {
    // Calibration needed
    bitSet(status->mask, STATUS_ORPCAL);
    bitSet(status->mask, STATUS_DOCAL);
    // Hardwired defaults
    calData->magic = CalDataMagic;
    calData->timestamp = 0UL;     // XXX FIXME
    calData->version = DISTINCT_VERSION;
    calData->doSatVolt.f = 1127.6;
    calData->doSatTemp.f = 25.0;
  } else {
    if( calData->version < DISTINCT_VERSION ) {
      if(Serial) Serial.println("Warning: loading data from earlier version");
    } else if(calData->version > DISTINCT_VERSION ) {
      if(Serial) Serial.println("Warning: loading data written by a future version");
    }
  }
}


void
writeCalData(cal_data_t *data, status_led_t *)
{
  EEPROM.put(CalDataAddress, *data);
}
  

float
readTemperature()
{
    static OneWire ds(TMP_PIN);  // on digital pin 2

    byte data[12];
    byte addr[8];

    if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return ERRORTEMP;
    }

    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return ERRORTEMP;
    }

    if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return ERRORTEMP;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44,1); // start conversion, with parasite power on at the end

    byte present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE); // Read Scratchpad

  
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
    }
  
    ds.reset_search();
  
    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); //using two's complement
    float TemperatureSum = tempRead / 16;

    bitClear(global_status.mask, STATUS_TMPSER);
    return TemperatureSum; //add your code here to get the temperature from your temperature sensor
}


float
averagearray(int const *const arr, int number)
{
  int i;
  int max,min;
  float avg;
  long amount=0;
  if(number <= 0) {
    printf("Error number for the array to average!/n");
    return 0;
  }
  if(number<5) {   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  } else {
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2; i<number; i++) {
      if(arr[i] < min) {
	amount += min;        //arr<min
	min = arr[i];
      } else {
	if(arr[i] > max){
          amount += max;    //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }
    }
    avg = ((float)amount)/(number-2);
  }
  return avg;
}


float
readAnalogMilli(int analog_pin)
{
  int v = analogRead(analog_pin);
  // int m = analogReadResolution();
  float c = 5000.0 / 1024.0;
  return v*c;
}


void
measureAll(data_t *sense, status_led_t *status)
{
  // hack for TDS measurement
  static float averageVoltage = 0.0f;

  float temperature = readTemperature();
  // code uses -1000 as error
  if(temperature == ERRORTEMP) {
    bitSet(status->mask, STATUS_TMPSER);
    temperature = 20.0f; /* fallback for compensation */
  } else {
    bitClear(status->mask, STATUS_TMPSER);
    sense->temp = temperature;
  }

  float voltPh = readAnalogMilli(PH_PIN);
  sense->pH = ph.readPH(voltPh, temperature);

  float voltEC = readAnalogMilli(EC_PIN);
  sense->ec = ec.readEC(voltEC, temperature);

  // Read the three pins into arrays; these are read as integer values (0-1023)
  sense->orpArray[sense->indx_] = analogRead(ORP_PIN);
  sense->tdsArray[sense->indx_] = analogRead(TDS_PIN);
  sense->doArray[sense->indx_] = analogRead(DO_PIN);
  if( ++sense->indx_ == SCOUNT )
    sense->indx_ = 0;
  if( sense->imax_ < SCOUNT )
    ++sense->imax_;

  //convert the analog value to orp according the circuit
  sense->orp = ((30*VREF)-(75.0*averagearray(sense->orpArray, sense->imax_)*VREF/1024.0))/75-OFFSET;
    
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  sense->turb = readAnalogMilli(TURB_PIN)/1000.0;

  //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationCoefficient=1.0+0.02*(temperature-25.0);
  //temperature compensation
  averageVoltage = getMedianNum(sense->tdsArray, sense->imax_) * (VREF / 1024.0 / 1000.0);
  float compensationVoltage = averageVoltage/compensationCoefficient;
  //convert voltage value to tds value
  sense->tds = ((133.42*compensationVoltage - 255.86)*compensationVoltage + 857.39)*compensationVoltage*0.5;
  
  //calculate doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
  averageVoltage = getMedianNum(sense->doArray, sense->imax_) * (VREF / 1024.0 / 1000.0);
  int doIndex = (int)(sense->calData.doSatTemp.f+0.5);
  // safety checks that index is in range
  doIndex = doIndex < 0 ? 0 : doIndex;
  doIndex = doIndex > 40 ? 40 : doIndex;
  sense->dox = pgm_read_float_near( &SaturationValueTab[doIndex] ) * averageVoltage / sense->calData.doSatVolt.f;
}  


void
printAll(data_t *sense, status_led_t *status)
{
  Serial.print("{\"temp\":");
  if(bitRead(status->mask, STATUS_TMPSER))
    Serial.print("\"ERR\"");
  else
    Serial.print(sense->temp);
  Serial.print(",\"do\":"); 
  Serial.print(sense->dox);
  Serial.print(",\"turb\":"); Serial.print(sense->turb);
  Serial.print(",\"tds\":"); Serial.print(sense->tds);
  Serial.print(",\"orp\":"); Serial.print(sense->orp);
  Serial.print(",\"ec\":"); Serial.print(sense->ec);
  Serial.print(",\"pH\":"); Serial.print(sense->pH);
  Serial.println("}");
}


void
display(status_led_t *s)
{
    /** Which nonzero status bit is next to be displayed? */
    static byte which = 0;
    /** How many pulses yet to flash in this second? */
    static byte count = 1;
    /** Max number of pulses per time unit (ticks) */
    static byte pps = LED_FLASH_MAX;
    /** Where are we in the cycle (on/off) */
    static byte onoff = 1;

    // Schedule next update
    if(s->tick >= 0xFFFFFF00) /* 50 day rollover? */
        s->tick = 0;
    else
        s->tick += 100;

    // Do we need to turn the LED on?
    if(onoff) {
        onoff = 0;
        // where are we in the time unit
        if( pps-- ) {
            // do we need another pulse?
            if( count ) {
                --count;
                digitalWrite(LED_PIN, 1);
            }
            return;
        }

        // Reset to potential pulses per time interval
        pps = LED_FLASH_MAX;

        /* Need to turn on - find the next status to display */
        if( s-> mask == 0 ) {
            // Single flash says everything OK */
            count = 1;
        } else {
          // Find the next bit which is 1
          byte w = which + 1;
          while( !((1UL << w) & s->mask) ) {
            if( ++w == 8*sizeof(unsigned long) )
                w = 0;
          }
          which = count = w;
      }
    } else {                  // onoff == 0
      // turn it off and set next call to turn it on
      digitalWrite(LED_PIN, 0);
      onoff = 1;
    }
}



/** 0.1alpha - first release
 *  
 *  0.1alpha-1
 *  - slowed LED status and added one to errors (so OK is 1)
 *  - turb divided by 1000
 *  - slower flashes by factor 2
 *  - tds missing piece reinstated
 *  - orp formula fix
 *  - (one) temp print bug fixed
 *  - missing do average reinstated and index safety check
 *  - EC and pH added (accidentally omitted previously)
 *  
 *  0.2alpha
 *  - turb now updated in sensor data structure
 *  - tidying DO code and debug values
 *  - better use bit functions instead of C style 1UL << bitno mask
 *  - more calibration data can now be saved (in theory) and it is versioned
 *  - more consistent calibration commands, but still needs a reference clock,
 *    albeit with higher complexity (calState and calMode)
 *  - restructed loop() making it much cleaner
 *  - indentation (Arduino seems to prefer 2) is now consistent in more places
 *    - ordinarily I tend to work on 4s with tabs for 8, Emacs style.
 *  - bugfix - TMP_PIN mode wasn't set in setup()
 *
 * 0.3
 *  - never outputs anything on USB other than JSON
 */
