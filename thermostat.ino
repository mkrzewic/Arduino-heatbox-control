#include <RotaryEncoder.h>
#include <LiquidCrystal.h>
#include <digitalWriteFast.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#undef REQUIRESALARMS
#include <EEPROM.h>
#include <CRC32.h>

//#define DEBUG

//pin defs
constexpr int8_t encoderPin1 = 11;
constexpr int8_t encoderPin2 = 10;
constexpr int8_t encoderButtonPin = 12;

constexpr int8_t oneWirePinMain = A1;
constexpr int8_t oneWirePinHeater = A2;
constexpr int8_t oneWirePinRelay = A3;

constexpr int8_t relaySSRpin = 13;
constexpr int8_t relayClickPin = A0;

//
constexpr unsigned long ULONG_MAX{0xffffffff};
constexpr unsigned long thermoMainSamplingPeriod = 2000; //ms
constexpr unsigned long thermoHeaterSamplingPeriod = 500; //ms
constexpr unsigned long thermoRelaySamplingPeriod = 500; //ms
constexpr uint8_t iResolutionHeaterT = 2;
constexpr uint8_t iResolutionRelayT = 2;
constexpr unsigned int uiSpringBackDelay = 3500;

//each sensor on a separate bus to avoid complications with sddresses when exchanging sensors
OneWire oneWireMain(oneWirePinMain);
OneWire oneWireHeater(oneWirePinHeater);
OneWire oneWireRelay(oneWirePinRelay);
DallasTemperature thermoMain(&oneWireMain);
DallasTemperature thermoHeater(&oneWireHeater);
DallasTemperature thermoRelay(&oneWireRelay);
DeviceAddress addrMain;
DeviceAddress addrHeater;
DeviceAddress addrRelay;

RotaryEncoder encoder(encoderPin1, encoderPin2);
LiquidCrystal lcd(9,8,7,6,5,4);

volatile unsigned long bounceTimer{0};
uint8_t modeButton[2] = {1};
int8_t knobPosition[2] = {0};

constexpr int8_t nResolutionsT = 3;
constexpr uint8_t stepT[nResolutionsT] =                    {16, 32, 64};
constexpr uint8_t tempSensorResolution[nResolutionsT] =     {11,    10,     9};
constexpr uint8_t displayPrecision[nResolutionsT] =         {3,     2,      1};

// settable parameters
struct Param_t {
  int16_t targetT{5120};
  int16_t limitHeaterT{8960};
  int16_t hysteresisT{64};
  int8_t heatingMode{1}; //-1 or 1 for cooling/heating
  uint8_t iStepT{2};
  int16_t maxRelayT{12672};
  int16_t maxTargetT{7680};
};

Param_t param{};

int16_t heaterMaxT{0};
int16_t heaterMinT(0);
int16_t mainT = {0};
int16_t heaterT = {0};
int16_t relayT = {0};
int8_t slopeT = {-1};
int8_t slopeH = {-1};
int8_t running = {-1}; //start in the off state
int8_t wasrunning = {0};
int8_t heaterIsOn{0}, heaterWasOn{0};
bool saveSettings{false};
int devCountMain{0};
int devCountHeater{0};
int devCountRelay{0};
uint32_t eepromCRC{0};
uint32_t addressEEPROMcrc = {0};
uint32_t addressEEPROMSettings = {addressEEPROMcrc + sizeof(addressEEPROMcrc)};

unsigned long timeStartMainTConversion{0};
unsigned long timeStartHeaterTConversion{0};
unsigned long timeStartRelayTConversion{0};
unsigned long timeStartMainTReadout{ULONG_MAX};
unsigned long timeStartHeaterTReadout{ULONG_MAX};
unsigned long timeStartRelayTReadout{ULONG_MAX};

enum class State_t {run, setTargetT, setLimitHeaterT, error, man,
                    setHeatingMode, setTargetDeltaT, setTemperatureStep,
                    saveSettings, showTemperatures,  setMaxTargetT, setLimitRelayT};
struct UI_t {
  unsigned long lastChange{0};
  State_t state{State_t::run};
  State_t lastState{State_t::man};
  bool redraw{true};
  const char* errorMsg;

  void changeState(State_t newState){
    lastState = state;
    state = newState;
    lastChange = millis();
    redraw = true;
  }

  void tick() {redraw = true; lastChange = millis();}

  void error(const char* message ) {
    running=-1;
    errorMsg = message;
    state = State_t::error;
    lastChange = 0;
    redraw = true;
  }

  template<typename T>
    T intpow(T base, T exponent) {
      T ret{1};
      while (exponent>0) {
        ret *= base;
        exponent--;
      }
      return ret;
    }

  // TODO: minimize chance of overflow, multiplying stuff by 1000 goes to large numbers potentially
  template<typename T, typename U, typename V, typename W, typename X>
    void printAsFloat(T const x, X const valPerUnit, U& out, W const places, V base) {
      out.print(x / valPerUnit, base);
      out.print(".");
      // TODO: here be dragons
      auto nb = out.print(abs(int32_t(x % valPerUnit)*intpow<int16_t>(10,places)/valPerUnit), base);
      for (; nb < places; ++nb) {
        out.print("0");
      }
    }

  template<typename U>
    void printDallasTempC(int32_t const raw, U& out, uint8_t const places) {
      printAsFloat(int32_t(raw), uint16_t(128), out, places, DEC);
      out.print(char(223));
      out.print("C ");
    }

  void update(LiquidCrystal& lcd) {
    if (!redraw) return;
    redraw = false;
    if (lastState!=state) { lcd.clear(); lastState = state; }
    lcd.home();
    switch(state) {
      case State_t::run:
        if (running==-1 && relayT > param.maxRelayT) {
          lcd.print("relay overheated");
        } else {
          if (slopeT>0 && param.heatingMode>0) {
            lcd.print(heaterIsOn>0?"HEAT":"heat");
          } else if (slopeT<0 && param.heatingMode<0) {
            lcd.print(heaterIsOn>0?"COOL":"cool");
          } else {
            lcd.print("steady");
          }
          lcd.print("    ");
          lcd.setCursor(8,0);
          printDallasTempC(mainT, lcd, displayPrecision[param.iStepT]);
        }
        lcd.setCursor(0,1);
        lcd.print(char(126));
        lcd.setCursor(8,1);
        printDallasTempC(param.targetT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setTargetT:
        lcd.print("Set temperature");
        lcd.setCursor(6,2);
        printDallasTempC(param.targetT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setLimitHeaterT:
        param.heatingMode>0 ? lcd.print("Max heater T") : lcd.print("Min cooler T");
        lcd.setCursor(6,1);
        printDallasTempC(param.limitHeaterT, lcd, displayPrecision[iResolutionHeaterT]);
        break;
      case State_t::setHeatingMode:
        lcd.print("Set cool <> heat");
        lcd.setCursor(8,1);
        lcd.print(param.heatingMode<0?"cool":"heat");
        break;
      case State_t::setTargetDeltaT:
        lcd.print("T variation");
        lcd.setCursor(6,1);
        printDallasTempC(param.hysteresisT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setTemperatureStep:
        lcd.print("T precision");
        lcd.setCursor(6, 1);
        printDallasTempC(stepT[param.iStepT], lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::showTemperatures:
        param.heatingMode > 0 ? lcd.print("heater:") : lcd.print("cooler");;
        lcd.setCursor(8,0);
        if (devCountHeater==0) {
          lcd.print("?");
        } else {
          printDallasTempC(heaterT, lcd, displayPrecision[iResolutionHeaterT]);
        }
        lcd.setCursor(0,1);
        lcd.print("relay:");
        lcd.setCursor(8,1);
        printDallasTempC(relayT, lcd, displayPrecision[iResolutionRelayT]);
        break;
      case State_t::saveSettings:
        lcd.print("Save as defaults");
        lcd.setCursor(2,1); (saveSettings)?lcd.print("press to save"):lcd.print("no");
        break;
      case State_t::setLimitRelayT:
        lcd.print("Max relay T");
        lcd.setCursor(8,1);
        printDallasTempC(param.maxRelayT, lcd, displayPrecision[iResolutionRelayT]);
        break;
      case State_t::setMaxTargetT:
        param.heatingMode>0 ? lcd.print("Max settable T") : lcd.print("Min settable T");
        lcd.setCursor(8,1);
        printDallasTempC(param.maxTargetT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::error:
        lcd.print("error:");
        lcd.setCursor(0,1);
        lcd.print(errorMsg);
        exit(0);
        break;
      case State_t::man:
        lcd.print("    Press to    ");
        lcd.setCursor(0,1);
        lcd.print("change settings");
        break;
    }
  }
};

UI_t ui{};

// handle interrupt on bank 0
ISR(PCINT0_vect) {
  encoder.tick(); // just call tick() to check the state.
  bounceTimer=millis();
}

void SaveSettings() {
  saveSettings=false;
  EEPROM.put(addressEEPROMSettings, param);
  eepromCRC = CRC32::calculate(&param, 1);
  EEPROM.put(addressEEPROMcrc, eepromCRC);
  lcd.clear();
  lcd.print("saved");
  delay(1000);
}

bool RestoreSettings() {
  Param_t tmp;
  EEPROM.get(addressEEPROMSettings, tmp);
  EEPROM.get(addressEEPROMcrc, eepromCRC);
  if (CRC32::calculate(&tmp,1) == eepromCRC) {
    param = tmp;
    return true;
  }
  return false;
}

void copyAddress(DeviceAddress source, DeviceAddress target) {
  for (int i=0; i<8; ++i) {
    target[i] = source[i];
  }
}

bool equalAddress(DeviceAddress source, DeviceAddress target) {
  for (int i=0; i<8; ++i) {
    if (target[i] != source[i]) return false;
  }
  return true;
}

void initSensors() {
  thermoMain.begin();
  thermoHeater.begin();
  thermoRelay.begin();
  devCountMain = thermoMain.getDeviceCount();
  devCountHeater = thermoHeater.getDeviceCount();
  devCountRelay = thermoRelay.getDeviceCount();
  if (devCountMain==0) {ui.error("no main sensor");}
  if (devCountRelay==0) {ui.error("no relay sensor");}
  if (!thermoMain.getAddress(addrMain,0)) {ui.error("no main sensor");}
  if (!thermoHeater.getAddress(addrHeater,0)) {}
  if (!thermoRelay.getAddress(addrRelay,0)) {ui.error("no relay sensor");}
  thermoMain.setResolution(tempSensorResolution[param.iStepT]);
  thermoHeater.setResolution(tempSensorResolution[iResolutionHeaterT]);
  thermoRelay.setResolution(tempSensorResolution[iResolutionRelayT]);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  lcd.begin(16,2);
  if (!RestoreSettings()) {
    lcd.clear();
    lcd.print("  EEPROM error");
    lcd.setCursor(0,1);
    lcd.print("fallback default");
    delay(1000);
    SaveSettings();  //maybe a new board or something went wrong, store defaults
  }

  lcd.clear();
  lcd.print("       MKr");

  pinMode(encoderButtonPin, INPUT_PULLUP);

  //enable interrupts on bank b
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4);

  initSensors();

  // switch the main switch ON, the delay before switching the SSR
  // is covered by the reading of temperatures below (>90ms);
  pinMode(relayClickPin, OUTPUT);  //goes to LOW by default, switching on the relay
  pinMode(relaySSRpin, OUTPUT);

  //initial read of temperatures.
  thermoMain.setWaitForConversion(true);
  thermoHeater.setWaitForConversion(true);
  thermoRelay.setWaitForConversion(true);

  thermoMain.requestTemperatures();
  mainT = thermoMain.getTemp(addrMain);

  thermoHeater.requestTemperatures();
  heaterT = thermoHeater.getTemp(addrHeater);

  thermoRelay.requestTemperatures();
  relayT = thermoRelay.getTemp(addrRelay);

  thermoMain.setWaitForConversion(false);
  thermoHeater.setWaitForConversion(false);
  thermoRelay.setWaitForConversion(false);

  heaterMaxT = max(param.targetT, param.limitHeaterT);
  heaterMinT = min(param.targetT, param.limitHeaterT);
} // setup()

// Read the current position of the encoder and print out when updated.
void loop()
{
#ifdef DEBUG
  static unsigned long tajm{0},acc{0};
  static unsigned long counter{0};
  tajm=micros();
#endif

  ui.update(lcd);

  knobPosition[0] = encoder.getPosition();
  if (knobPosition[1]!=knobPosition[0]) {
    knobPosition[1]=knobPosition[0];
    ui.tick();
    switch (ui.state) {
      case State_t::run:
        if  (encoder.getDirection() == RotaryEncoder::Direction::CLOCKWISE) {
          ui.changeState(State_t::man);
        } else {
          ui.changeState(State_t::showTemperatures);
        }
        break;
      case State_t::setTargetT:
        param.targetT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.iStepT];
        if (param.heatingMode*param.targetT > param.heatingMode*param.maxTargetT) { param.targetT = param.maxTargetT; }
        break;
      case State_t::setLimitHeaterT:
        param.limitHeaterT += static_cast<int8_t>(encoder.getDirection()) * stepT[iResolutionHeaterT];
        break;
      case State_t::setHeatingMode:
        param.heatingMode = static_cast<int8_t>(encoder.getDirection());
        break;
      case State_t::setTargetDeltaT:
        param.hysteresisT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.iStepT];
        if (param.hysteresisT<stepT[param.iStepT]) { param.hysteresisT = stepT[param.iStepT]; }
        break;
      case State_t::setTemperatureStep:
        static int8_t dir{0};
        dir = static_cast<int8_t>(encoder.getDirection());
        if ((dir > 0) && (param.iStepT == nResolutionsT-1)) {
        } else if ((dir < 0) && (param.iStepT == 0)) {
        } else {
          param.iStepT = param.iStepT + dir;
        }
        thermoMain.setResolution(tempSensorResolution[param.iStepT]);
        if (param.hysteresisT < stepT[param.iStepT]) { param.hysteresisT = stepT[param.iStepT]; }
        break;
      case State_t::saveSettings:
        static_cast<int8_t>(encoder.getDirection())==1 ? saveSettings=true : saveSettings=false ;
        break;
      case State_t::setLimitRelayT:
        param.maxRelayT += static_cast<int8_t>(encoder.getDirection()) * stepT[iResolutionRelayT];
        break;
      case State_t::setMaxTargetT:
        param.maxTargetT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.iStepT];
        break;
      case State_t::showTemperatures:
        ui.changeState(State_t::man);
        break;
      default:
        break;
    }
    int16_t limitHeaterOther{0};
    limitHeaterOther = (param.targetT + param.limitHeaterT)/2;
    heaterMaxT = max(limitHeaterOther, param.limitHeaterT);
    heaterMinT = min(limitHeaterOther, param.limitHeaterT);
  }

  if ((ui.lastChange != 0) && ((millis()-ui.lastChange) > uiSpringBackDelay)) {
    ui.changeState(State_t::run);
    ui.lastChange = 0;
  }

  //read button
  if (bounceTimer!=0 && ((millis()-bounceTimer)>50)) {
    bounceTimer = 0;
    modeButton[1] = modeButton[0];
    modeButton[0] = digitalReadFast(encoderButtonPin);
    if (modeButton[0] < modeButton[1]) {
    switch(ui.state) {
      case State_t::run:
        ui.changeState(State_t::setTargetT);
        break;
      case State_t::setTargetT:
        ui.changeState(State_t::setLimitHeaterT);
        break;
      case State_t::setLimitHeaterT:
        ui.changeState(State_t::setTargetDeltaT);
        break;
      case State_t::setTargetDeltaT:
        ui.changeState(State_t::setTemperatureStep);
        break;
      case State_t::setTemperatureStep:
        ui.changeState(State_t::setMaxTargetT);
        break;
      case State_t::setMaxTargetT:
        ui.changeState(State_t::setLimitRelayT);
        break;
      case State_t::setLimitRelayT:
        ui.changeState(State_t::setHeatingMode);
        break;
      case State_t::setHeatingMode:
        ui.changeState(State_t::saveSettings);
        break;
      case State_t::saveSettings:
        if (saveSettings) { SaveSettings(); ui.changeState(State_t::run);}
        ui.changeState(State_t::run);
        break;
      case State_t::error:
        break;
      case State_t::man:
        ui.changeState(State_t::setTargetT);
        break;
      case State_t::showTemperatures:
        break;
      default:
        break;
    }
    }
  }

  // start conversion period for main
  if (millis() > timeStartMainTConversion) {
    thermoMain.requestTemperatures();
    timeStartMainTConversion = millis() + thermoMainSamplingPeriod;
    timeStartMainTReadout = millis() + thermoMain.millisToWaitForConversion();
  }

  //read teperatures from main sensor
  if (millis() > timeStartMainTReadout) {
    timeStartMainTReadout = ULONG_MAX;
    mainT = thermoMain.getTemp(addrMain);
    if (mainT == DEVICE_DISCONNECTED_RAW) {
      ui.error("bad main sensor");
    }
    ui.redraw = true;
  }

  // start conversion period for heater only if it is there
  if (millis() > timeStartHeaterTConversion && devCountHeater>0) {
    if (devCountHeater > 0) {
      thermoHeater.requestTemperatures();
      timeStartHeaterTConversion = millis() + thermoHeaterSamplingPeriod;
      timeStartHeaterTReadout = millis() + thermoHeater.millisToWaitForConversion();
    }
  }

  //read teperatures from heater sensor
  if (millis() > timeStartHeaterTReadout) {
    timeStartHeaterTReadout = ULONG_MAX;
    heaterT = thermoHeater.getTemp(addrHeater);
    if (heaterT == DEVICE_DISCONNECTED_RAW) {
      ui.error("bad heater sensor");
    }
  }

  // start conversion period for relay
  if (millis() > timeStartRelayTConversion) {
    thermoRelay.requestTemperatures();
    timeStartRelayTConversion = millis() + thermoRelaySamplingPeriod;
    timeStartRelayTReadout = millis() + thermoRelay.millisToWaitForConversion();
  }

  static int8_t skipRelayCheck{0};
  //read teperatures from relay sensor
  if (millis() > timeStartRelayTReadout) {
    timeStartRelayTReadout = ULONG_MAX;
    relayT = thermoRelay.getTemp(addrRelay);
    if (relayT == DEVICE_DISCONNECTED_RAW) {
      ui.error("bad relay sensor");
    }

    static uint8_t nWeirdValuesRelayT{0};
    if (relayT > 19200) { nWeirdValuesRelayT++; skipRelayCheck = 1;}
    else { skipRelayCheck = -1; }
    if (nWeirdValuesRelayT>10) ui.error("relay T weird");
  }

  //control the actual regulated temperature
  if (slopeT>0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeT *= (mainT >= (param.targetT + param.hysteresisT)) ? -1 : 1;
  } else {
    //if we're on decline, we flip when crossing lower bound
    slopeT *= (mainT <= (param.targetT - param.hysteresisT)) ? -1 : 1;
  }

  //control the heater temperature
  if (devCountHeater == 0) {
    slopeH = param.heatingMode;
  } else if (slopeH > 0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeH *= (heaterT >= heaterMaxT) ? -1 : 1;
  } else if (slopeH < 0) {
    //if we're on decline, we flip when crossing lower bound (or upper if we're cooling)
    slopeH *= (heaterT <= heaterMinT) ? -1 : 1;
  }

  //control main relay
  heaterIsOn = (((param.heatingMode * slopeT) > 0) && ((param.heatingMode * slopeH) > 0)) ? 1 : -1;

  // no need to access the hardware on every iteration, do it only when something changes
  if ((heaterIsOn != heaterWasOn) && (running > 0)) {
#ifdef DEBUG
  Serial.print("heatingMode: ");
  Serial.println(param.heatingMode);
  Serial.print("heaterIsOn: ");
  Serial.println(heaterIsOn);
  Serial.print("slopeT: ");
  Serial.println(slopeT);
  Serial.print("slopeH: ");
  Serial.println(slopeH);
  Serial.print("heaterMaxT: ");
  Serial.println(heaterMaxT);
  Serial.print("heaterMinT: ");
  Serial.println(heaterMinT);
  Serial.print("mainT: ");
  Serial.println(mainT);
  Serial.print("heaterT: ");
  Serial.println(heaterT);
#endif

    digitalWriteFast(relaySSRpin, (heaterIsOn > 0) ? HIGH : LOW);
    heaterWasOn = heaterIsOn;
    ui.tick();
  }

  // this is a safety feature
  if (relayT > param.maxRelayT && skipRelayCheck < 0) {
    running = -1;
  } else {
    running = 1;
  }

  //handle the connecting/disconnecting of the main relay based on the running condition
  //this essentially handles error conditions, so it is OK to use delay here
  if (running != wasrunning) {
    wasrunning = running;
    ui.tick();
    if (running<0) {
      //we switch off the relays in sequence
      digitalWriteFast(relaySSRpin, LOW);
      delay(20); //with 50Hz it takes up to 10ms to switch off (at next zero crossing);
      digitalWriteFast(relayClickPin, HIGH);
    } else {
      //switch on the main relay
      digitalWriteFast(relayClickPin,LOW);
      delay(100);  //allow mechanical relay to switch and stabilize, probably around 50-70ms
    }
  }

#ifdef DEBUG
  acc += micros() - tajm;
  counter++;
  if (counter == 10000) {
    Serial.println(acc/counter);
    counter = 0;
    acc = 0;
  }
#endif
} // loop ()
