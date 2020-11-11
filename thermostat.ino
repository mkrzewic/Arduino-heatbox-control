#include <RotaryEncoder.h>
#include <LiquidCrystal.h>
#include <digitalWriteFast.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#undef REQUIRESALARMS
#include <EEPROM.h>
#include <CRC32.h>

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

volatile uint8_t modeButton[2] = {1};
int8_t knobPosition[2] = {0};
volatile unsigned long bounceTimer{0};
volatile bool readEncoderButton{false};
uint8_t readMainTonce{0};
uint8_t readHeaterTonce{0};
uint8_t readRelayTonce{0};

constexpr int8_t nResolutionsT = 3;
float  stepT[nResolutionsT] =                    {0.125, 0.25, 0.5};
int8_t tempSensorResolution[nResolutionsT] =     {11,    10,     9};
int8_t displayPrecision[nResolutionsT] =         {3,     2,      1};

// settable parameters
struct Param_t {
  float targetT{40.};
  float limitHeaterT{70.};
  float hysteresisT{.5};
  int8_t heatingMode{1}; //-1 or 1 for cooling/heating
  uint8_t iStepT{2};
  float maxRelayT = 99.;
};

Param_t param{};

float heaterMaxT{0.};
float heaterMinT(0.);
float mainT = {0.};
float heaterT = {0.};
float relayT = {0.};
float errT = {0.};
int8_t slopeT = {1}; //invalid value 0 needed for bootstrap
int8_t slopeH = {1}; //invalid value 0 needed for bootstrap
int8_t running = {1};
bool saveSettings{false};
int devCountMain{0};
int devCountHeater{0};
int devCountRelay{0};
uint32_t eepromCRC{0};
uint32_t addressEEPROMcrc = {10};
uint32_t addressEEPROMSettings = {100};

unsigned long timeStartMainTConversion{0};
unsigned long timeStartHeaterTConversion{0};
unsigned long timeStartRelayTConversion{0};

enum class State_t {run, setT, setLimitHeaterT, error, man,
                    setHeatingMode, setTargetDeltaT, setTemperatureStep,
                    saveSettings, showTemperatures,  setLimitRelayT};
struct UI_t {
  unsigned long lastChange{0};
  State_t state{State_t::run};
  bool redraw{true};
  const char* errorMsg;

  void changeState(State_t newState){
    state = newState;
    lastChange = millis();
    redraw = true;
  }

  void tick() {redraw = true; lastChange = millis();}

  void error(const char* message ) {
    digitalWriteFast(relaySSRpin,LOW);
    digitalWriteFast(relayClickPin,HIGH);
    running=-1;
    errorMsg = message;
    state = State_t::error;
    lastChange = 0;
    redraw = true;
  }

  void update(LiquidCrystal& lcd) {
    if (!redraw) return;
    redraw = false;
    lcd.clear();
    switch(state) {
      case State_t::run:
        if (running==-1 && relayT > param.maxRelayT) {
          lcd.print("relay overheat");
        } else {
        lcd.print("now:");
        lcd.setCursor(8,0);
        lcd.print(mainT,displayPrecision[param.iStepT]);lcd.print(char(223));lcd.print("C");
        }
        lcd.setCursor(0,1);
        lcd.print("-->");
        lcd.setCursor(8,1);
        lcd.print(param.targetT,displayPrecision[param.iStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setT:
        lcd.setCursor(0,0);
        lcd.print("Set temperature");
        lcd.setCursor(6,2);
        lcd.print(param.targetT,displayPrecision[param.iStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setLimitHeaterT:
        lcd.setCursor(0,0);
        param.heatingMode>0?lcd.print("Heater max T"):lcd.print("Cooler min T");
        lcd.setCursor(6,1);
        lcd.print(param.limitHeaterT,displayPrecision[iResolutionHeaterT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setHeatingMode:
        lcd.print("Set cool <> heat");
        lcd.setCursor(8,1);
        lcd.print(param.heatingMode<0?"cool":"heat");
        break;
      case State_t::setTargetDeltaT:
        lcd.print("Hysteresis");
        lcd.setCursor(6,1);
        lcd.print(param.hysteresisT,displayPrecision[param.iStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setTemperatureStep:
        lcd.print("Temp resolution");
        lcd.setCursor(6, 1);
        lcd.print(stepT[param.iStepT], displayPrecision[param.iStepT]);
        break;
      case State_t::showTemperatures:
        lcd.print("heater:");
        lcd.setCursor(8,0);
        if (devCountHeater==0) {
          lcd.print("absent");
        } else {
          lcd.print(heaterT,displayPrecision[iResolutionHeaterT]);lcd.print(char(223));lcd.print("C");
        }
        lcd.setCursor(0,1);
        lcd.print("relay:");
        lcd.setCursor(8,1);
        lcd.print(relayT,displayPrecision[iResolutionRelayT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::saveSettings:
        lcd.print("Save as defaults");
        lcd.setCursor(2,1); (saveSettings)?lcd.print("press to save"):lcd.print("no");
        break;
      case State_t::setLimitRelayT:
        lcd.print("Max relay temp");
        lcd.setCursor(8,1);
        lcd.print(param.maxRelayT);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::error:
        lcd.setCursor(0,0);
        lcd.print("error:");
        lcd.setCursor(0,1);
        lcd.print(errorMsg);
        exit(0);
        break;
      case State_t::man:
        lcd.setCursor(0,0);
        lcd.print("    Press to    ");
        lcd.setCursor(0,1);
        lcd.print("change settings");
        break;
    }
  }
};

UI_t ui{};

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
  lcd.begin(16,2);
  //Serial.begin(9600);

  if (!RestoreSettings()) {
    lcd.clear();
    lcd.print("   Bad EEPROM");
    lcd.setCursor(0,1);
    lcd.print("fallback default");
    SaveSettings();  //maybe a new board or something went wrong, store defaults
  }

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
  thermoMain.requestTemperatures();
  mainT = thermoMain.getTempC(addrMain);

  thermoHeater.setWaitForConversion(true);
  thermoHeater.requestTemperatures();
  heaterT = thermoHeater.getTempC(addrHeater);

  thermoRelay.setWaitForConversion(true);
  thermoRelay.requestTemperatures();
  relayT = thermoRelay.getTemp(addrRelay);

  heaterMaxT = max(param.targetT, param.limitHeaterT);
  heaterMinT = min(param.targetT, param.limitHeaterT);
  running = 1;
} // setup()


// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT0_vect) {
  encoder.tick(); // just call tick() to check the state.
  bounceTimer=millis();
}

// Read the current position of the encoder and print out when updated.
void loop()
{
  //static unsigned long tajm = 0;
  //tajm=millis();
  //read knob

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
      case State_t::setT:
        param.targetT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.iStepT];
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
      case State_t::showTemperatures:
        ui.changeState(State_t::man);
        break;
      default:
        break;
    }
    float limitHeaterOther{0.};
    limitHeaterOther = (param.targetT + param.limitHeaterT)/2.;
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
        ui.changeState(State_t::setT);
        break;
      case State_t::setT:
        ui.changeState(State_t::setLimitHeaterT);
        break;
      case State_t::setLimitHeaterT:
        ui.changeState(State_t::setTargetDeltaT);
        break;
      case State_t::setTargetDeltaT:
        ui.changeState(State_t::setTemperatureStep);
        break;
      case State_t::setTemperatureStep:
        ui.changeState(State_t::setHeatingMode);
        break;
      case State_t::setHeatingMode:
        ui.changeState(State_t::setLimitRelayT);
        break;
      case State_t::setLimitRelayT:
        ui.changeState(State_t::saveSettings);
        break;
      case State_t::saveSettings:
        if (saveSettings) { SaveSettings(); ui.changeState(State_t::run);}
        ui.changeState(State_t::run);
        break;
      case State_t::error:
        break;
      case State_t::man:
        ui.changeState(State_t::setT);
        break;
      case State_t::showTemperatures:
        break;
      default:
        break;
    }
    }
  }

  // start conversion period for main
  if ((millis() - timeStartMainTConversion)>thermoMainSamplingPeriod) {
    if (devCountMain > 0) {
      timeStartMainTConversion = millis();
      thermoMain.setWaitForConversion(false);
      noInterrupts();
      thermoMain.requestTemperatures();
      interrupts();
      readMainTonce = 1;
    }
  }

  //read teperatures from main sensor
  if (readMainTonce==1 && ((millis() - timeStartMainTConversion) > thermoMain.millisToWaitForConversion())) {

    noInterrupts();
    mainT = thermoMain.getTempC(addrMain);
    interrupts();
    if (mainT == DEVICE_DISCONNECTED_C) {
      ui.error("bad main sensor");
    }
    readMainTonce=0;
    ui.redraw = true;
  }

  // start conversion period for heater
  if ((millis() - timeStartHeaterTConversion)>thermoHeaterSamplingPeriod) {
    if (devCountHeater > 0) {
      timeStartHeaterTConversion = millis();
      thermoHeater.setWaitForConversion(false);
      noInterrupts();
      thermoHeater.requestTemperatures();
      interrupts();
      readHeaterTonce = 1;
    }
  }

  //read teperatures from heater sensor
  if (readHeaterTonce==1 && ((millis() - timeStartHeaterTConversion) > thermoHeater.millisToWaitForConversion())) {

    noInterrupts();
    heaterT = thermoHeater.getTempC(addrHeater);
    interrupts();
    if (heaterT == DEVICE_DISCONNECTED_C) {
      ui.error("bad heater sensor");
    }
    readHeaterTonce=0;
  }

  // start conversion period for relay
  if ((millis() - timeStartRelayTConversion)>thermoRelaySamplingPeriod) {
    if (devCountRelay > 0) {
      timeStartRelayTConversion = millis();
      thermoRelay.setWaitForConversion(false);
      noInterrupts();
      thermoRelay.requestTemperatures();
      interrupts();
      readRelayTonce = 1;
    }
  }

  static int8_t skipRelayCheck{0};
  //read teperatures from relay sensor
  if (readRelayTonce==1 && ((millis() - timeStartRelayTConversion) > thermoRelay.millisToWaitForConversion())) {

    noInterrupts();
    relayT = thermoRelay.getTempC(addrRelay);
    interrupts();
    if (relayT == DEVICE_DISCONNECTED_C) {
      ui.error("bad relay sensor");
    }
    static uint8_t nWeirdValuesRelayT{0};
    if (relayT > 150.) { nWeirdValuesRelayT++; skipRelayCheck = 1;}
    else { skipRelayCheck = -1; }
    if (nWeirdValuesRelayT>10) ui.error("relay sensor weird");
    readRelayTonce=0;
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
  if (slopeH>0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeH *= (heaterT >= heaterMaxT) ? -1 : 1;
  } else {
    //if we're on decline, we flip when crossing lower bound (or upper if we're cooling)
    slopeH *= (heaterT <= heaterMinT) ? -1 : 1;
  }

  //control main relay
  static int8_t heaterIsOn{0}, heaterWasOn{0};
  heaterIsOn = ((param.heatingMode * slopeT) > 0) && ((param.heatingMode * slopeH) > 0) && (running > 0);
  // no need to access the hardware on every iteration, do it only when something changes
  if (heaterIsOn != heaterWasOn || !running) {
    digitalWriteFast(relaySSRpin, (heaterIsOn > 0) ? HIGH : LOW);
    heaterWasOn = heaterIsOn;
  }

  // this is a safety feature
  static int relayClickNow{HIGH};
  static int relayClickThen{LOW};
  relayClickNow = (relayT > param.maxRelayT)?HIGH:LOW;
  if (relayClickNow != relayClickThen && (skipRelayCheck<0)) {
    if (relayClickNow == LOW) {
      running = 1;
    } else {
      digitalWriteFast(relaySSRpin, LOW);
      running = -1;
    }
    ui.tick();
    digitalWriteFast(relayClickPin, relayClickNow);
    relayClickThen = relayClickNow;
  }
} // loop ()

// The End
