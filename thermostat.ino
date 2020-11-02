#include <RotaryEncoderMK.h>
#include <LiquidCrystal.h>
#include <digitalWriteFast.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#undef REQUIRESALARMS
#include <EEPROM.h>
#include <CRC32.h>
#include "thermistor.h"

//pin defs
constexpr int8_t encoderPin1 = 12;
constexpr int8_t encoderPin2 = 11;
constexpr int8_t encoderButtonPin = 10;

constexpr int8_t oneWirePinMain = A1;
constexpr int8_t oneWirePinHeater = A2;
constexpr int8_t oneWirePinRelay = A3;

constexpr int8_t relay1Pin = 13;
constexpr int8_t relay2Pin = A0;
//constexpr int8_t thermistorPin = A1;

//
constexpr unsigned long thermoSamplingPeriod = 2000; //ms
constexpr unsigned int uiSpringBackDelay = 3000;

//each sensor on a separate bus to avoid complications with sddresses when exchanging sensors
OneWire oneWireMain(oneWirePinMain);
OneWire oneWireHeater(oneWirePinHeater);
OneWire oneWireRelay(oneWirePinRelay);
DallasTemperature thermo(&oneWireMain);
DallasTemperature thermoHeater(&oneWireHeater);
DallasTemperature thermoRelay(&oneWireRelay);

RotaryEncoder encoder(encoderPin1, encoderPin2);
LiquidCrystal lcd(9,8,7,6,5,4);

volatile uint8_t modeButton[2] = {1};
int8_t knobPosition[2] = {0};
volatile unsigned long bounceTimer{0};
volatile bool readEncoderButton{false};
uint8_t readTonce{1};

constexpr int8_t nResolutionsT = 3;
float  stepT[nResolutionsT] =                    {0.125, 0.25, 0.5};
int8_t thermoResolution[nResolutionsT] =         {11,    10,     9};
int8_t displayPrecision[nResolutionsT] =         {3,     2,      1};

struct Param_t {
  DeviceAddress mainAddr   {0x28, 0xAA, 0x58, 0x48, 0x4B, 0x14, 0x01, 0xA3};
  DeviceAddress heaterAddr {0x28, 0xAA, 0x6C, 0x79, 0x4B, 0x14, 0x01, 0xFF};
  DeviceAddress relayAddr  {0x28, 0xAA, 0xE1, 0x43, 0x4B, 0x14, 0x01, 0x8B};
  float targetT{40.};
  float maxHeaterT{70.};
  float deltaHeaterT{10.};
  float targetTdelta{1.};
  int8_t heatingMode{1}; //-1 or 1 for cooling/heating
  int8_t whichStepT{2};
};

Param_t param{};

unsigned long conversionDelay = 750;

float currentT[2] = {0.};
float heaterT[2] = {0.};
float relayT[2] = {0.};
float errT = {0.};
int8_t slopeT[2] = {1,0}; //invalid value 0 needed for bootstrap
int8_t slopeH[2] = {1,0}; //invalid value 0 needed for bootstrap
bool running = {false};
bool saveSettings{false};
int devCount{0};
int devCountHeater{0};
int devCountRelay{0};
uint32_t eepromCRC{0};
uint32_t addressEEPROMcrc = {10};
uint32_t addressEEPROMSettings = {100};

unsigned long timeStartTConversion{0};

enum class State_t {run, setT, setMaxHeaterT, setDeltaHeaterT, error, man, 
                    setHeatingMode, setTargetDeltaT, setTemperatureStep,
                    saveSettings};
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
    running=false;
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
        lcd.print("now:");
        lcd.setCursor(8,0);
        lcd.print(currentT[0],displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        lcd.setCursor(0,1);
        lcd.print("-->");
        lcd.setCursor(8,1);
        lcd.print(param.targetT,displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setT:
        lcd.setCursor(0,0);
        lcd.print("Set temperature");
        lcd.setCursor(6,2);
        lcd.print(param.targetT,displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setMaxHeaterT:
        lcd.setCursor(0,0);
        param.heatingMode>0?lcd.print("Heater max T"):lcd.print("Cooler min T");
        lcd.setCursor(6,1);
        lcd.print(param.maxHeaterT,displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setDeltaHeaterT:
        lcd.setCursor(0,0);
        param.heatingMode>0?lcd.print("Heater delta T"):lcd.print("Cooler delta T");
        lcd.setCursor(6,1);
        lcd.print(param.deltaHeaterT,displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setHeatingMode:
        lcd.print("Set cool <> heat");
        lcd.setCursor(8,1);
        lcd.print(param.heatingMode<0?"cool":"heat");
        break;
      case State_t::setTargetDeltaT:
        lcd.print("Hysteresis");
        lcd.setCursor(6,1);
        lcd.print(param.targetTdelta,displayPrecision[param.whichStepT]);lcd.print(char(223));lcd.print("C");
        break;
      case State_t::setTemperatureStep:
        lcd.print("Temp resolution");
        lcd.setCursor(6, 1);
        lcd.print(stepT[param.whichStepT], displayPrecision[param.whichStepT]);
        break;
      case State_t::saveSettings:
        lcd.print("Save as defaults");
        lcd.setCursor(2,1); (saveSettings)?lcd.print("press to save"):lcd.print("no");
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

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
      //Serial.print("0");
    }
    //Serial.print(deviceAddress[i], HEX);
  }
  //Serial.println();
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
  thermo.begin();
  thermoHeater.begin();
  thermoRelay.begin();
  devCount = thermo.getDeviceCount();
  devCountHeater = thermoHeater.getDeviceCount();
  devCountRelay = thermoRelay.getDeviceCount();
  if (devCount==0) {ui.error("no sensor"); return;}
  if (devCountRelay==0) {ui.error("relay sensor"); return;}
  thermo.setResolution(thermoResolution[param.whichStepT]);
  thermoHeater.setResolution(9);
  thermoRelay.setResolution(9);
}

void setup()
{
  lcd.begin(16,2);

  //Serial.begin(57600);
  if (!RestoreSettings()) {
    lcd.clear();
    lcd.print("   Bad EEPROM");
    lcd.setCursor(0,1);
    lcd.print("fallback default");
  }

  pinMode(encoderButtonPin, INPUT_PULLUP);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);

  //enable interrupts on bank b
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4);

  initSensors();

  thermo.requestTemperatures();
  currentT[0] = param.mainAddr?thermo.getTempCByIndex(0):-127.;

  if (devCountHeater>0) {
    thermoHeater.requestTemperatures();
    heaterT[0] = thermoHeater.getTempCByIndex(0);
  }

  thermoRelay.requestTemperatures();
  relayT[0] = param.relayAddr?thermoRelay.getTempCByIndex(0):-127.;
  conversionDelay = 750 / (1 << (12-thermoResolution[param.whichStepT]));

  running = true;

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
        encoder.getDirection();
        ui.changeState(State_t::man);
        break;
      case State_t::setT:
        param.targetT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.whichStepT];
        break;
      case State_t::setMaxHeaterT:
        param.maxHeaterT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.whichStepT];
        if (param.maxHeaterT < param.targetT + param.deltaHeaterT) param.maxHeaterT = param.targetT + param.deltaHeaterT;
        break;
      case State_t::setDeltaHeaterT:
        param.deltaHeaterT += static_cast<int8_t>(encoder.getDirection()) * stepT[param.whichStepT];
        break;
      case State_t::setHeatingMode:
        param.heatingMode = static_cast<int8_t>(encoder.getDirection());
        break;
      case State_t::setTargetDeltaT:
        param.targetTdelta += static_cast<int8_t>(encoder.getDirection()) * stepT[param.whichStepT];
        param.targetTdelta = param.targetTdelta<0?0:param.targetTdelta;
        break;
      case State_t::setTemperatureStep:
        static int8_t dir{0};
        dir = static_cast<int8_t>(encoder.getDirection());
        if ((dir > 0) && (param.whichStepT == nResolutionsT-1)) {

        } else if ((dir < 0) && (param.whichStepT == 0)) {

        } else {
          param.whichStepT = param.whichStepT + dir;
        }
        conversionDelay = 750 / (1 << (12-thermoResolution[param.whichStepT]));
        thermo.setResolution(thermoResolution[param.whichStepT]);
        break;
      case State_t::saveSettings:
        static_cast<int8_t>(encoder.getDirection())==1 ? saveSettings=true : saveSettings=false ;
        break;
      default:
        break;
    }
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
        ui.changeState(State_t::setMaxHeaterT);
        break;
      case State_t::setMaxHeaterT:
        ui.changeState(State_t::setDeltaHeaterT);
        break;
      case State_t::setDeltaHeaterT:
        ui.changeState(State_t::setTemperatureStep);
        break;
      case State_t::setTemperatureStep:
        ui.changeState(State_t::setTargetDeltaT);
        break;
      case State_t::setTargetDeltaT:
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
        ui.changeState(State_t::setT);
        break;
      default:
        break;
    }
    }
  }

  // start conversion period
  if ((millis() - timeStartTConversion)>thermoSamplingPeriod) {
    timeStartTConversion = millis();
    noInterrupts();
    thermo.setWaitForConversion(false);
    thermo.requestTemperatures();

    if (devCountHeater) {
      thermoHeater.setWaitForConversion(false);
      thermoHeater.requestTemperatures();
    }

    thermoRelay.setWaitForConversion(false);
    thermoRelay.requestTemperatures();
    interrupts();
    readTonce = 1;
  }

  //read teperatures from sensors
  if (readTonce==1 && ((millis() - timeStartTConversion) > conversionDelay)) {
    currentT[1] = currentT[0];
    heaterT[1] = currentT[0];
    relayT[1] = relayT[0];
    noInterrupts();

    if (devCountHeater>0) {
      heaterT[0] = thermoHeater.getTempCByIndex(0);
    }

    currentT[0] = thermo.getTempCByIndex(0);
    relayT[0] = thermoRelay.getTempCByIndex(0);

    //Serial.print(currentT[0]); Serial.print(" "); Serial.print(heaterT[0]);Serial.print(" "); Serial.println(relayT[0]);

    interrupts();
    readTonce=0;
    ui.redraw = true;
  }

  //control the actual regulated temperature
  if (slopeT[0]>0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeT[0] *= (currentT[0] >= (param.targetT + param.targetTdelta)) ? -1 : 1;
  } else {
    //if we're on decline, we flip when crossing lower bound
    slopeT[0] *= (currentT[0] < (param.targetT - param.targetTdelta)) ? -1 : 1;
  }

  //control the heater temperature
  if (slopeH[0]>0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeH[0] *= (heaterT[0] >= param.maxHeaterT) ? -1 : 1;
  } else {
    //if we're on decline, we flip when crossing lower bound (or upper if we're cooling)
    slopeH[0] *= (heaterT[0] < (param.maxHeaterT - param.heatingMode*param.deltaHeaterT)) ? -1 : 1;
  }

  if ((slopeH[0] != slopeH[1]) || (slopeT[0] != slopeT[1]) || !running) {
    //Serial.print("slopeH: "); Serial.print(slopeH[0] * param.heatingMode);Serial.print(" slopeT: ");Serial.println(slopeT[0]*param.heatingMode);
    digitalWriteFast(relay1Pin, ((slopeT[0] * slopeH[0] * param.heatingMode > 0) && running) ? HIGH : LOW);
    digitalWriteFast(relay2Pin, ((slopeT[0] * slopeH[0] * param.heatingMode > 0) && running) ? HIGH : LOW);
    slopeH[1] = slopeH[0];
    slopeT[1] = slopeT[0];
  }

} // loop ()

// The End