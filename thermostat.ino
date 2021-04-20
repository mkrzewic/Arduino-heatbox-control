#include <Adafruit_SSD1306.h>
#include <RotaryEncoder.h>  //version 1.5.0 has a weird bug!
#include <digitalWriteFast.h>
#include <DallasTemperature.h>
#undef REQUIRESALARMS
#include <EEPROM.h>
#include <CRC32.h>

//#define DEBUG

//pin defs
constexpr uint8_t encoderPin1 = 11;
constexpr uint8_t encoderPin2 = 10;
constexpr uint8_t encoderButtonPin = 12;

constexpr uint8_t oneWirePinMain = A1;
constexpr uint8_t oneWirePinHeater = A2;
constexpr uint8_t oneWirePinRelay = A3;

constexpr uint8_t relaySSRpin = 13;
constexpr uint8_t relayClickPin = A0;

constexpr unsigned long ULONG_MAX{0xffffffff};
constexpr unsigned long thermoMainSamplingPeriod = 1000; //ms
constexpr unsigned long thermoHeaterSamplingPeriod = 500; //ms
constexpr unsigned long thermoRelaySamplingPeriod = 500; //ms
constexpr uint8_t iResolutionHeaterT = 2;
constexpr uint8_t iResolutionRelayT = 2;
constexpr unsigned int uiSpringBackDelay = 3500;

// OLED display stuff
constexpr uint8_t oled_screen_width = 128;
constexpr uint8_t oled_screen_height = 64;
constexpr uint8_t oled_reset_pin = 4;
constexpr uint8_t oled_address = 0x3C;
constexpr uint8_t largeTextSize = 3;
constexpr uint8_t smallTextSize = 1;

Adafruit_SSD1306 lcd(oled_screen_width, oled_screen_height, &Wire, 4);

//each sensor on a separate bus to avoid complications with 1-wire addressing
//when exchanging sensors
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

volatile unsigned long bounceTimer{0};
uint8_t modeButton[2] = {1};
int8_t knobPosition[2] = {0,0};

constexpr uint8_t nResolutionsT = 4;
constexpr uint8_t stepT[nResolutionsT] =                    {16, 32, 64, 128};
constexpr uint8_t tempSensorResolution[nResolutionsT] =     {11,    10,     9,  9};
constexpr uint8_t displayPrecision[nResolutionsT] =         {3,     2,      1,  0};

//settable parameters
struct Param_t {
  int16_t targetT{2560};
  int16_t limitHeaterT{8960};
  int16_t hysteresisT{128};
  int8_t heatingMode{1}; //-1 or 1 for cooling/heating
  uint8_t iStepT{2};
  int16_t maxRelayT{12672};
  int16_t maxTargetT{7680};
};

Param_t param{};
Param_t param_tmp;

int16_t heaterMaxT{0};
int16_t heaterMinT(0);
int16_t mainT{DEVICE_DISCONNECTED_RAW};
int16_t heaterT{DEVICE_DISCONNECTED_RAW};
int16_t relayT{DEVICE_DISCONNECTED_RAW};
int16_t cpuT{DEVICE_DISCONNECTED_RAW};
int8_t slopeT = {-1};
int8_t slopeH = {-1};
uint8_t errors = {0}; //start in the on state
int8_t waserrors = {1};
int8_t heaterIsOn{0}, heaterWasOn{1};
bool saveSettings{false};
uint8_t devCountMain{0};
uint8_t devCountHeater{0};
uint8_t devCountRelay{0};
uint32_t eepromCRC{0};
uint16_t nWeirdValuesRelayT{0};
uint16_t nWeirdValuesMainT{0};
uint16_t nWeirdValuesHeaterT{0};
uint16_t nRelayOverheatEvents{0};

//TODO: using size_t here does not work - why?
constexpr uint32_t addressEEPROMcrc = {0};
constexpr uint32_t addressEEPROMSettings = {addressEEPROMcrc + sizeof(addressEEPROMcrc)};

unsigned long timeStartMainTConversion{0};
unsigned long timeStartHeaterTConversion{0};
unsigned long timeStartRelayTConversion{0};
unsigned long timeStartMainTReadout{ULONG_MAX};
unsigned long timeStartHeaterTReadout{ULONG_MAX};
unsigned long timeStartRelayTReadout{ULONG_MAX};

#ifdef DEBUG
void DEBUGPRINT(const char* header = nullptr) {
  if (header) Serial.println(header);
  Serial.print(F("heatingMode: "));
  Serial.println(param.heatingMode);
  Serial.print(F("heaterIsOn: "));
  Serial.println(heaterIsOn);
  Serial.print(F("slopeT: "));
  Serial.println(slopeT);
  Serial.print(F("slopeH: "));
  Serial.println(slopeH);
  Serial.print(F("heaterMaxT: "));
  Serial.println(heaterMaxT);
  Serial.print(F("heaterMinT: "));
  Serial.println(heaterMinT);
  Serial.print(F("mainT: "));
  Serial.println(mainT);
  Serial.print(F("heaterT: "));
  Serial.println(heaterT);
  Serial.print(F("targetT: "));
  Serial.println(param.targetT);
}
#endif

void cpu_temp()
{
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  //enable ADC
  ADCSRA |= _BV(ADEN);  // enable the ADC
  // Start AD conversion
  ADCSRA |= _BV(ADSC);  // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  // return raw data
  cpuT = ADCW - 286;
  cpuT *= 105;
}

enum class State_t: uint8_t {overview, setTargetT, setLimitHeaterT, man,
  setHeatingMode, setTargetDeltaT, setTemperatureStep,
  saveSettings, showTemperatures,  setMaxTargetT, setLimitRelayT, idle };

enum class Error_t: int {badMainSensor, badHeaterSensor, badRelaySensor, relayOverheated, unexpectedDeltaT};
const char* errorString[] = {"main sensor", "heater sensor", "relay sensor", "relay overheated", "dT wrong sign"};

struct UI_t {
  unsigned long jumpBackNow{0}; 
  State_t state{State_t::idle};
  bool redraw{false};
  const char* errorMsg{nullptr};

  void changeState(State_t newState){
    state = newState;
    tick();
  }

  void tick() { redraw = true; jumpBackNow = millis() + uiSpringBackDelay; }

  void error(Error_t error) {
    BIT_SET(errors, static_cast<int>(error));
    errorMsg = errorString[static_cast<int>(error)];
    redraw = true;
  }
 
  void error_clear(Error_t error) {
    BIT_CLEAR(errors, static_cast<int>(error));
    redraw = true;
  }

  bool error_check(Error_t error) {
    return BIT_READ(errors, static_cast<int>(error));    
  }

  template<typename T>
    T intpow(T base, T exponent) {
      T ret{1};
      while (0 < exponent--) {
        ret *= base;
      }
      return ret;
    }

  //TODO: minimize chance of overflow, multiplying stuff by 1000 goes to large numbers potentially
  template<typename T, typename U, typename V, typename W, typename X>
    void printAsFloat(T const x, X const valPerUnit, U& out, W const places, V base) {
      out.print(x / valPerUnit, base);
      if (places <= 0) return;
      out.write('.');
      // TODO: here be dragons
      auto nb = out.print(abs(int32_t(x % valPerUnit)*intpow<int16_t>(base,places)/valPerUnit), base);
      for (; nb < places; ++nb) {
        out.print("0");
      }
    }

  template<typename U>
    void printDallasTempC(int32_t const raw, U& out, uint8_t const places) {
      if (raw == DEVICE_DISCONNECTED_RAW) {
        out.write('?');
      } else {
        printAsFloat(int32_t(raw), int32_t(128), out, places, DEC);
      }
      out.setTextSize(1);
      out.write(248);
      out.write('C');
    }

  void update() {
    if (!redraw) return;
    redraw = false;
    lcd.setTextColor(SSD1306_WHITE);
    lcd.setTextSize(smallTextSize);
    lcd.setCursor(0,0);
    switch(state) {
      case State_t::idle:
        break;
      case State_t::overview:
        lcd.clearDisplay();
        if (errors != 0) {
          lcd.setTextSize(largeTextSize);
          lcd.println(F("error:"));
          lcd.setTextSize(smallTextSize);
          lcd.println(errorMsg);
          lcd.setCursor(0,40);
          lcd.setTextSize(largeTextSize);
          lcd.setTextColor(SSD1306_WHITE);
          printDallasTempC(mainT, lcd, displayPrecision[param.iStepT]);
        } else {
          if (param.heatingMode>0) {
            lcd.println(F("Heat"));
          } else {
            lcd.println(F("Cool"));
          }
          lcd.print(F("to"));
          lcd.setCursor(30,0);
          lcd.setTextSize(2*smallTextSize);
          printDallasTempC(param.targetT, lcd, displayPrecision[param.iStepT]);
          lcd.println();
          lcd.setCursor(0,24);
          for (uint8_t i=0; i<21; ++i) { lcd.write(heaterIsOn > 0 ? 176 : 250); }
        }
        
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(mainT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setTargetT:
        lcd.clearDisplay();
        lcd.clearDisplay();
        lcd.println(F("Set temperature"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(param.targetT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setLimitHeaterT:
        lcd.clearDisplay();
        param.heatingMode>0 ? lcd.println(F("Max heater T")) : lcd.println(F("Min cooler T"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(param.limitHeaterT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setHeatingMode:
        lcd.clearDisplay();
        lcd.println(F("Set cool <> heat"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        lcd.print(param.heatingMode<0?F("cool"):F("heat"));
        break;
      case State_t::setTargetDeltaT:
        lcd.clearDisplay();
        lcd.println(F("T variation"));
        lcd.println(F("(hysteresis)"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(param.hysteresisT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setTemperatureStep:
        lcd.clearDisplay();
        lcd.println(F("T measurement"));
        lcd.println(F("resolution"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(stepT[param.iStepT], lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::showTemperatures:
        lcd.clearDisplay();
        lcd.print(F("main: "));
        printDallasTempC(mainT, lcd, displayPrecision[param.iStepT]);
        lcd.println();
        param.heatingMode > 0 ? lcd.print(F("heater: ")) : lcd.print(F("cooler: "));;
        printDallasTempC(heaterT, lcd, displayPrecision[param.iStepT]);
        lcd.println();
        lcd.print(F("relay: "));
        printDallasTempC(relayT, lcd, displayPrecision[param.iStepT]);
        lcd.println();
        lcd.print(F("cpu: "));
        printDallasTempC(cpuT, lcd, 0);
        lcd.println();
        lcd.print(F("SSR: "));
        lcd.print(digitalReadFast(relaySSRpin));
        lcd.print(F(" Click: "));
        lcd.println(!digitalReadFast(relayClickPin));
        lcd.print(F("error bits: "));
        lcd.println(errors,BIN);
        lcd.print(F("#err: "));
        lcd.print(nWeirdValuesMainT);
        lcd.write(' ');
        lcd.print(nWeirdValuesHeaterT);
        lcd.write(' ');
        lcd.println(nWeirdValuesRelayT);
        lcd.print(F("#overheat: "));
        lcd.print(nRelayOverheatEvents);
        break;
      case State_t::saveSettings:
        lcd.clearDisplay();
        lcd.println(F("Save as defaults"));
        lcd.println();
        lcd.setTextSize(largeTextSize);
        if (saveSettings) {
          lcd.println(F("press"));
          lcd.print(F("to save"));
        } else {
          lcd.print(F("no"));
        }
        break;
      case State_t::setLimitRelayT:
        lcd.clearDisplay();
        lcd.println(F("Max relay T"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(param.maxRelayT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::setMaxTargetT:
        lcd.clearDisplay();
        param.heatingMode>0 ? lcd.println(F("Max settable T")) : lcd.println(F("Min settable T"));
        lcd.setCursor(0,40);
        lcd.setTextSize(largeTextSize);
        printDallasTempC(param.maxTargetT, lcd, displayPrecision[param.iStepT]);
        break;
      case State_t::man:
        lcd.clearDisplay();
        lcd.setTextSize(2);
        lcd.println(F("Press to"));
        lcd.println(F("change"));
        lcd.print(F("settings")); 
        break;
    }
    lcd.display();
  }
};

UI_t ui{};

//handle interrupts on bank 0
ISR(PCINT0_vect) {
  encoder.tick(); // just call tick() to check the state.
  bounceTimer=millis()+50;
}

void SaveSettings() {
  saveSettings=false;
  EEPROM.put(addressEEPROMSettings, param);
  eepromCRC = CRC32::calculate(&param, 1); //second arg is number of elements of decltype(first param)
  EEPROM.put(addressEEPROMcrc, eepromCRC);
}

bool RestoreSettings() {
  EEPROM.get(addressEEPROMSettings, param_tmp);
  EEPROM.get(addressEEPROMcrc, eepromCRC);
  if (CRC32::calculate(&param_tmp,1) == eepromCRC) {
    param = param_tmp;
    return true;
  }
  return false;
}

void initSensors() {
  thermoMain.begin();
  thermoHeater.begin();
  thermoRelay.begin();
  thermoMain.setWaitForConversion(false);
  thermoHeater.setWaitForConversion(false);
  thermoRelay.setWaitForConversion(false);
  devCountMain = thermoMain.getDeviceCount();
  devCountHeater = thermoHeater.getDeviceCount();
  devCountRelay = thermoRelay.getDeviceCount();
  if (devCountMain==0) {ui.error(Error_t::badMainSensor);}
  if (devCountRelay==0) {ui.error(Error_t::badRelaySensor);}
  if (!thermoMain.getAddress(addrMain,0)) {ui.error(Error_t::badMainSensor);}
  if (!thermoHeater.getAddress(addrHeater,0) && devCountHeater>0) {ui.error(Error_t::badHeaterSensor);}
  if (!thermoRelay.getAddress(addrRelay,0)) {ui.error(Error_t::badRelaySensor);}
  thermoMain.setResolution(tempSensorResolution[param.iStepT]);
  thermoHeater.setResolution(tempSensorResolution[iResolutionHeaterT]);
  thermoRelay.setResolution(tempSensorResolution[iResolutionRelayT]);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  if(!lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
#ifdef DEBUG
      Serial.println(F("SSD1306 allocation failed"));
#endif
    for(;;); // Don't proceed, loop forever
  }
  
  lcd.setTextColor(SSD1306_WHITE);
  lcd.cp437(true);
  lcd.dim(true);
  //lcd.setRotation(0);
  
  if (!RestoreSettings()) {
    lcd.clearDisplay();
    lcd.println(F("  EEPROM error"));
    lcd.print(F("fallback default"));
    lcd.display();
    ui.jumpBackNow = millis()+1000;
    SaveSettings();  //maybe a new board or something went wrong, store defaults
  }

  lcd.clearDisplay();
  lcd.setCursor(36,28);
  lcd.print(F("MKr")); lcd.write(' '); lcd.print(F("2021"));
  lcd.display();
  ui.jumpBackNow = millis()+1000;

  pinMode(encoderButtonPin, INPUT_PULLUP);

  //enable interrupts on bank 0
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4);

  initSensors();

  digitalWriteFast(relayClickPin,HIGH);
  pinMode(relayClickPin, OUTPUT);
  pinMode(relaySSRpin, OUTPUT);

}

void loop()
{
#ifdef DEBUG
  static unsigned long tajm{0};
  tajm=micros();
#endif
  static unsigned long loopMillis{0};
  loopMillis = millis();

  ui.update();
  
  if ((ui.jumpBackNow != 0) && (loopMillis > ui.jumpBackNow)) {
    ui.changeState(State_t::overview);
    ui.jumpBackNow = 0;
  }

  //rotary encoder knob handling
  knobPosition[0] = encoder.getPosition();
  if (knobPosition[1]!=knobPosition[0]) {
    knobPosition[1]=knobPosition[0];
    ui.tick();
    auto dir = static_cast<int8_t>(encoder.getDirection());
    switch (ui.state) {
      case State_t::overview:
        if  (dir > 0) {
          ui.changeState(State_t::man);
        } else if ( dir < 0 ) {
          ui.changeState(State_t::showTemperatures);
          ui.jumpBackNow = loopMillis + 7000;
        }
        break;
      case State_t::setTargetT:
        param.targetT += dir * stepT[param.iStepT];
        if (param.heatingMode*param.targetT > param.heatingMode*param.maxTargetT) { param.targetT = param.maxTargetT; }
        break;
      case State_t::setLimitHeaterT:
        param.limitHeaterT += dir * stepT[iResolutionHeaterT];
        break;
      case State_t::setHeatingMode:
        param.heatingMode = dir;
        break;
      case State_t::setTargetDeltaT:
        param.hysteresisT += dir * stepT[param.iStepT];
        if (param.hysteresisT<stepT[param.iStepT]) { param.hysteresisT = stepT[param.iStepT]; }
        break;
      case State_t::setTemperatureStep:
        if ((dir > 0) && (param.iStepT == nResolutionsT-1)) {
        } else if ((dir < 0) && (param.iStepT == 0)) {
        } else {
          param.iStepT = param.iStepT + dir;
        }
        thermoMain.setResolution(tempSensorResolution[param.iStepT]);
        if (param.hysteresisT < stepT[param.iStepT]) { param.hysteresisT = stepT[param.iStepT]; }
        break;
      case State_t::saveSettings:
        dir == 1 ? saveSettings=true : saveSettings=false ;
        break;
      case State_t::setLimitRelayT:
        param.maxRelayT += dir * stepT[iResolutionRelayT];
        break;
      case State_t::setMaxTargetT:
        param.maxTargetT += dir * stepT[param.iStepT];
        break;
      case State_t::showTemperatures:
        ui.changeState(State_t::man);
        break;
      case State_t::man:
        
      default:
        break;
    }

#ifdef DEBUG
    DEBUGPRINT("knob handler");
#endif
  }

  //encoder button handling
  if (bounceTimer!=0 && (loopMillis > bounceTimer)) {
    bounceTimer = 0;
    modeButton[1] = modeButton[0];
    modeButton[0] = digitalReadFast(encoderButtonPin);
    if (modeButton[0] < modeButton[1]) {
      switch(ui.state) {
        case State_t::overview:
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
          if (saveSettings) {
            SaveSettings();
            lcd.clearDisplay();
            lcd.setTextSize(largeTextSize);
            lcd.setCursor(15, 20);
            lcd.print("saved");
            lcd.display();
            ui.changeState(State_t::idle);
            ui.jumpBackNow = loopMillis + 1000;
          } else {
            ui.changeState(State_t::overview);
          }
          break;
        case State_t::man:
          ui.changeState(State_t::setTargetT);
          break;
        case State_t::showTemperatures:
          ui.changeState(State_t::setTargetT);
          break;
        default:
          break;
      }
    }
  }

  //start conversion period for main
  if (loopMillis > timeStartMainTConversion) {
    thermoMain.requestTemperatures();
    timeStartMainTConversion = millis() + thermoMainSamplingPeriod + thermoMain.millisToWaitForConversion();
    timeStartMainTReadout = millis() + thermoMain.millisToWaitForConversion();
  }

  //read teperatures from main sensor
  if (loopMillis > timeStartMainTReadout) {
    timeStartMainTReadout = ULONG_MAX;
    mainT = thermoMain.getTemp(addrMain);
    
    if (mainT == DEVICE_DISCONNECTED_RAW) {
      ui.error(Error_t::badMainSensor);
    } else {
      ui.error_clear(Error_t::badMainSensor);
    }
    if (mainT >= 16000 || mainT <= -7040) { nWeirdValuesMainT++;}
    ui.redraw = true;

    cpu_temp();
  }

  //start conversion period for heater only if it is there
  if (devCountHeater > 0 &&  loopMillis > timeStartHeaterTConversion) {
    thermoHeater.requestTemperatures();
    timeStartHeaterTConversion = millis() + thermoHeaterSamplingPeriod;
    timeStartHeaterTReadout = millis() + thermoHeater.millisToWaitForConversion();
  }

  //read teperatures from heater sensor
  if (loopMillis > timeStartHeaterTReadout) {
    timeStartHeaterTReadout = ULONG_MAX;
    heaterT = thermoHeater.getTemp(addrHeater);
    
    if (heaterT == DEVICE_DISCONNECTED_RAW) {
      ui.error(Error_t::badHeaterSensor);
    } else {
      ui.error_clear(Error_t::badHeaterSensor);
    }
    if (heaterT >= 16000 || heaterT <= -7040) { nWeirdValuesHeaterT++;}    
  }

  //start conversion period for relay
  if (loopMillis > timeStartRelayTConversion) {
    thermoRelay.requestTemperatures();
    timeStartRelayTConversion = millis() + thermoRelaySamplingPeriod;
    timeStartRelayTReadout = millis() + thermoRelay.millisToWaitForConversion();
  }

  //read teperatures from relay sensor
  if (loopMillis > timeStartRelayTReadout) {
    timeStartRelayTReadout = ULONG_MAX;
    relayT = thermoRelay.getTemp(addrRelay);
    
    if (relayT == DEVICE_DISCONNECTED_RAW) {
      ui.error(Error_t::badRelaySensor);
    } else {
      ui.error_clear(Error_t::badRelaySensor);
    }
    
    if (!ui.error_check(Error_t::relayOverheated) && (relayT > param.maxRelayT)) { 
      ui.error(Error_t::relayOverheated);
      nRelayOverheatEvents++;
    }
    if (ui.error_check(Error_t::relayOverheated) && (relayT < (param.maxRelayT - param.hysteresisT))) {
      ui.error_clear(Error_t::relayOverheated);
    }

    if (relayT >= 16000 || relayT <= -7040) { nWeirdValuesRelayT++;}
  }
  
  //control the actual regulated temperature
  if (slopeT>0) {
    //if we're on the rise, we flip sign when we cross higher threshhold
    slopeT *= (mainT >= (param.targetT + param.hysteresisT)) ? -1 : 1;
  } else {
    //if we're on decline, we flip when crossing lower bound
    slopeT *= (mainT <= (param.targetT - param.hysteresisT)) ? -1 : 1;
  }

  //TODO this does not need to be recalculated every time
  static int16_t limitHeaterOther = (param.targetT + param.limitHeaterT)34/2;
  heaterMaxT = max(limitHeaterOther, param.limitHeaterT);
  heaterMinT = min(limitHeaterOther, param.limitHeaterT);

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

  //no need to access the hardware on every iteration, do it only when something changes
  if (((heaterIsOn != heaterWasOn) || (errors != waserrors)) && (errors == 0)) {

#ifdef DEBUG
    DEBUGPRINT("heater handler");
#endif

    digitalWriteFast(relaySSRpin, (heaterIsOn > 0) ? HIGH : LOW);
    heaterWasOn = heaterIsOn;
    ui.redraw = true;
  }

  //handle the connecting/disconnecting of the main relay based on the running condition
  //this essentially handles error conditions, so it is OK to use delay here
  if (errors != waserrors ) {
    waserrors = errors;
    ui.redraw = true;
    if (errors!=0) {
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
  static unsigned long acc{0};
  static unsigned long counter{0};
  acc += micros() - tajm;
  counter++;
  if (counter == 10000) {
    Serial.println(acc/counter);
    counter = 0;
    acc = 0;
  }
#endif
}
