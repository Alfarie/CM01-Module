#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include<SoftwareSerial.h>
#include <EEPROM.h>
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define VREF 5000

#define StartConvert 0
#define ReadTemperature 1

#define EC_FEED_RELAY 3
#define PH_FEED_RELAY 5
#define compensationFactorAddress 8

#define UP 11
#define DOWN 12
#define SET 13

#define MAIN 0
#define SETTING 1
#define SETTING2 2
#define EC_CALIBRATION 3
#define PH_CALIBRATION 4
int index = 0;

const int  ph_pin = 0;
const int ec_pin = 1;
const int wt_pin = 10;
const byte numReadings = 30;
long ph_feed_millis , ec_feed_millis;
bool detect_ph = false, detect_ec = false;

SoftwareSerial ms(8, 9);

OneWire ds(wt_pin);
LiquidCrystal_I2C lcd(0x27, 20, 4);

int display_state = 0;

// sensor variable
float ph_meter, ec_meter, wt_meter , raw_ec;
float compensationFactor;
unsigned long dtInterval = 0, dtSample;
unsigned long AnalogValueTotal = 0;                  // the running total
unsigned int AnalogAverage = 0, averageVoltage = 0;
int readings[numReadings];
unsigned long AnalogSampleTime, printTime, tempSampleTime;
unsigned int AnalogSampleInterval = 25, printInterval = 1000, tempSampleInterval = 850;


int DetectingTime = 120; // 180 sec
unsigned int ec_feed_time = 2; // 3sec
unsigned int ph_feed_time = 4; // 3sec

float ph_feed_data , ec_feed_data , dt_data;
int ec_feed_addr = 20 , ph_feed_addr = 24 , dt_addr = 28 , ecth_addr = 32, phth_addr = 36;
float ec_th = 1.6;
float ph_th = 6.0;


#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the sample voltage
int analogBufferIndex = 0;

String str = "";

float slopeValue = 3.5, interceptValue = 0, averagepHVoltage;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ms.begin(9600);
  
  readCharacteristicValues();
  readSetting();
  lcd.begin();
  lcd.setCursor(0, 0);
  lcd.print("INITIALIZING..");
  TempProcess(StartConvert);   //let the DS18B20 start the convert
  initTimer();
  for (byte thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  pinMode(EC_FEED_RELAY , OUTPUT);
  pinMode(PH_FEED_RELAY , OUTPUT);

  digitalWrite(EC_FEED_RELAY, HIGH);
  digitalWrite(PH_FEED_RELAY, HIGH);

  dtInterval = 1000 * dt_data;
  ec_feed_time = ec_feed_data * 1000;
  ph_feed_time = ph_feed_data * 1000;
  lcd.clear();

}

void initButton() {
  pinMode(UP , INPUT);
  pinMode(DOWN , INPUT);
  pinMode(SET , INPUT);
}

void initTimer() {

  AnalogSampleTime = millis();
  printTime = millis();
  tempSampleTime = millis();
  dtSample = millis();
}
bool isUp() {
  return (digitalRead(UP) == LOW) ? true : false;
}

bool isDown() {
  return (digitalRead(DOWN) == LOW) ? true : false;
}

bool isSet() {
  return (digitalRead(SET) == LOW) ? true : false;
}


void loop() {
  if (isUp()) {
    delay(300);
    lcd.clear();
    Serial.println("UP");
    display_state++;
    display_state = (display_state <= PH_CALIBRATION) ? display_state : 0;
  }
  if (isDown()) {
    delay(300);
    lcd.clear();
    Serial.println("Down");
    display_state--;
    display_state = (display_state >= 0) ? display_state : PH_CALIBRATION;
  }
  if (isSet()) {
    delay(300);
    lcd.clear();
    Serial.println("Set");
  }
  if (display_state == MAIN) {
    main_display();
  }
  else if (display_state == SETTING) {
    setting_display();
  }

  else if (display_state == SETTING2) {
    setting2_display();
  }
  else if (display_state == EC_CALIBRATION) {
    ec_cal_display();
  }
  else if (display_state == PH_CALIBRATION) {
    ph_cal_display();
  }

  if (millis() - AnalogSampleTime >= AnalogSampleInterval)
  {
    AnalogSampleTime = millis();
    // subtract the last reading:
    AnalogValueTotal = AnalogValueTotal - readings[index];
    // read from the sensor:
    readings[index] = analogRead(ec_pin);
    // add the reading to the total:
    AnalogValueTotal = AnalogValueTotal + readings[index];
    // advance to the next position in the array:
    index = index + 1;
    // if we're at the end of the array...
    if (index >= numReadings)
      // ...wrap around to the beginning:
      index = 0;
    // calculate the average:
    AnalogAverage = getMedianNum(readings , numReadings);
  }

  if (millis() - tempSampleTime >= tempSampleInterval)
  {
    tempSampleTime = millis();
    wt_meter = TempProcess(ReadTemperature);  // read the current temperature from the  DS18B20
    TempProcess(StartConvert);                   //after the reading,start the convert for next reading
  }

  if (millis() - printTime >= printInterval)
  {
    ph_meter = averagepHVoltage / 1000.0 * slopeValue + interceptValue;
    printTime = millis();
    averageVoltage = AnalogAverage * (float)5000 / 1024;

    float TempCoefficient = 1.0 + 0.0185 * (wt_meter - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge = (float)averageVoltage / TempCoefficient;
    if (CoefficientVolatge <= 448) ec_meter = 6.84 * CoefficientVolatge - 64.32; //1ms/cm<EC<=3ms/cm
    else if (CoefficientVolatge <= 1457)ec_meter = 6.98 * CoefficientVolatge - 127; //3ms/cm<EC<=10ms/cm
    else ec_meter = 5.3 * CoefficientVolatge + 2278;                     //10ms/cm<EC<20ms/cm
    raw_ec = ec_meter / 1000; //convert us/cm to ms/cm

    ec_meter = ec_meter / compensationFactor / 1000.0;
    ec_meter = (ec_meter >=0)?ec_meter:0;
    Serial.println("PH: " + String(ph_meter) + " EC: " + String(ec_meter) + "ms/cm" + " Sol Temp: " + String(wt_meter) + "^C" );
    String str = "{cm01" + String(ph_meter) + "," + String(ec_meter) + "," + String(wt_meter) + "}";
    ms.println(str);

  }
  getPh2();
  if ( millis() - dtSample >= dtInterval ) {
    if (display_state == MAIN || display_state == SETTING) {
      Serial.println("Detecting Time!!");
      if ( ph_meter >= ph_th) {
        Serial.println("[start] ph");
        digitalWrite(PH_FEED_RELAY , LOW);
        ph_feed_millis = millis();
        detect_ph = true;
      }
      if ( ec_meter <= ec_th ) {
        Serial.println("[start] ec");
        digitalWrite(EC_FEED_RELAY , LOW);
        ec_feed_millis = millis();
        detect_ec = true;
      }
      initTimer();
    }
    else {
      initTimer();
    }
  }

  if (detect_ph) {
    if (millis() - ph_feed_millis >= ph_feed_time) {
      Serial.println("[stop] ph");
      digitalWrite(PH_FEED_RELAY, HIGH);
      detect_ph = false;
    }
  }
  if (detect_ec) {
    if (millis() - ec_feed_millis >= ec_feed_time) {
      Serial.println("[stop] ec");
      digitalWrite(EC_FEED_RELAY, HIGH);
      detect_ec = false;
    }
  }


  while (ms.available()) {
    char ch = ms.read();
    str += ch;
    if (str.endsWith("}")) {
      Serial.println(str);
      str.replace("{" , "");
      str.replace("}" , "");
      int i = 0 , si = 0 , ei;
      float data[5] = { 0, 0, 0, 0, 0};
      int j = 0;
      while (j < 5) {
        int index = str.indexOf(",");
        String a =  str.substring(0 , index);
        data[j] = a.toFloat();
        si = index;
        Serial.println("val : " + String(data[j]));
        str = str.substring(index + 1);
        j++;
      }

      ph_th = data[0];
      ph_feed_data = data[1];
      ec_th = data[2];
      ec_feed_data = data[3];
      dt_data = data[4];

      EEPROM_write(phth_addr , ph_th);
      EEPROM_write(ecth_addr , ec_th);
      EEPROM_write(ec_feed_addr , ec_feed_data);
      EEPROM_write(ph_feed_addr , ph_feed_data);
      EEPROM_write(dt_addr, dt_data);
      dtInterval = 1000 * dt_data;
      ec_feed_time = ec_feed_data * 1000;
      ph_feed_time = ph_feed_data * 1000;
      str = "";
    }
  }
}


float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if (!ch) {
    if ( !ds.search(addr)) {
      Serial.println("no more sensors on chain, reset search!");
      ds.reset_search();
      return 0;
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0;
    }
    if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized!");
      return 0;
    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end
  }
  else {
    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad
    for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
    }
    ds.reset_search();
    byte MSB = data[1];
    byte LSB = data[0];
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    TemperatureSum = tempRead / 16;
  }
  TemperatureSum = (TemperatureSum >= 0) ? TemperatureSum : 0;
  return TemperatureSum;
}

void main_display() {
  lcd.setCursor(0, 0); lcd.print("Sensor display");
  lcd.setCursor(0, 1); lcd.print("EC: " + String(ec_meter) + " ms/cm");
  lcd.setCursor(0, 2); lcd.print("pH: " + String(ph_meter)) ;
  lcd.setCursor(0, 3); lcd.print("Temp: " + String(wt_meter) + " C");
}

void setting_display() {
  lcd.setCursor(0, 0); lcd.print("Setting time");
  lcd.setCursor(0, 1); lcd.print("EC feed : " + String(ec_feed_data) + " s");
  lcd.setCursor(0, 2); lcd.print("pH feed : " + String(ph_feed_data) + " s") ;
  lcd.setCursor(0, 3); lcd.print("Detecting : " + String(dt_data) + " s");
}
void setting2_display() {
  lcd.setCursor(0, 0); lcd.print("Set point");
  lcd.setCursor(0, 1); lcd.print("EC: " + String(ec_th) + " ms/cm");
  lcd.setCursor(0, 2); lcd.print("pH: " + String(ph_th)) ;
}
void ec_cal_display() {
  lcd.setCursor(0, 0); lcd.print("EC CALIBRATION");
  lcd.setCursor(0, 1); lcd.print("raw EC : " + String(ec_meter));
  lcd.setCursor(0, 2); lcd.print("Cal : 12.88");
  if (isSet()) {
    delay(300);
    lcd.clear();
    if (Calibration(raw_ec)) {
      lcd.setCursor(5, 2); lcd.print("Successful");
      display_state = MAIN;

    }
    else {
      lcd.setCursor(5, 2); lcd.print("Fail");

    }
    delay(1000);
  }
}
void ph_cal_display() {
  lcd.setCursor(0, 0); lcd.print("pH CALIBRATION");
  lcd.setCursor(0, 1); lcd.print("pH : " + String(ph_meter));
  lcd.setCursor(0, 2); lcd.print("Cal : 4.0");
  if (isSet()) {

  }
}

void readCharacteristicValues()
{
  EEPROM_read(compensationFactorAddress, compensationFactor);
  if (EEPROM.read(compensationFactorAddress) == 0xFF && EEPROM.read(compensationFactorAddress + 1) == 0xFF && EEPROM.read(compensationFactorAddress + 2) == 0xFF && EEPROM.read(compensationFactorAddress + 3) == 0xFF)
  {
    compensationFactor = 1.0;   // If the EEPROM is new, the compensationFactorAddress is 1.0(default).
    EEPROM_write(compensationFactorAddress, compensationFactor);
  }
}

void readSetting() {
  EEPROM_read(phth_addr, ph_th);
  if (EEPROM.read(phth_addr) == 0xFF && EEPROM.read(phth_addr + 1) == 0xFF && EEPROM.read(phth_addr + 2) == 0xFF && EEPROM.read(phth_addr + 3) == 0xFF)
  {
    ph_th = 6.0;
    EEPROM_write(phth_addr, ph_th);
  }

  EEPROM_read(ecth_addr, ec_th);
  if (EEPROM.read(ecth_addr) == 0xFF && EEPROM.read(ecth_addr + 1) == 0xFF && EEPROM.read(ecth_addr + 2) == 0xFF && EEPROM.read(ecth_addr + 3) == 0xFF)
  {
    ec_th = 6.0;
    EEPROM_write(ecth_addr, ec_th);
  }

  EEPROM_read(ec_feed_addr, ec_feed_data);
  if (EEPROM.read(ec_feed_addr) == 0xFF && EEPROM.read(ec_feed_addr + 1) == 0xFF && EEPROM.read(ec_feed_addr + 2) == 0xFF && EEPROM.read(ec_feed_addr + 3) == 0xFF)
  {
    ec_feed_data = 3.0;
    EEPROM_write(ec_feed_addr, ec_feed_data);
  }

  EEPROM_read(ph_feed_addr, ph_feed_data);
  if (EEPROM.read(ph_feed_addr) == 0xFF && EEPROM.read(ph_feed_addr + 1) == 0xFF && EEPROM.read(ph_feed_addr + 2) == 0xFF && EEPROM.read(ph_feed_addr + 3) == 0xFF)
  {
    ph_feed_data = 6.0;
    EEPROM_write(ph_feed_addr, ph_feed_data);
  }

  EEPROM_read(dt_addr, dt_data);
  if (EEPROM.read(dt_addr) == 0xFF && EEPROM.read(dt_addr + 1) == 0xFF && EEPROM.read(dt_addr + 2) == 0xFF && EEPROM.read(dt_addr + 3) == 0xFF)
  {
    dt_data = 6.0;
    EEPROM_write(dt_addr, dt_data);
  }
}

bool Calibration(float raw_ec) {
  float factorTemp;
  factorTemp = raw_ec / 12.88;
  if ((factorTemp > 0.85) && (factorTemp < 1.15))
  {
    Serial.println();
    Serial.println(F(">>>Confrim Successful<<<"));
    Serial.println();
    compensationFactor =  factorTemp;
    EEPROM_write(compensationFactorAddress, compensationFactor);
    Serial.print(F(">>>Calibration Successful"));
    return true;

  }
  else {
    Serial.println();
    Serial.println(F(">>>Confirm Failed,Try Again<<<"));
    Serial.println();
    return false;
  }
}

void getPh2() {
  static unsigned long sampleTimepoint = millis();
  if (millis() - sampleTimepoint > 40U)
  {
    sampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(ph_pin) / 1024.0 * VREF; //read the voltage and store into the buffer,every 40ms
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
    averagepHVoltage = getMedianNum(analogBuffer, SCOUNT);  // read the stable value by the median filtering algorithm
    //Serial.println(averagepHVoltage);
  }
}

unsigned int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
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



