#include <AS5600.h>
#include <Wire.h>
#include <AbsMouse.h>

#define FLASH_DEBUG 0
#include <FlashStorage_SAMD.h>


#define DEBUG_LOGGING 1

#ifdef DEBUG_LOGGING
  #define debugPrint(...) DEBUGLOGGING Serial.print(...)

#else
  #define debugPrint(...)
#endif

FlashStorage(flash_storage, StorageVars_t);

typedef struct
{
  /* CARTESIAN PLANE WITH ORIGIN AT SHOULDER JOINT
   *                 +-----------+
   *      X          |           |
   *shoulder joint   |           |
   *    (0,0)        |           | y_size
   *                 |           |
   *   (x_origin, -> +-----------+
   *    y_origin)        x_size
   */
  int x_origin;
  int y_origin;
  int x_size;
  int y_size;
} TabletArea_t;

typedef struct
{
  int A_0;
  int A_90;
  int B_0;
  int B_90;
  uint64_t first_write_timestamp;
  uint64_t last_write_timestamp;
  TabletArea_t tabletArea;
} StorageVars_t;

StorageVars_t storage_vars;

int POTA_PIN = 1;
int POTB_PIN = 2;
int ACTIVATE_PIN = 8;
int SELECT_PIN = 3;
int G_PIN = 11;
int R_PIN = 10;
int B_PIN = 9;

float rValue = 0;
float gValue = 0;
float bValue = 0;

float valueA = 0;
float valueB = 0;

float prevValueA = -1;
float prevValueB = -1;
float prevPrevValueA = -1;
float prevPrevValueB = -1;

float expFilterWeight = 1;
float expFilterWeightXY = 1;

float angleA = 0;
float angleB = 0;

int rawAngleA = 0;
int rawAngleB = 0;

float l1 = 60;
float l2 = 60;
float x_raw = 0;
float y_raw = 0;

float x_rawPrev = 0;
float y_rawPrev = 0;

int A_0 = 3954;
int A_90 = 3954 - (4096 / 4);
int B_90 = 3872 - (4096 / 4);
int B_180 = 3872;

int pollRate_hz = 550;
int period_uS = (1000 * 1000) / pollRate_hz;
unsigned long lastRunTime_uS = 0;
unsigned long currentTime_uS = 0;

int screenWidth = 1920;
int screenHeight = 1080;

float x_origin = 40;
float x_width = 67.7;
float x_max = x_origin + x_width;
float y_height = screenHeight * 1.0 / screenWidth * x_width;
float y_origin = y_height / 2;
float y_max = y_origin - y_height;

int ADCSamples = 9;

unsigned long lastButtonTime = 0;

volatile int activateMouse = 1;

void setup()
{
  // put your setup code here, to run once:
  pinMode(POTA_PIN, INPUT);
  pinMode(POTB_PIN, INPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(R_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(ACTIVATE_PIN, INPUT_PULLUP);
  pinMode(SELECT_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ACTIVATE_PIN), toggleMouseActivate, FALLING);
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(800000UL);

  analogReadResolution(12);
  analogReference(AR_DEFAULT);

  AbsMouse.init(screenWidth, screenHeight);

  digitalWrite(SELECT_PIN, HIGH);
  delayMicroseconds(50);
  //        0123456789ABCDEF
  as5600Setup();

  prevPrevValueA = readTwoBytesFast();
  prevValueA = prevPrevValueA;
  valueA = prevValueA;

  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(50);
  //        0123456789ABCDEF
  as5600Setup();

  prevPrevValueB = readTwoBytesFast();
  prevValueB = prevPrevValueB;
  valueB = prevValueB;
}

void loop()
{

  currentTime_uS = micros();

  if (currentTime_uS - lastRunTime_uS > period_uS)
  {

    lastRunTime_uS = micros();

    //valueA = analogSample(POTA_PIN,ADCSamples);
    //valueB = analogSample(POTB_PIN,ADCSamples);

    prevPrevValueA = prevValueA;
    prevPrevValueB = prevValueB;

    prevValueA = valueA;
    prevValueB = valueB;

    // READ ANGLE A
    digitalWrite(SELECT_PIN, HIGH);
    delayMicroseconds(10);
    valueA = readTwoBytesFast();

    int valueA_median = median_of_3(valueA, prevPrevValueB, prevValueA);

    // READ ANGLE B
    digitalWrite(SELECT_PIN, LOW);
    delayMicroseconds(10);
    valueB = readTwoBytesFast();
    int valueB_median = median_of_3(valueB, prevPrevValueB, prevValueB);

    //Serial.println(valueA_median);

    //Serial.println(String(valueA,DEC) + "," + String(valueB,DEC) + "," + String(valueA_median,DEC) + "," String(valueB_median,DEC));

    // Sensor ADC counts to angle in degrees
    angleA = mapf(valueA, A_0, A_90, 0, 90);
    angleB = mapf(valueB, B_90, B_180, 90, 180);

    float angleB2 = -(180 - angleA - angleB);

    x_raw = l1 * cos(angleA * PI / 180) + l2 * cos(angleB2 * PI / 180);
    y_raw = l1 * sin(angleA * PI / 180) + l2 * sin(angleB2 * PI / 180);

    x_raw = expFilter(x_raw, x_rawPrev, expFilterWeightXY);
    y_raw = expFilter(y_raw, y_rawPrev, expFilterWeightXY);

    x_rawPrev = x_raw;
    y_rawPrev = y_raw;

    float x_mapped = constrain(mapf(x_raw, x_origin, x_max, 0.0, screenWidth), 0, screenWidth);
    float y_mapped = constrain(mapf(y_raw, y_origin, y_max, 0.0, screenHeight), 0, screenHeight);

    //Serial.print(" A raw ADC: ");Serial.print(valueA);
    //Serial.print(" B raw ADC: ");Serial.print(valueB);

    //Serial.print(valueA); Serial.print(" "); Serial.println(valueB);

    //Serial.print(" A angle: ");Serial.print(angleA);
    //Serial.print(" B angle: ");Serial.print(angleB);

    //Serial.print(" x: ");Serial.print(x_raw);
    //Serial.print(" y: ");Serial.print(y_raw);

    //Serial.print(" x mapped: ");Serial.println(x_mapped);
    //Serial.print(" y mapped: ");Serial.println(y_mapped);

    //Serial.println("");

    if (activateMouse == 1)
    {
      AbsMouse.move(round(x_mapped), round(y_mapped));
      //AbsoluteMouse.moveTo(round(x_mapped), round(y_mapped));
      //AbsMouse.move(500, 500);
    }

    float t_const = 0.001;
    float a_const = 255;
    unsigned long current_time = millis();

    analogWrite(R_PIN, sin(millis() * t_const) * a_const / 2 + a_const / 2);
    analogWrite(G_PIN, sin(millis() * t_const + (2 * PI / 3)) * a_const / 2 + a_const / 2);
    analogWrite(B_PIN, sin(millis() * t_const + (4 * PI / 3)) * a_const / 2 + a_const / 2);
  }
}

void toggleMouseActivate()
{
  noInterrupts();
  if (activateMouse == 0)
  {
    activateMouse = 1;
  }
  else
  {
    activateMouse = 0;
  }
  interrupts();
}

float analogSample(int pin, int samples)
{

  int sum = 0;

  int sample[samples];

  for (int i = 0; i < samples; i++)
  {
    sample[i] = analogRead(pin);
  }

  qsort(sample, samples, sizeof(sample[0]), sort_desc);

  for (int i = 0; i < samples; i++)
  {
    //Serial.print(sample[i]); Serial.print(" ");
  }
  //Serial.println("");

  int startIndex = 1;
  int endIndex = samples - 1;

  for (int i = startIndex; i < endIndex; i++)
  {
    sum += sample[i];
  }

  return ((sum * 1.0) / (endIndex - startIndex));
}

float expFilter(float current, float prev, float weight)
{
  return prev * (1 - weight) + current * (weight);
}

float medianFilter(int A1, int A2, int A3)
{
  int sampleList[] = {A1, A2, A3};
  qsort(sampleList, 3, sizeof(sampleList[0]), sort_desc);
  return sampleList[1];
}

int median_of_3(int a, int b, int c)
{
  int the_max = max(max(a, b), c);
  int the_min = min(min(a, b), c);
  // unnecessarily clever code
  int the_median = the_max ^ the_min ^ a ^ b ^ c;
  return (the_median);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
/*
word getRawAngle2()
{
  int _raw_ang_hi = 0x0c;
  int _raw_ang_lo = 0x0d;
  return readTwoBytes2(_raw_ang_hi, _raw_ang_lo);
}

word readTwoBytes2(int in_adr_hi, int in_adr_lo)
{
  int _ams5600_Address = 0x36;
  
  word retVal = -1;

  //Read Low Byte
  myWire.beginTransmission(_ams5600_Address);
  myWire.write(in_adr_lo);
  myWire.endTransmission();
  myWire.requestFrom(_ams5600_Address, 1);
  while (myWire.available() == 0)
    ;
  int low = myWire.read();

  //Read High Byte
  myWire.beginTransmission(_ams5600_Address);
  myWire.write(in_adr_hi);
  myWire.endTransmission();
  myWire.requestFrom(_ams5600_Address, 1);

  while (myWire.available() == 0)
    ;

  word high = myWire.read();

  high = high << 8;
  retVal = high | low;

  return retVal;
}*/

void as5600Setup()
{
  //        0123456789ABCDEF
  // 8,9 are filter settings
  setConf(0b0000000010000000);

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("AS5600 ERROR");
  }
}

void setConf(word _conf)
{
  uint8_t err = writeOneByte(0x07, highByte(_conf));
  delay(2);
  writeOneByte(0x08, lowByte(_conf));
  delay(2);
}

/*******************************************************
  Method: getRawAngle
  In: none
  Out: value of raw angle register
  Description: gets raw value of magnet position.
  start, end, and max angle settings do not apply
*******************************************************/
word getRawAngleFast()
{
  return readTwoBytesFast();
}

/*******************************************************
  Method: writeOneByte
  In: address and data to write
  Out: none
  Description: writes one byte to a i2c register
*******************************************************/
uint8_t writeOneByte(int adr_in, int dat_in)
{
  Wire.beginTransmission(0x36);
  Wire.write(adr_in);
  Wire.write(dat_in);
  uint8_t err = Wire.endTransmission();
  return err;
}

/*******************************************************
  Method: readTwoBytes
  In: two registers to read
  Out: data read from i2c as a word
  Description: reads two bytes register from i2c
*******************************************************/
word readTwoBytesFast()
{
  word retVal = -1;

  // Automatic pointer reload for rawAngle
  //register, don't need to write reg addr
  //Wire.beginTransmission(0x36);
  //Wire.write(0x0c);
  //Wire.endTransmission(false);

  Wire.requestFrom(0x36, 2);
  while (Wire.available() < 2)
    ;

  word high = Wire.read();
  int low = Wire.read();

  high = high << 8;
  retVal = high | low;

  return retVal;
}

bool loadFlashStorage(StorageVars_t *storage_vars)
{
  if (!EEPROM.isValid())
  {
    // Flash has not been written to yet.
  }
  storage_vars->A_0;
}
