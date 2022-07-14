//Library imports
#include <DHT.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <PMS.h>
#include <SoftwareSerial.h>
#include "TimerOne.h" // Timer Interrupt set to 2 second for read sensors
#include <math.h>

//Pin definition
#define anem 2
#define mulA 3
#define mulB 4
#define mulC 5
#define mulOut 6
#define rainG 7
#define setPMS 8
#define RxPMS 9
#define TxPMS 10

//DHT Parameters
#define DHTpin 11
#define DHTtype DHT11
DHT dht(DHTpin, DHTtype);

//RF Comms
RH_ASK driver;

//Wind Vane Multiplexer
#define number_of_mux 1
#define numOfMuxPins number_of_mux * 8

//Wind vane patterns
static struct {
  const char *windDirection;
  uint8_t pattern[2];
} windDirectionPattern [] = {
  //  single bit, two bit, three and four bit pattern
  //  to cope with different reed and magnet characteristics
  { "N",    { 0b00000001, 0b10000011 } },
  { "NNE",  { 0b00000011, 0b10000111 } },
  { "NE",   { 0b00000010, 0b00000111 } },
  { "ENE",  { 0b00000110, 0b00001111 } },
  { "E",    { 0b00000100, 0b00001110 } },
  { "ESE",  { 0b00001100, 0b00011110 } },
  { "SE",   { 0b00001000, 0b00011100 } },
  { "SSE",  { 0b00011000, 0b00111100 } },
  { "S",    { 0b00010000, 0b00111000 } },
  { "SSW",  { 0b00110000, 0b01111000 } },
  { "SW",   { 0b00100000, 0b01110000 } },
  { "WSW",  { 0b01100000, 0b11110000 } },
  { "W",    { 0b01000000, 0b11100000 } },
  { "WNW",  { 0b11000000, 0b11100001 } },
  { "NW",   { 0b10000000, 0b11000001 } },
  { "NNW",  { 0b10000001, 0b11000011 } }
};

//PMS5003 Sensor
SoftwareSerial Serial1(RxPMS, TxPMS); // RX, TX
PMS pms(Serial1);

//Anemometer
int LastValue; // last direction value
volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed
volatile unsigned int TimerCount; // used to determine 2.5sec timer count
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime;
int WindSpeed; // speed miles per hour


void setup() {
  //  Pin modes
  pinMode(rainG, INPUT_PULLUP);
  //  Serial
  Serial.begin(9600);
  //  DHT
  dht.begin();
  //  RF Comms
  if (!driver.init()) {
    Serial.println(F("RF Init Error"));
  }
  // Wind vane multiplexer
  pinMode(mulA, OUTPUT);
  pinMode(mulB, OUTPUT);
  pinMode(mulC, OUTPUT);
  pinMode(mulOut, INPUT);
  digitalWrite(mulA, LOW);
  digitalWrite(mulB, LOW);
  digitalWrite(mulC, LOW);

  //  Anemometer
  pinMode(anem, INPUT_PULLUP);
  LastValue = 0;
  IsSampleRequired = false;
  TimerCount = 0;
  Rotations = 0;

  attachInterrupt(digitalPinToInterrupt(anem), isr_rotation, RISING);

  Timer1.initialize(500000);// Timer interrupt every 2.5 seconds
  Timer1.attachInterrupt(isr_timer);

  //  PMS Serial
  pinMode(setPMS, OUTPUT);
  digitalWrite(setPMS, HIGH);
  Serial1.begin(9600);
}

void loop() {
  if (IsSampleRequired) {
    // convert to mp/h using the formula V=P(2.25/T)
    // V = P(2.25/2.5) = P * 0.9
    WindSpeed = (int)((Rotations * 0.9) * 1.60934);
    Rotations = 0; // Reset count for next sample

    IsSampleRequired = false;

    String iteration;

    iteration = readDHTData();
    iteration += ",";
    iteration += readPMSData();
    iteration += ",";

    if (WindSpeed < 10) {
      iteration += "00";
      iteration += (String)WindSpeed;
    }
    else if (WindSpeed >= 10 && WindSpeed < 100) {
      iteration += "0";
      iteration += (String)WindSpeed;
    }
    else {
      iteration += (String)WindSpeed;
    }
    
    iteration += ",";
    iteration += readWindVaneDir();


    Serial.println(iteration);
  }

}
/******************************************************************************/
String readPMSData() {
  PMS::DATA data;
  char infoPMS[11];

  // Clear buffer (removes potentially old data) before read. Some data could have been also sent before switching to passive mode.
  while (Serial.available()) {
    Serial.read();
  }
  pms.requestRead();

  if (pms.readUntil(data))
  {
    int pm1 = data.PM_AE_UG_1_0;
    int pm25 = data.PM_AE_UG_2_5;
    int pm10 = data.PM_AE_UG_10_0;

    sprintf(infoPMS, "%03d,%03d,%03d", pm1, pm25, pm10);
    return infoPMS;
  }
  else
  {
    return "PMSReadFail";
  }
}
/******************************************************************************/
String readWindVaneDir() {
  const char *result = NULL;
  uint8_t directions = 0;

//  Serial.print("active directions: ");

  for (int i = 0; i < 8; i++) {
    //  select one of 8 vane contacts
    digitalWrite(mulA, i & 0b00000001 ? HIGH : LOW);
    digitalWrite(mulB, i & 0b00000010 ? HIGH : LOW);
    digitalWrite(mulC, i & 0b00000100 ? HIGH : LOW);
    delay(10); // this fixes issues with wrong digitalRead() results below

    //  read and store in directions
    if (digitalRead(mulOut)) {
      directions = directions | (0b1 << i);
//      Serial.print(windDirectionPattern[i * 2].windDirection);
//      Serial.print(" ");
    }
  }

  //  Check for all pin pattern...
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 2; j++) {
      uint8_t pattern = windDirectionPattern[i].pattern[j];
      if (directions == pattern)
        result = windDirectionPattern[i].windDirection;
    }
  }
  
  return result;
}
/***************************************************************************/
String readDHTData() {
  char info[14];
  float t = dht.readTemperature();     // read celcius
  int h = dht.readHumidity();          // read humidity
  if (isnan(h) || isnan(t)) {          // Error detection
    return ("DhtReadFailXXX");
  }
  float hic = dht.computeHeatIndex(t, h);

  sprintf(info, "%02d.%02d,%02d,%02d.%02d", (int)t, (int)(fabsf(t) * 100) % 100, h, (int)hic, (int)(fabsf(hic) * 100) % 100); //Saves to info char array the variables
  return info;
}

/******************************************************************************/
void isr_timer() {
  TimerCount++;

  if (TimerCount == 6)
  {
    IsSampleRequired = true;
    TimerCount = 0;
  }
}

void isr_rotation() {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}
/******************************************************************************/
void sendRfData(String msg) {
  const char *msgPointer = msg.c_str();
  driver.send((uint8_t *)msgPointer, strlen(msgPointer));
  driver.waitPacketSent();
}
