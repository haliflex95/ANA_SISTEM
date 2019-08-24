#include <SPI.h>
#include <RH_RF95.h>


RH_RF95 rf95;

//------------------------

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 6;   // tx i 4 e rx i 3 e takıyosun gpsin
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;   // The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//------------------------

const int voltageSensorPin = A3;          // sensor pin
float vIn;                                // gemeten voltage (3.3V = max. 16.5V, 5V = max 25V)
float vOut;
float voltageSensorVal;                   // waarde op pin A3 (0 - 1023)
const float factor = 5.095;               // reductie factor van het Voltage Sensor shield
const float vCC = 5.00;                   // Arduino invoer voltage (na te meten met voltmeter)

//------------------------


#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

//------------------------


int switch1 = 7;
int switch2 = 8;
int fpvrole = 9;






void setup()
{
            //gps sistem
  Serial.begin(9600);
  ss.begin(GPSBaud); 

  
            // rf sistem
  while (!Serial) ; 
  if (!rf95.init())
    Serial.println("init failed"); //rf sistem

Serial.println("Sistemler Hazırlanıyor 5sn Bekleyin.");


            //g kuvveti sensörü
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


              
           //switcler ve röle
pinMode(switch1, INPUT);
pinMode(switch2, INPUT);
pinMode(fpvrole, INPUT);


}
      
int ZAxGForce;
void loop()
{
          //  switcler ve röle
  int switch1durum = digitalRead(switch1);
  int switch2durum = digitalRead(switch2);
  int fpvroledurum = digitalRead(fpvrole);



           //g kuvveti sensörü
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        
       
      
        ZAxGForce = az / 2048;
        
       
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
 

  voltageSensorVal = analogRead(voltageSensorPin);    
  vOut = (voltageSensorVal / 1024) * vCC;             
  vIn =  vOut * factor;                               
  
  
  printFloat(gps.location.lng(),gps.location.isValid(), 8, 5);
  Serial.print("*");
  printFloat(gps.location.lat(),gps.location.isValid(), 8, 5);
  Serial.print("*");
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 4, 2);
  Serial.print("*");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 3, 0);
  Serial.print("*");
  printInt(gps.satellites.value(), gps.satellites.isValid(),2);
  Serial.print("*");
  Serial.print(vIn);
  Serial.print("*");
  Serial.print(ZAxGForce);
  Serial.print("*");
  Serial.print(switch1durum);
  Serial.print("*");
  Serial.print(switch2durum);
  Serial.print("*");
  Serial.print(fpvroledurum);
  Serial.print("*");
  
  
 uint8_t data[] = "dfsdvsd" ;
 rf95.send(data, sizeof(data));
  
 
  
 
  

  
  
  
  delay(400);
//  smartDelay(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
