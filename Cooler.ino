#define BLYNK_USE_DIRECT_CONNECT

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>

#ifndef BlynkSerialBLE_h
#define BlynkSerialBLE_h

#ifndef BLYNK_INFO_CONNECTION
#define BLYNK_INFO_CONNECTION "SerialBLE"
#endif

#define BLYNK_SEND_ATOMIC
#define BLYNK_SEND_CHUNK      20
#define BLYNK_SEND_THROTTLE   40
#define BLYNK_NO_INFO

#include <Adapters/BlynkSerial.h>

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_BLYNK)
  BlynkTransportStream _blynkTransport;
  BlynkStream Blynk(_blynkTransport);
#else
  extern BlynkStream Blynk;
#endif

#include <BlynkWidgets.h>

#endif

#ifndef _ADAFRUIT_SENSOR_H
#define _ADAFRUIT_SENSOR_H

#ifndef ARDUINO
#include <stdint.h>
#elif ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

/* Constants */
#define SENSORS_GRAVITY_EARTH (9.80665F) 
#define SENSORS_GRAVITY_MOON (1.6F)      
#define SENSORS_GRAVITY_SUN (275.0F)    
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)\
#define SENSORS_MAGFIELD_EARTH_MAX                                             
  (60.0F)
#define SENSORS_MAGFIELD_EARTH_MIN                                             \
  (30.0F) 
#define SENSORS_PRESSURE_SEALEVELHPA                                           \
  (1013.25F) 
#define SENSORS_DPS_TO_RADS                                                    \
  (0.017453293F)                           \
                  */
#define SENSORS_RADS_TO_DPS                                                    \
  (57.29577793F) 
#define SENSORS_GAUSS_TO_MICROTESLA                                            \
  (100) 

/** Sensor types */
typedef enum {
  SENSOR_TYPE_ACCELEROMETER = (1), /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD = (2),
  SENSOR_TYPE_ORIENTATION = (3),
  SENSOR_TYPE_GYROSCOPE = (4),
  SENSOR_TYPE_LIGHT = (5),
  SENSOR_TYPE_PRESSURE = (6),
  SENSOR_TYPE_PROXIMITY = (8),
  SENSOR_TYPE_GRAVITY = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION =
      (10), /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE = (13),
  SENSOR_TYPE_OBJECT_TEMPERATURE = (14),
  SENSOR_TYPE_VOLTAGE = (15),
  SENSOR_TYPE_CURRENT = (16),
  SENSOR_TYPE_COLOR = (17)
} sensors_type_t;

/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
  union {
    float v[3]; ///< 3D vector elements
    struct {
      float x; ///< X component of vector
      float y; ///< Y component of vector
      float z; ///< Z component of vector
    };         ///< Struct for holding XYZ component
    /* Orientation sensors */
    struct {
      float roll; 
      
      float pitch;   
                        
      float heading; 
      
    };             
  };                
                     
  int8_t status;   
  uint8_t reserved[3]; 
} sensors_vec_t;

/** struct sensors_color_s is used to return color data in a common format. */
typedef struct {
  union {
    float c[3]; 
    /* RGB color space */
    struct {
      float r;   
      float g;  
      float b;  
    };          
  };            
  uint32_t rgba; 
} sensors_color_t;

/* Sensor event (36 bytes) */
typedef struct {
  int32_t version;   
  int32_t sensor_id; 
  int32_t type;      
  int32_t reserved0; 
  int32_t timestamp; 
  union {
    float data[4];              
    sensors_vec_t acceleration; 
    
    sensors_vec_t
        magnetic; 
    sensors_vec_t orientation; 
    sensors_vec_t gyro;        
    float temperature; 
    float distance;    
    float light;       
    float pressure;   
    float relative_humidity; 
    float current;           
    float voltage;          
    sensors_color_t color;   
  };                         ///< Union for the wide ranges of data we can carry
} sensors_event_t;

typedef struct {
  char name[12];     
  int32_t version;   
  int32_t sensor_id; 
  int32_t type;     
  float max_value;  
  float min_value;   
  float resolution; 
  
  int32_t min_delay; 
  
} sensor_t;

class Adafruit_Sensor {
public:

  Adafruit_Sensor() {}
  virtual ~Adafruit_Sensor() {}
  
  virtual void enableAutoRange(bool enabled) {
    (void)enabled; 
  };
  
  virtual bool getEvent(sensors_event_t *) = 0;
  virtual void getSensor(sensor_t *) = 0;

  void printSensorDetails(void);

private:
  bool _autoRange;
};

#endif

/*!
   @file Adafruit_HMC5883_U.h
*/
#ifndef __HMC5883_H__
#define __HMC5883_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

#define HMC5883_ADDRESS_MAG (0x3C >> 1) 

typedef enum {
  HMC5883_REGISTER_MAG_CRA_REG_M = 0x00,
  HMC5883_REGISTER_MAG_CRB_REG_M = 0x01,
  HMC5883_REGISTER_MAG_MR_REG_M = 0x02,
  HMC5883_REGISTER_MAG_OUT_X_H_M = 0x03,
  HMC5883_REGISTER_MAG_OUT_X_L_M = 0x04,
  HMC5883_REGISTER_MAG_OUT_Z_H_M = 0x05,
  HMC5883_REGISTER_MAG_OUT_Z_L_M = 0x06,
  HMC5883_REGISTER_MAG_OUT_Y_H_M = 0x07,
  HMC5883_REGISTER_MAG_OUT_Y_L_M = 0x08,
  HMC5883_REGISTER_MAG_SR_REG_Mg = 0x09,
  HMC5883_REGISTER_MAG_IRA_REG_M = 0x0A,
  HMC5883_REGISTER_MAG_IRB_REG_M = 0x0B,
  HMC5883_REGISTER_MAG_IRC_REG_M = 0x0C,
  HMC5883_REGISTER_MAG_TEMP_OUT_H_M = 0x31,
  HMC5883_REGISTER_MAG_TEMP_OUT_L_M = 0x32
} hmc5883MagRegisters_t;

typedef enum {
  HMC5883_MAGGAIN_1_3 = 0x20, 
  HMC5883_MAGGAIN_1_9 = 0x40, 
  HMC5883_MAGGAIN_2_5 = 0x60, 
  HMC5883_MAGGAIN_4_0 = 0x80, 
  HMC5883_MAGGAIN_4_7 = 0xA0, 
  HMC5883_MAGGAIN_5_6 = 0xC0, 
  HMC5883_MAGGAIN_8_1 = 0xE0  
} hmc5883MagGain;

typedef struct hmc5883MagData_s {
  float x;          
  float y;           
  float z;          
  float orientation; 
} hmc5883MagData;

#define HMC5883_ID (0b11010100)

//! Unified sensor driver for the magnetometer ///
class Adafruit_HMC5883_Unified : public Adafruit_Sensor {
  public:
  
    Adafruit_HMC5883_Unified(int32_t sensorID = -1);

    bool begin(void); //!< @return Returns whether connection was successful
    void setMagGain(hmc5883MagGain gain);
    bool
    getEvent(sensors_event_t *);
    void getSensor(sensor_t *);

  private:
    hmc5883MagGain _magGain;
    hmc5883MagData _magData;
    int32_t _sensorID;

    void write8(byte address, byte reg, byte value);
    byte read8(byte address, byte reg);
    void read(void);
};

#endif

#ifndef TinyGPS_h
#define TinyGPS_h

#include "Arduino.h"

#define _GPS_VERSION 10
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001

class TinyGPS
{
  public:
    TinyGPS();
    bool encode(char c);
    TinyGPS &operator << (char c) {
      encode(c);
      return *this;
    }

    // lat/long in hundred thousandths of a degree and age of fix in milliseconds
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ?
                                GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
    {
      if (date) *date = _date;
      if (time) *time = _time;
      if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ?
                                GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    inline long altitude() {
      return _altitude;
    }

    inline unsigned long course() {
      return _course;
    }

    unsigned long speed() {
      return _speed;
    }

#ifndef _GPS_NO_STATS
    void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

    inline void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
    {
      long lat, lon;
      get_position(&lat, &lon, fix_age);
      *latitude = lat / 100000.0;
      *longitude = lon / 100000.0;
    }

    inline void crack_datetime(int *year, byte *month, byte *day,
                               byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0)
    {
      unsigned long date, time;
      get_datetime(&date, &time, fix_age);
      if (year)
      {
        *year = date % 100;
        *year += *year > 80 ? 1900 : 2000;
      }
      if (month) *month = (date / 100) % 100;
      if (day) *day = date / 10000;
      if (hour) *hour = time / 1000000;
      if (minute) *minute = (time / 10000) % 100;
      if (second) *second = (time / 100) % 100;
      if (hundredths) *hundredths = time % 100;
    }

    inline float f_altitude()    {
      return altitude() / 100.0;
    }
    inline float f_course()      {
      return course() / 100.0;
    }
    inline float f_speed_knots() {
      return speed() / 100.0;
    }
    inline float f_speed_mph()   {
      return _GPS_MPH_PER_KNOT * f_speed_knots();
    }
    inline float f_speed_mps()   {
      return _GPS_MPS_PER_KNOT * f_speed_knots();
    }
    inline float f_speed_kmph()  {
      return _GPS_KMPH_PER_KNOT * f_speed_knots();
    }

    static int library_version() {
      return _GPS_VERSION;
    }

    enum {GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 999999999, GPS_INVALID_ALTITUDE = 999999999, GPS_INVALID_DATE = 0,
          GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = 999999999, GPS_INVALID_FIX_TIME = 0xFFFFFFFF
         };


    static float distance_between (float lat1, float long1, float lat2, float long2);

  private:
    enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER};

    // properties
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;

    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;

#ifndef _GPS_NO_STATS
    // statistics
    unsigned long _encoded_characters;
    unsigned short _good_sentences;
    unsigned short _failed_checksum;
    unsigned short _passed_checksum;
#endif

    // internal utilities
    int from_hex(char a);
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    bool term_complete();
    bool gpsisdigit(char c) {
      return c >= '0' && c <= '9';
    }
    long gpsatol(const char *str);
    int gpsstrcmp(const char *str1, const char *str2);
};

#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif

char auth[] = "blynk-token";

#define SERVO_PIN 3

#define GPS_TX_PIN 6

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

#define MOTOR_A_EN_PIN 5
#define MOTOR_B_EN_PIN 9
#define MOTOR_A_IN_1_PIN 7
#define MOTOR_A_IN_2_PIN 8
#define MOTOR_B_IN_1_PIN 12
#define MOTOR_B_IN_2_PIN 4

#define MOTOR_A_OFFSET 20
#define MOTOR_B_OFFSET 0

#define DECLINATION_ANGLE 0.23f

#define COMPASS_OFFSET 0.0f

#define GPS_UPDATE_INTERVAL 1000

#define GPS_STREAM_TIMEOUT 18

#define GPS_WAYPOINT_TIMEOUT 45

#define SERVO_LID_OPEN 20
#define SERVO_LID_CLOSE 165

struct GeoLoc {
  float lat;
  float lon;
};

enum CoolerLid {
  OPENED,
  CLOSED
};

TinyGPS gps;

Servo lidServo;
CoolerLid lidState = CLOSED;

// Master Enable
bool enabled = false;

//WidgetTerminal terminal(V3);

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);            // TXD to digital pin 6

/* Compass */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;

  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;

  gps.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;

  Serial.print(coolerLoc.lat, 7); Serial.print(", "); Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

// Lid Hook
BLYNK_WRITE(V0) {
  switch (lidState) {
    case OPENED:
      setServo(SERVO_LID_CLOSE);
      lidState = CLOSED;
      break;
    case CLOSED:
      setServo(SERVO_LID_OPEN);
      lidState = OPENED;
      break;
  }
}

// Killswitch Hook
BLYNK_WRITE(V1) {
  enabled = !enabled;

  //Stop the wheels
  stop();
}

// GPS Streaming Hook
BLYNK_WRITE(V2) {
  GpsParam gps(param);

  Serial.println("Received remote GPS: ");

  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook
BLYNK_WRITE(V3) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;

  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');

    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex + 1);

      if (colonIndex != -1) {
        lonStr = rawInput.substring(commaIndex + 1, colonIndex);
      }

      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();

      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;

        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }

    rawInput = rawInput.substring(colonIndex + 1);

  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon - a.lon) * cos(b.lat);
  float x = cos(a.lat) * sin(b.lat) - sin(a.lat) * cos(b.lat) * cos(b.lon - a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat - a.lat) * DEGTORAD;
  float dl = (b.lon - a.lon) * DEGTORAD;

  float x = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  float y = 2 * atan2(sqrt(x), sqrt(1 - x));

  return R * y;
}

float geoHeading() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Offset
  heading -= DECLINATION_ANGLE;
  heading -= COMPASS_OFFSET;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

void setServo(int pos) {
  lidServo.attach(SERVO_PIN);
  lidServo.write(pos);
  delay(2000);
  lidServo.detach();
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);

  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);

  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void drive(int distance, float turn) {
  int fullSpeed = 230;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }

  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;

  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);

  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0) {
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);

  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
    float d = 0;
    //Start move loop here
    do {
      nss.listen();
      coolerLoc = checkGPS();
      bluetoothSerial.listen();

      d = geoDistance(coolerLoc, loc);
      float t = geoBearing(coolerLoc, loc) - geoHeading();

      Serial.print("Distance: ");
      Serial.println(geoDistance(coolerLoc, loc));

      Serial.print("Bearing: ");
      Serial.println(geoBearing(coolerLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());

      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout > 0);

    stop();
  }
}

void setupCompass() {
  /* Initialise the compass */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displayCompassDetails();
}

void setup()
{
  // Compass
  setupCompass();

  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(4800);

  nss.begin(9600);

  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
}

// Testing
void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);

  while (!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }

  stop();
}

void loop()
{
  Blynk.run();
}
