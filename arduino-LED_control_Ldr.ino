#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);

const char* ssid = "OPPO Reno4 Pro";
const char* password = "tonkla998";

int GMTOffset = 25200;  //Replace with your GMT Offset in seconds
int daylightOffset = 0;  //Replace with your daylight savings offset in seconds

const int speek = 0;
const int LED = 2;
const int LDRInput = A0;

void setup() {
  
  Serial.begin(115200);
  pinMode(LDRInput, INPUT);
  pinMode(speek, OUTPUT);
  pinMode(LED, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }

  Serial.println("Connected to Wi-Fi!");

  configTime(GMTOffset, daylightOffset, "th.pool.ntp.org", "offsetTime");
}
void loop() {
  time_t rawtime = time(nullptr);
  struct tm* timeinfo = localtime(&rawtime);

  int radius = 35;
  display.drawCircle(display.width() / 2, display.height() / 2, 2, WHITE);
  //draw clock
  for ( int i = 0; i < 360; i = i + 30 ) {
    float angle = i ;
    angle = (angle / 57.29577951) ;
    int x1 = (64 + (sin(angle) * radius));
    int y1 = (32 - (cos(angle) * radius));
    int x2 = (64 + (sin(angle) * (radius - 5)));
    int y2 = (32 - (cos(angle) * (radius - 5)));
    display.drawLine(x1, y1, x2, y2, WHITE);
  }

  //draw second hand
  float angle = timeinfo->tm_sec * 6 ;
  angle = (angle / 57.29577951) ;
  int x2 = (64 + (sin(angle) * (radius)));
  int y2 = (32 - (cos(angle) * (radius)));
  display.drawLine(64, 32, x2, y2, WHITE);

  // draw minute hand
  angle = timeinfo->tm_min * 6 ;
  angle = (angle / 57.29577951) ;
  x2 = (64 + (sin(angle) * (radius - 3)));
  y2 = (32 - (cos(angle) * (radius - 3)));
  display.drawLine(64, 32, x2, y2, WHITE);

  // draw hour hand
  angle = timeinfo->tm_hour * 30 + int((timeinfo->tm_min / 12) * 6 );
  angle = (angle / 57.29577951) ;
  x2 = (64 + (sin(angle) * (radius - 11)));
  y2 = (32 - (cos(angle) * (radius - 11)));
  display.drawLine(64, 32, x2, y2, WHITE);

  display.display();
  delay(100);
  display.clearDisplay();

  ///////////////////////////////////////////////////////////////////////////////

  int value = analogRead(LDRInput);
  Serial.println("LDR value is :");
  Serial.println(value);
  
  if (value < 300){
    
    digitalWrite(LED, HIGH);
    digitalWrite(speek, HIGH);
    
  }else{
    
    digitalWrite(LED, LOW);
    digitalWrite(speek, LOW);
    
  }

}
