#include "U8glib.h"
#include "stdio.h"


//Force sensor - pins
const int sigPin = A7;

//Rotary encoder - logic
unsigned long last_run = 0;

//Ultrasonic - pins
const int echoPin = 7;
const int trigPin = 8;
//Ultrasonic - outputs
long duration_us, dist_mm = 0;

//OLED - pins
const int dcPin = 9;
const int csPin = 10;
const int mosiPin = 11;
const int sckPin = 13;
//OLED - setup
U8GLIB_SH1106_128X64 u8g(sckPin, mosiPin, csPin, dcPin);
// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9

//Calculations - parameters
//Spring
const float springConstant = 0.44874; //(N/mm)
const float springMaxLength = 12.06; //(mm)
const float springMinLength = 4.0; //(mm)
//Writing pressure
const float weak = 0.81;    //(mm) using 0.81N as model       [src = https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6110209/]
const float normal = 1.45;  //(mm) using 1.4-1.5N as model    [src = https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6110209/]
const float strong = 4.25;  //(mm) using 4.25-4.96N as model  [src = https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6110209/]
//Calculations - outputs
float force = 0.0;
String type = "";

void setup() {
  Serial.begin(9600);
  //Force sensor - pin setup
  //Not needed for analog read

  //Ultrasound - pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //OLED - settings
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
}

void loop() {
  force = analogRead(sigPin);
  // getForce(float(force));

  //Ultrasonic - trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //Ultrasonic - calculate travel time
  duration_us = pulseIn(echoPin, HIGH);
  //Ultrasonic - extract distance
  dist_mm = usTOmm(duration_us);

  //OLED - drawing to screen
  u8g.firstPage();  
  do {
    draw();
  }
  while( u8g.nextPage() );
  // rebuild the picture after some delay and update ultrasonic reading

  delay(50);
}


float usTOmm(float microseconds) {
  // The speed of sound is 340 m/s or 2.9 microseconds per millimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 2.9 / 2;
}


void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  //u8g.setFont(u8g_font_osb21);

  // char buffer[4];
  // sprintf(buffer, "%-4i", counter);

  String msg0 = "N/mm: ";
  String msg1 = "type: ";
  String msg2 = "page dist: ";

  // sprintf(buffer, "%-4i", force)

  u8g.drawStr( 0, 15, msg0.c_str());
  u8g.drawStr(u8g.getStrPixelWidth(msg0.c_str()), 15, String(force).c_str());
  u8g.drawStr( 0, 30, msg1.c_str());
  u8g.drawStr(u8g.getStrPixelWidth(msg1.c_str()), 30, String(type).c_str());
  u8g.drawStr( 0, 45, msg2.c_str());
  u8g.drawStr(u8g.getStrPixelWidth(msg2.c_str()), 45, String(dist_mm).c_str());
}


// float getForce(float reading) {
  
// }