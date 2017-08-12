//TODO software
//Update algo for measuring angle by using all 3 axis
//Consider using gyros for better measurement
//Update calibration procedure
//Fix debounce for button
//Add beping after data squisition
//Make sure data is stable before aquiring data
//Reset all low pass filters before taking a reading
//Increase the value of alpha for regular display purposes
//Consider rounding to nearest degree while moving freely

//TODO hardware
//Consider selling units online

//Libraries
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//Pins
#define X_PIN A7
#define Y_PIN A6
#define Z_PIN A5
#define INT_PIN 2

//Debug variables
char calibrate = 0;

//Display
// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

//Accelerometer readings
int x, y, z; //raw readings
float lpf; //low pass filter
float alpha = 0.04; //lpf constant

//Calibration for horizontal, vertical (up), and vertical (down)
float x_calibrate = 296.3;
float x_max = x_calibrate - 232.15;
float x_min = x_calibrate - 358.3;

//Physical paramters
float person_walk_distance = 10;
float person_shoulder_height = 1.7;

//Button variables
volatile char mode = 'b';
char mode_ready;

//Current angle variables
float angle, angle_deg;
char angle_str[10];
char angle_deg_str[10];

//Remembered angle variables
volatile float base, top;
float base_deg, top_deg;
char base_deg_str[10];
char top_deg_str[10];

//Output variables
float height;
char height_str[10];

void setup() {
  //Set up acceleromter pins
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);

  //Set up button pins
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), buttonPress, RISING);

  //Set the backlight intensity
  analogWrite(9, 100);

  //Begin serial comms (debug only)
  Serial.begin(9600);

  //Get the initial lpf reading to partially correct initial transient
  lpf = analogRead(X_PIN) - x_calibrate;

  //Title screen
  display.begin();
  display.setContrast(80);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 10);
  display.println("Tree Height");
  display.println("Measurer");
  display.println("V1.2");
  display.display();

  delay(2000);
  
  display.clearDisplay();
  display.display();
}

void loop() {

  //Calibration (factory/debug use)
  if (calibrate) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Calibrating...");
    display.display();

    for (int i = 0; i < 1000; i++) {
      x = analogRead(X_PIN);
      //  y = analogRead(Y_PIN);
      //  z = analogRead(Z_PIN);

      lpf = x * alpha + (1 - alpha) * lpf;
      delay(10);
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println(lpf);
    display.display();

    delay(5000);

    calibrate = 0; //calibrated now
  }

  x = analogRead(X_PIN) - x_calibrate;
  //  y = analogRead(Y_PIN);
  //  z = analogRead(Z_PIN);

  lpf = x * alpha + (1 - alpha) * lpf;

  //Coorect for calibration
  if (-lpf > 0) { //elevated angle
    angle = asin(min(-lpf, x_max) / x_max);
  } else { //depressed angle
    angle = -asin(max(-lpf, x_min) / x_min);
  }

  //Calculate angles, create strings
  angle_deg = 180 / M_PI * angle;
  dtostrf(angle, 3, 1, angle_str);
  dtostrf(angle_deg, 3, 1, angle_deg_str);

  //Superceded algo for tree height
  //float a = sqrt(person_shoulder_height * person_shoulder_height + person_walk_distance * person_walk_distance);
  //height = a * sin(top - base) / sin(M_PI / 2 - top);

  //Tree height algorithm
  float omega = asin(person_shoulder_height * sin(M_PI / 2 + base) / person_walk_distance);
  float theta = M_PI / 2 - base - omega;
  float d = person_shoulder_height * sin(theta) / sin(omega);
  float t = d * sin(top - base) / sin(M_PI / 2 - top);

  //Set height, truncate it to reasonable values
  height = t;
  height = max(height, 0);
  height = min(height, 100);

  //Calculate more angles for strings
  base_deg = base * 180 / M_PI;
  top_deg = top * 180 / M_PI;
  dtostrf(base_deg, 3, 1, base_deg_str);
  dtostrf(top_deg, 3, 1, top_deg_str);
  dtostrf(height, 3, 1, height_str);

  //Create strings for presentation on screen
  char line1[20];
  char line2[20];
  char line3[20];
  char line4[20];
  char deg = 247;

  //Write strings for presentation on screen
  sprintf(line1, "%s%c", angle_deg_str, deg);
  sprintf(line2, "Base: %s%c", base_deg_str, deg);
  sprintf(line3, "Top:  %s%c", top_deg_str, deg);
  sprintf(line4, "%sm", height_str);
  
  //Write strings to screen
  display.clearDisplay();

  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(line1);

  display.setCursor(0, 15);
  display.setTextSize(1);
  display.println(line2);

  display.setCursor(0, 24);
  display.setTextSize(1);
  display.println(line3);

  display.setCursor(0, 33);
  display.setTextSize(2);
  display.println(line4);

  //Include the cursor
  if (mode == 'b') {
    display.setCursor(75, 15);
  } else {
    display.setCursor(75, 24);
  }
  display.setTextSize(1);
  display.println("<");

  //Master display
  display.display();

  //Delay for sanity
  delay(10);

  //Debounce
  mode_ready = 1;
}

void buttonPress() {
  //Change mode, and record angle
  if (mode_ready) {
    if (mode == 'b') {
      mode = 't';
      base = angle;
    } else {
      mode = 'b';
      top = angle;
    }
  }
  //Debounce
  mode_ready = 0;
}

