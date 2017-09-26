//TODO software

//TODO hardware
//Consider selling units online

//Libraries
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <EEPROM.h>

//Pins
#define X_PIN A7
#define Y_PIN A6
#define Z_PIN A5
#define BTN_PIN 2
#define SPK_PIN 10

//Display
// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

//Button variables
int button_state;             // the current reading from the input pin
int long_button_state;
int last_button_state = LOW;   // the previous reading from the input pin
unsigned long last_debounce_time = 0;  // the last time the output pin was toggled
const unsigned long debounce_delay = 30;    // the debounce time; increase if the output flickers
const unsigned long debounce_delay_long = 1000;

//Accelerometer readings
int x, y, z; //raw readings
float lpf_x, lpf_y, lpf_z; //low pass filters
const float alpha_calibrate = 0.001;
const float alpha_measure = 0.04;
const float alpha_move = 0.2;
float alpha = alpha_measure; //lpf constant

//Calibration
float calibrated_lpf_x, calibrated_lpf_y, calibrated_lpf_z; //calibrated (zero centered) values
float calibration_x = 327.7; //temporary guess, to be calibrated
float calibration_y = 340; //temporary guess, to be calibrated
float calibration_z = 352.5; //temporary guess, to be calibrated
float deviation_x = 70; //maximum deviation from calibration, to be calibrated
float deviation_y = 70; //maximum deviation from calibration, to be calibrated
float deviation_z = 70; //maximum deviation from calibration, to be calibrated
float flat_x, flat_y, flat_z; //horizontal unit vector

//Physical paramters
const float person_walk_distance = 10;
const float person_shoulder_height = 1.7;

//Button variables
char mode = 'b';

//Current angle variables
float angle, angle_deg;
//char angle_str[10];
char angle_deg_str[10];

//Remembered angle variables
float base, top;
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

  //Set up button pin
  pinMode(BTN_PIN, INPUT_PULLUP);

  //Set up beeper pin
  pinMode(SPK_PIN, OUTPUT);

  //Set the backlight intensity
  analogWrite(9, 100);

  //Begin serial comms (debug only)
  Serial.begin(250000);

  //Get the initial lpf reading to partially correct initial transient
  lpf_x = analogRead(X_PIN);
  lpf_y = analogRead(Y_PIN);
  lpf_z = analogRead(Z_PIN);

  //Title screen
  display.begin();
  display.setContrast(55);
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 15);
  display.println("Tree Measurer");
  display.println("V1.4");
  display.display();

  delay(2000);

  display.clearDisplay();
  display.display();

  //Load calibration values from EEPROM
  loadCalibration();

  beep();
}

void loop() {

  //Read the button
  int button_reading = digitalRead(BTN_PIN);
  if (button_reading != last_button_state) {
    //reset debounce timer
    last_debounce_time = millis();
    //Serial.println("debounce");
  }
  if ((millis() - last_debounce_time) > debounce_delay) {
    //button has been oficially pressed/unpressed
    if (button_reading != button_state) {
      //Serial.println("changed");
      button_state = button_reading;
      if (button_state == LOW) {
        //button has been pressed
        //Serial.println("press!");
        //record the base or top angle
        getMeasurement();
        beep();
      }
    }
  }
  if ((millis() - last_debounce_time) > debounce_delay_long) {
    //button has been oficially pressed/unpressed for a long time
    if (button_reading != long_button_state) {
      //Serial.println("changed");
      long_button_state = button_reading;
      if (long_button_state == LOW) {
        //button has been long pressed
        //Serial.println("pressssssssssss!");
        //enter calibration
        beep();
        delay(100);
        beep();
        calibrateAccelerometer();
      }
    }
  }
  //Save the reading
  last_button_state = button_reading;

  //set alpha
  alpha = alpha_move;

  //Get the x and z readings (calibrated_lpf_x/z)
  getReadings();
  //  getReadings();
  //  getReadings();
  //  getReadings();
  //  getReadings();

  //Clip the x and z readings
  clipReadings();

  //Calculate angle
  calculateAngle();

  //Tree height algorithm, and string conversions
  calculateTreeHeight();

  //Display on screen
  writeDisplay();
}

void getReadings() {
  x = analogRead(X_PIN); //sensitive around +-90
  y = analogRead(Y_PIN);
  z = analogRead(Z_PIN); //sensitive around 0

  lpf_x = x * alpha + (1 - alpha) * lpf_x;
  lpf_y = y * alpha + (1 - alpha) * lpf_y;
  lpf_z = z * alpha + (1 - alpha) * lpf_z;

  calibrated_lpf_x = lpf_x - calibration_x;
  calibrated_lpf_y = lpf_y - calibration_y;
  calibrated_lpf_z = lpf_z - calibration_z;

  //  if (alpha != alpha_calibrate) {
  //    Serial.print(x);
  //    Serial.print(",");
  //    Serial.print(lpf_x);
  //    Serial.print(",");
  //    Serial.print(calibrated_lpf_x);
  //    Serial.print(",");
  //    Serial.print(y);
  //    Serial.print(",");
  //    Serial.print(lpf_y);
  //    Serial.print(",");
  //    Serial.print(calibrated_lpf_y);
  //    Serial.print(",");
  //    Serial.print(z);
  //    Serial.print(",");
  //    Serial.print(lpf_z);
  //    Serial.print(",");
  //    Serial.println(calibrated_lpf_z);
  //  }
}

void clipReadings() {
  //clip x
  if (calibrated_lpf_x > deviation_x) {
    calibrated_lpf_x = deviation_x;
  } else if (calibrated_lpf_x < -deviation_x) {
    calibrated_lpf_x = -deviation_x;
  }
  //clip y
  if (calibrated_lpf_y > deviation_y) {
    calibrated_lpf_y = deviation_y;
  } else if (calibrated_lpf_y < -deviation_y) {
    calibrated_lpf_y = -deviation_y;
  }
  //clip z
  if (calibrated_lpf_z > deviation_z) {
    calibrated_lpf_z = deviation_z;
  } else if (calibrated_lpf_z < -deviation_z) {
    calibrated_lpf_z = -deviation_z;
  }
}

void getMeasurement() {
  alpha = alpha_measure;

  for (int i = 0; i < 1000; i++) {
    getReadings();
  }

  clipReadings();

  calculateAngle();

  //  Serial.println(angle);

  if (mode == 'b') {
    mode = 't';
    base = angle;
  } else {
    mode = 'b';
    top = angle;
  }
}

void calculateAngle() {
  //normalise angles into ratios
  float normalised_x = calibrated_lpf_x / deviation_x;
  float normalised_y = calibrated_lpf_y / deviation_y;
  float normalised_z = calibrated_lpf_z / deviation_z;

  //calculate angle of device
  //angle = atan(normalised_z / normalised_x);
  float dot_x = flat_x * normalised_x;
  float dot_y = flat_y * normalised_y;
  float dot_z = flat_z * normalised_z;
  float dot = dot_x + dot_y + dot_z;

  float mag_flat = sqrt(flat_x * flat_x + flat_y * flat_y + flat_z * flat_z);
  float mag_normalised = sqrt(normalised_x * normalised_x + normalised_y * normalised_y + normalised_z * normalised_z);

  angle = -M_PI / 2 + acos(dot / (mag_flat * mag_normalised));
}

void calculateTreeHeight() {
  float omega = asin(person_shoulder_height * sin(M_PI / 2 + base) / person_walk_distance);
  float theta = M_PI / 2 - base - omega;
  float d = person_shoulder_height * sin(theta) / sin(omega);
  float t = d * sin(top - base) / sin(M_PI / 2 - top);

  //Set height, truncate it to reasonable values
  height = t;
  height = max(height, 0);
  height = min(height, 100);

  //calculate degree angle, create strings
  angle_deg = angle * 180 / M_PI;
  //dtostrf(angle, 3, 1, angle_str);
  dtostrf(angle_deg, 3, 0, angle_deg_str);

  //  Serial.print(angle);
  //  Serial.print(" ");
  //  Serial.print(angle_deg);
  //  Serial.print(" ");
  //  Serial.println(angle_deg_str);

  //Calculate more angles for strings
  base_deg = base * 180 / M_PI;
  top_deg = top * 180 / M_PI;
  dtostrf(base_deg, 3, 1, base_deg_str);
  dtostrf(top_deg, 3, 1, top_deg_str);
  dtostrf(height, 3, 1, height_str);
}

void writeDisplay() {
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
  display.println(char(0x11));

  //Master display
  display.display();
}

void calibrateAccelerometer() {

  alpha = alpha_calibrate; //just for calibration purposes

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Calibration 1 Hold horizontally, press trigger.");
  display.display();
  waitForButton();

  display.println("Wait...");
  display.display();

  //reset lpfs
  getReadings();
  lpf_x = x;
  lpf_y = y;
  lpf_z = z;
  //calibrate
  for (int i = 0; i < 10000; i++) {
    getReadings();
  }
  calibration_y = lpf_y;
  calibration_z = lpf_z;
  float maxiumum_x = lpf_x;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration 1 completed!");
  display.display();
  beep();
  delay(100);
  beep();
  delay(3000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration 2 Hold vertically, press trigger.");
  display.display();
  waitForButton();

  display.println("Wait...");
  display.display();

  //reset lpfs
  getReadings();
  lpf_x = x;
  lpf_y = y;
  lpf_z = z;
  //calibrate
  for (int i = 0; i < 10000; i++) {
    getReadings();
  }
  calibration_x = lpf_x;
  float maxiumum_z = lpf_z;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration 2 completed!");
  display.display();
  beep();
  delay(100);
  beep();
  delay(3000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Calibration 3 Hold on side, press trigger.");
  display.display();
  waitForButton();

  display.println("Wait...");
  display.display();

  //reset lpfs
  getReadings();
  lpf_x = x;
  lpf_y = y;
  lpf_z = z;
  //calibrate
  for (int i = 0; i < 10000; i++) {
    getReadings();
  }
  float maxiumum_y = lpf_y;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration 3 completed!");
  display.display();
  beep();
  delay(100);
  beep();
  delay(3000);

  deviation_x = abs(calibration_x - maxiumum_x);
  deviation_y = abs(calibration_y - maxiumum_y);
  deviation_z = abs(calibration_z - maxiumum_z);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Calibration 4 Hold vertically, press trigger.");
  display.display();
  waitForButton();

  display.println("Wait...");
  display.display();

  //reset lpfs
  getReadings();
  lpf_x = x;
  lpf_y = y;
  lpf_z = z;
  //calibrate
  for (int i = 0; i < 10000; i++) {
    getReadings();
  }
  clipReadings();
  flat_x = calibrated_lpf_x / deviation_x;;
  flat_y = calibrated_lpf_y / deviation_y;;
  flat_z = calibrated_lpf_z / deviation_z;;

  //  Serial.print(flat_x);
  //  Serial.print(" ");
  //  Serial.print(flat_y);
  //  Serial.print(" ");
  //  Serial.println(flat_z);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration 4 completed!");
  display.display();
  beep();
  delay(100);
  beep();
  delay(3000);

  saveCalibration();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Completed!");
  display.display();
  beep();
  delay(100);
  beep();
  delay(100);
  beep();
  delay(3000);

  //  Serial.print(calibration_z);
  //  Serial.print(" ");
  //  Serial.println(calibration_x);
  //  Serial.print(deviation_z);
  //  Serial.print(" ");
  //  Serial.println(deviation_x);
}

void beep() {
  analogWrite(SPK_PIN, 127);
  delay(50);
  analogWrite(SPK_PIN, 0);
}

void loadCalibration() {
  int address = 0;
  EEPROM.get(address, calibration_x);
  address += sizeof(float);
  EEPROM.get(address, calibration_y);
  address += sizeof(float);
  EEPROM.get(address, calibration_z);
  address += sizeof(float);
  EEPROM.get(address, deviation_x);
  address += sizeof(float);
  EEPROM.get(address, deviation_y);
  address += sizeof(float);
  EEPROM.get(address, deviation_z);
  address += sizeof(float);
  EEPROM.get(address, flat_x);
  address += sizeof(float);
  EEPROM.get(address, flat_y);
  address += sizeof(float);
  EEPROM.get(address, flat_z);
  address += sizeof(float);
}

void saveCalibration() {
  int address = 0;
  EEPROM.put(address, calibration_x);
  address += sizeof(float);
  EEPROM.put(address, calibration_y);
  address += sizeof(float);
  EEPROM.put(address, calibration_z);
  address += sizeof(float);
  EEPROM.put(address, deviation_x);
  address += sizeof(float);
  EEPROM.put(address, deviation_y);
  address += sizeof(float);
  EEPROM.put(address, deviation_z);
  address += sizeof(float);
  EEPROM.put(address, flat_x);
  address += sizeof(float);
  EEPROM.put(address, flat_y);
  address += sizeof(float);
  EEPROM.put(address, flat_z);
  address += sizeof(float);
}

void waitForButton() {
  while (1) {
    //Read the button
    int button_reading = digitalRead(BTN_PIN);
    if (button_reading != last_button_state) {
      //reset debounce timer
      last_debounce_time = millis();
      //Serial.println("debounce");
    }
    if ((millis() - last_debounce_time) > debounce_delay) {
      //button has been oficially pressed/unpressed
      if (button_reading != button_state) {
        //Serial.println("changed");
        button_state = button_reading;
        if (button_state == LOW) {
          //button has been pressed
          //Serial.println("press!");
          beep();
          return;
        }
      }
    }
    // save the reading
    last_button_state = button_reading;
  }
}
