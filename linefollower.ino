#include <MotorDriver.h>

#include <QTRSensors.h>
#include <Wire.h> //I2C Arduino Library
#include "compass.h"
#include "MotorDriver.h"
#include <SoftwareSerial.h>
struct linestat {
  double err;
  double adj_speed;
};
struct linestat stats[100];
int stat_count = 0;

#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#define R_TRIG_PIN 52
#define R_ECHO_PIN 53
#define C_TRIG_PIN 50
#define C_ECHO_PIN 51
#define L_TRIG_PIN 48
#define L_ECHO_PIN 49

#define NUM_SENSORS   8    // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2
#define LEFT_ENCODER 18
#define RIGHT_ENCODER 19
// Arduino Mega
#define digitalPinToInterrupt(p)  ( (p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : -1)) )


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc_left((unsigned char[]) {
  22, 23, 24, 25, 26, 27, 28, 29
}, NUM_SENSORS, TIMEOUT, 255);
QTRSensorsRC qtrrc_right((unsigned char[]) {
  40, 41, 42, 43, 44, 45, 46, 47
}, NUM_SENSORS, TIMEOUT, 255);
unsigned int sensorValues[NUM_SENSORS * 2];


int curr_left_speed = 0;
int curr_right_speed = 0;



int MIN_MAX = 300;
int STOP = 0;
int SLOW = 75;
int CRUISE = 130;
int EMITTER = 2;
int REAL_SLOW = 32;
int INCR = 15;
int x, y, z; //triple axis data

char bufferX [20];



int toggle = 0;
float Kp = 0.07;
float Kd = 0;
int MAX_ADJ_SPEED = 255;
int curr_mode = 0; //1-line, 2-remote control, 3-calibrate, 4-menu request
int dozer_target = -1;


int error_found = 0;
int prev_error_found = 0;
int base_speed = 75;
int last_skew = 0;
int found_max = 0;
int straight_speed = 0;
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;

struct MaxIdx {
  unsigned int idx;
  unsigned int reading;
};

////////////////Normalizing of Sensors
unsigned int find_min(unsigned int readings[], int num_readings) {
  int i = 0;
  unsigned int min_value = 10000;
  for (i = 0; i < num_readings; i++) {
    if (readings[i] < min_value) {
      min_value = readings[i];
    }
  }
  return min_value;
}
unsigned int find_max(unsigned int readings[], int num_readings) {
  int i = 0;
  unsigned int max_value = 0;
  for (i = 0; i < num_readings; i++) {
    if (readings[i] > max_value) {
      max_value = readings[i];
    }
  }
  return max_value;
}
struct MaxIdx find_max_idx(unsigned int readings[], int num_readings) {
  int i = 0;
  struct MaxIdx max_idx;
  unsigned int max_value = 0;
  for (i = 0; i < num_readings; i++) {
    if (readings[i] > max_value) {
      max_value = readings[i];
      max_idx.reading = readings[i];
      max_idx.idx = i;
    }
  }
  return max_idx;
}
void min_adjust(unsigned int *readings, int num_readings) {
  int i = 0;
  unsigned int result[num_readings];
  unsigned int min_value = 10000;
  min_value = find_min(readings, num_readings);
  for (i = 0; i < num_readings; i++) {
    readings[i] = readings[i] - min_value;
  }
}

void normalize(unsigned int readings[], int num_readings) {
  int i = 0;
  unsigned int max_value;
  unsigned int min_value;
  unsigned int diff;
  unsigned long tmp;
  unsigned long sum = 0;

  min_value = find_min(readings, num_readings);
  max_value = find_max(readings, num_readings);
  diff = max_value - min_value;
  min_adjust(readings, num_readings);
  for (i = 0; i < num_readings; i++) {
    sum += readings[i];
  }
  for (i = 0; i < num_readings; i++) {
    tmp = (readings[i] );
    tmp = tmp * 1000;
    tmp = tmp / sum;
    readings[i] = (unsigned int)tmp;
  }
}

int get_error(unsigned int readings[], int num_readings) {
  struct MaxIdx found_max = find_max_idx(readings, num_readings);
  if (found_max.reading < 300) Serial3.println("No Max found");
  int result = (found_max.idx * 1000) - 8000;
  return(result);
  
}

int get_error_orig(unsigned int readings[], int num_readings) {
  int i = 0;
  int skew = 0;
  int mul_idx;

  found_max = find_max(readings, num_readings);

  normalize(readings, num_readings);

  for (i = num_readings / 2; i < num_readings; i++) {
    mul_idx = i - num_readings / 2 + 1;
    skew += readings[i] * mul_idx;
  }
  for (i = 0; i < num_readings / 2; i++) {
    skew -= readings[i] * ( num_readings / 2  - i );

  }
  //for(i=0;i<num_readings;i++) {
  //  Serial3.print(readings[i]);
  //  Serial3.print(",");
  //}
  if (toggle == 1) {
    Serial3.print("<");
    Serial3.print(skew);
    Serial3.print(">");
  }

  return (skew);
}


/////////////////////MOTOR CONTROLS
void stop(void) //Stop
{
  curr_left_speed = 0;
  curr_right_speed = 0;
  motordriver.stop();
}
void  backup(char left_speed, char right_speed) //Move forward
{
  motordriver.setSpeed(left_speed,MOTORB);
  motordriver.setSpeed(right_speed,MOTORA);
  motordriver.goBackward();
}
void advance(char left_speed, char right_speed) //Move forward
{
    Serial3.print("SPEED>");
    Serial3.print(left_speed);
    Serial3.print(",");
    Serial3.println(right_speed);
  if (left_speed < 0) {
    turnL(left_speed * -1, right_speed);
  }
  else if (right_speed < 0) {
    turnR(left_speed , right_speed * -1);
  }
  else {
    forward(left_speed, right_speed);
  }
}

void forward(unsigned char left_speed, unsigned char right_speed) //Move forward
{
  motordriver.setSpeed(right_speed,MOTORB);
  motordriver.setSpeed(left_speed,MOTORA);
  motordriver.goForward();
}
void turnR(unsigned char left_speed, unsigned char right_speed) //Move forward
{
  motordriver.setSpeed(right_speed,MOTORB);
  motordriver.setSpeed(left_speed,MOTORA);
  motordriver.goRight();
}
void turnL(unsigned char left_speed, unsigned char right_speed) //Move forward
{
  motordriver.setSpeed(right_speed,MOTORB);
  motordriver.setSpeed(left_speed,MOTORA);
  motordriver.goLeft();
}



void autocalibrate() {
  long start_time = millis();
  compass_heading();
  float start = bearing;
  float adjustment = bearing - 180;
  Serial3.println("Autocalibrate");
  char output[200]="";
  digitalWrite(EMITTER, HIGH);
  motordriver.setSpeed(75,MOTORB);
  motordriver.setSpeed(75,MOTORA);
  float adj_bearing = bearing - adjustment;
  while(adj_bearing >90 && ((millis() - start_time) < 2000L)) {
    compass_heading();
      qtrrc_left.calibrate();
  qtrrc_right.calibrate();
    adj_bearing = (int)(bearing - adjustment) %360;
    motordriver.goLeft();
    //sprintf(output,"Bearing %f %f", bearing, adj_bearing);
   
  }
  motordriver.stop();
  while (adj_bearing < 270 && ((millis() - start_time) < 3000L)) {
    compass_heading();
    qtrrc_left.calibrate();
    qtrrc_right.calibrate();
     adj_bearing = (int)(bearing - adjustment) %360;
     motordriver.goRight();
    
  
  }
  motordriver.stop();
  while (adj_bearing > 180 && ((millis() - start_time) < 4000L)) {
    compass_heading();
    qtrrc_left.calibrate();
    qtrrc_right.calibrate();
    adj_bearing = (int)(bearing - adjustment) %360;
    motordriver.goLeft();

  
  }
  digitalWrite(EMITTER, HIGH);
  motordriver.stop();
}
//Multi-Functions
void calibrate()
{
  int i;


  Serial3.print("Calibrating");
  pinMode(EMITTER, OUTPUT);
  digitalWrite(EMITTER, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  qtrrc_left.calibrate();
  qtrrc_right.calibrate();

  stop();
  digitalWrite(EMITTER, LOW);
  Serial3.print("Calibration done");

}

void compass_setup() {

  x = 0;
  y = 0;
  z = 0;

  Wire.begin();
  compass_x_offset = -55.74;
  compass_y_offset = 122.97;
  compass_z_offset = 351.43;
  compass_x_gainError = 1.09;
  compass_y_gainError = 1.03;
  compass_z_gainError = 0.98;
  compass_init(2);

}
void compass_loop(void) {
  //Tell the HMC5883 where to begin reading data
  compass_heading();

}


void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("STARTING");
  Serial3.println("STARTING2");
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), left_encoder_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), right_encoder_update, CHANGE);
  compass_setup();
  pinMode(L_TRIG_PIN, OUTPUT);
  pinMode(L_ECHO_PIN, INPUT);
  pinMode(C_TRIG_PIN, OUTPUT);
  pinMode(C_ECHO_PIN, INPUT);
  pinMode(R_TRIG_PIN, OUTPUT);
  pinMode(R_ECHO_PIN, INPUT);
  motordriver.init();
  motordriver.setSpeed(200,MOTORB);
  motordriver.setSpeed(200,MOTORA);


}

void lookL() {
  turnL(SLOW, SLOW);
}
void lookR() {

  turnR(SLOW, SLOW);
}
void left_encoder_update() {
  left_encoder_count += 1;

}
void right_encoder_update() {
  right_encoder_count += 1;

}
void line_follow()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.read(sensorValues);
  int read_attempts = 0;


  int adj_speed = 0;
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position

  prev_error_found = error_found;
  error_found = 0;
  
  qtrrc_left.read(sensorValues);
  qtrrc_right.read(sensorValues + 8);
  found_max = find_max(sensorValues, NUM_SENSORS * 2);

  error_found = get_error(sensorValues, NUM_SENSORS * 2);
  Serial3.print("ERROR Found:");
  Serial3.println(error_found);
  adj_speed = Kp * error_found + Kd * (error_found - prev_error_found);


  if (adj_speed < -255) adj_speed = -255;
  if (adj_speed > 255)  adj_speed = 255;
  stats[stat_count].adj_speed = adj_speed;
  stats[stat_count].err = error_found;
  
  stat_count++;
  if (stat_count > 30) {
    line_dump();
  }

  advance(adj_speed,-1 * adj_speed );

}

int distance_query(int trig_pin,int echo_pin) {

  int duration =0;
  int distance = 0;
  
/* The following TRIG_PIN/ECHO_PIN cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trig_pin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trig_pin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trig_pin, LOW);
 duration = pulseIn(echo_pin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58.2;
 

 /* Send the distance to the computer using Serial protocol, and
 turn LED OFF to indicate successful reading. */


 
 //Delay 50ms before next reading.
 delay(50);

  return(distance);
}

int turnDistance(int astart, int aend, int dir)
{
  if (dir == 1) {
    if (astart > aend && dir == 1) {
      return(360 - astart + aend);
    }
    else {
      return(aend - astart);
    }
  }
  else {
    if (astart < aend) {
      return(360 - aend + astart);     
    }
    else {
      return(astart - aend);
    }
  }
}


void bulldozer() {
    static int distance = 1000;
    qtrrc_left.read(sensorValues);
    qtrrc_right.read(sensorValues + 8);

    int max_val = find_max(sensorValues, 16);
    if (max_val == 2500) {
      backup(SLOW,SLOW);
      Serial3.println("BACKUP");
      delay(100);
    }

    int ldist = distance_query(L_TRIG_PIN, L_ECHO_PIN);

    int cdist = distance_query(C_TRIG_PIN, C_ECHO_PIN);

    int rdist = distance_query(R_TRIG_PIN, R_ECHO_PIN);
  Serial3.print(">" );Serial3.print( ldist);Serial3.print(",") ; Serial3.print( cdist); Serial3.print( "," ); Serial3.print( rdist);
    if (ldist > cdist && rdist > cdist) {advance(CRUISE, CRUISE ); Serial.println("straight");}
    if (ldist < rdist) {advance(0, SLOW + 32); Serial3.println("left");}
    else if (rdist < ldist) {advance(SLOW + 32, 0);Serial3.println("right");}
 

    

}

int get_front_sensor_info() {
  int err, min_val, max_val;
  err = get_error(sensorValues, NUM_SENSORS * 2);
  min_val = find_min(sensorValues, NUM_SENSORS * 2);
  max_val = find_max(sensorValues, NUM_SENSORS * 2);
  for (int i = 0; i < NUM_SENSORS * 2; i++) {
    Serial3.print(sensorValues[i]);
    Serial3.print(",");
  }
  Serial3.println("");
  Serial3.print("ERR:");
  Serial3.print(err);
  Serial3.print(", MIN:");
  Serial3.print(min_val);
  Serial3.print(", MAX:");
  Serial3.println(max_val);
  return(max_val);
}

int get_beacon_sensor() {
  int analogPin0 = 0;     // potentiometer wiper (middle terminal) connected to analog pin 1
int analogPin1 = 1;
int analogPin2=2;
int analogPin3=3;
int analogPin4=4;
int analogPin5=5;
  int left = analogRead(analogPin0)-analogRead(analogPin1);
  int center = analogRead(analogPin2) - analogRead(analogPin3);
  int right = analogRead(analogPin4) - analogRead(analogPin5);
  Serial3.print(left);
  Serial3.print(",");
  Serial3.print(center);
  Serial3.print(",");
  Serial3.print(right);
  Serial3.print(";");
  if (abs(left) > abs(center) && abs(left) > abs(right)) {
    Serial3.println("Left");
    return -1;
  }  
  else if (abs(right) > abs(center) && abs(right) > abs(left)) {
    Serial3.println("Right");
    return 1;
  }
  else if (abs(center) > abs(left) && abs(center) > abs(right)) {
    Serial3.println("Center");
    return 0;
  }
}

void line_dump() {
      Serial3.println(stat_count);
    for(int i=0;i<stat_count;i++) {
      Serial3.print(stats[i].err);
      Serial3.print(",");
      Serial3.println(stats[i].adj_speed);
    }
    stat_count=0;
}

void process_linefollow_commands(int incomingByte) {
  //INCREASE Kp
  if (incomingByte == 'P') {
    Kp = Kp + 0.001;
    Serial3.print("Kp=");
    Serial3.println(Kp, 3);
  }
    //DECREASE Kp
  if (incomingByte == 'p') {
    Kp = Kp - 0.001;
    Serial3.print("Kp=");
    Serial3.println(Kp, 3);
  }
  //INCREASE Kd
  if (incomingByte == 'D') {
    Kd = Kd + 0.005;
    Serial3.print("Kd=");
    Serial3.println(Kd, 3);
  }
  //DECREASE Kd
  if (incomingByte == 'd') {
    Kd = Kd - 0.005;
    Serial3.print("Kd=");
    Serial3.println(Kd, 3);
  }
    //INCREASE Speed
  if (incomingByte == 'S') {
    base_speed += 5;
  }
  //DECREASE SPEED
  if (incomingByte == 's') {
    base_speed -= 5;
  }
}
void menu() {
  Serial3.println("MENU================");
  Serial3.println("0. Reset");
  Serial3.println("1. Follow line");
  Serial3.println("2. Manual Override");
  Serial3.println("3. Manual Calibrate");
  Serial3.println("4. Read Sensors");
  Serial3.println("5. Distance Query");
  Serial3.println("6. Read Calibrated Sensors"); 
  Serial3.println("7. Auto Calibrate");
  Serial3.println("8. bulldozer");
  Serial3.println("9. Get compass heading");
  Serial3.println("-. Beacon");
  Serial3.println("+. compass calibrate");
  Serial3.println("t. Toggle");
  Serial3.println("Space. Menu and stop");
}

void process_manual_commands(int incomingByte) {
    if (incomingByte == 'a') {
      left_encoder_count = 0;
      right_encoder_count = 0;
      forward(0, CRUISE);
      Serial3.println("Turn Left");
    }
    if (incomingByte == 'd') {
      left_encoder_count = 0;
      right_encoder_count = 0;
      forward(CRUISE, 0);
      Serial3.println("Turn Right");
    }
    if (incomingByte == 'w') {
      left_encoder_count = 0;
      right_encoder_count = 0;
      forward(SLOW, SLOW);
      Serial3.println("Straight");
    }
    if (incomingByte == 's') {
      stop();
      Serial3.print(left_encoder_count);
      Serial3.print(",");
      Serial3.print(right_encoder_count);
      Serial3.println("   Stop");
    }
}

//int curr_mode = 0; //1-line, 2-remote control, 3-calibrate
void loop()
{
  int dist=0;
  compass_loop();
  int incomingByte = 0;
  
  if (Serial3.available() > 0) {
    incomingByte = Serial3.read();
    Serial3.print("INCOMING BYTE>");
    Serial3.print(incomingByte);
    Serial3.print(",Mode>");
    Serial3.println(curr_mode);
  }
  process_linefollow_commands(incomingByte);
  if (curr_mode == 1) {
  
    line_follow();
  }
  else if (curr_mode == 2) {
    process_manual_commands(incomingByte);
  }
  else if (curr_mode == 5) {

   int ldist = distance_query(L_TRIG_PIN, L_ECHO_PIN);

    int cdist = distance_query(C_TRIG_PIN, C_ECHO_PIN);

    int rdist = distance_query(R_TRIG_PIN, R_ECHO_PIN);
    Serial3.print( ldist);
    Serial3.print( ",");
    Serial3.print(cdist);
    Serial3.print(",");
    Serial3.println(rdist);
  }
  else if (curr_mode == 8) {
    bulldozer();
  }
  else if (curr_mode == 10) {
    int dir = get_beacon_sensor();
    if (dir == -1) {
      advance(SLOW ,CRUISE);
    }

    else if (dir == 1) {
      advance(CRUISE ,SLOW);
    }
    else {
      advance(CRUISE,CRUISE);
    }
  }

  if (incomingByte != 0) {
    if (incomingByte == '0') {
      curr_mode = 0;
    }
    else if (incomingByte == '1') {
      stat_count = 0;
      curr_mode = 1;
    }
    else if (incomingByte == '2') {
      curr_mode = 2;
      stop();
      Serial3.println("You take over");
    }
    else if (incomingByte == '3') {
      curr_mode = 3;
    }
    else if (incomingByte == '4') {
  
      qtrrc_left.read(sensorValues);
      qtrrc_right.read(sensorValues + 8);
      for(int i=0;i<16;i++) {
        Serial3.print(sensorValues[i]);
        Serial3.print(":");
      }
      get_front_sensor_info();
  
    }
    else if (incomingByte == '5') {
      curr_mode = 5;
    }
    else if (incomingByte == '6') {
  
      qtrrc_left.read(sensorValues);
      qtrrc_right.read(sensorValues + 8);
  
      for(int i=0;i<16;i++) {
        Serial3.print(sensorValues[i]);
        Serial3.print(",");
      }
      //int max_val = find_max(sensorValues, 16);
      //Serial3.print("MAX:");
      //Serial3.println(max_val);
      
    }
    else if (incomingByte == '7') {
      autocalibrate();
    }
    else if (incomingByte == '8') {
      curr_mode=8;
      Serial3.println("BULLDOZER");
        bulldozer();
    }
    else if (incomingByte == '9') {
      Serial3.println("compass_heading");
      compass_heading();
      Serial3.println(bearing);
    }
    else if (incomingByte == '-') {
    curr_mode=10;
    }
    else if (incomingByte == ' ') {
  
      menu();
      stop();
      curr_mode = 0;
    }
    else if (incomingByte == '+') {
      Serial3.println("Calibrate Compass");
      forward(30,-30);
      compass_offset_calibration(3);
      stop();
    }
    else if (incomingByte == 't') {
      if (toggle == 1) {
        toggle = 0;
      }
      else {
        toggle = 1;
      }
      Serial3.print("Toggle => ");
      Serial3.println(toggle);
    }
  }
}
