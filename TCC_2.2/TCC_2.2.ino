#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h> 
#include <ESPmDNS.h> 
#include <WiFiUdp.h> 
#include "MPU6050_6Axis_MotionApps20.h"

#define MOT_PIN 25
#define SERV_LIDAR 21
#define ENC_PIN 14
#define deg 12.0
#define OUT_OF_RANGE 8190
#define uS_MIN  550
#define uS_MAX  2000
#define DEL_uS 10

#define MPU6050_ACCEL_OFFSET_X -564
#define MPU6050_ACCEL_OFFSET_Y 1060
#define MPU6050_ACCEL_OFFSET_Z 1167
#define MPU6050_GYRO_OFFSET_X  -45
#define MPU6050_GYRO_OFFSET_Y  -24
#define MPU6050_GYRO_OFFSET_Z  83


static int taskCore = 0;

const char* ssid = "TARDIS";
const char* password =  "PraiaPeruibe";
const uint16_t port = 8090;
const int udpPort = 8091;
const char * host = "192.168.0.115";

Servo motor;
Servo servo_lidar;
VL53L0X sensor;
WiFiClient client;
WiFiUDP udp;
MPU6050 mpu;

volatile unsigned long int last_t = 0;
volatile double del = 0;
volatile int angle_real_alt = 0, angle_estimate_alt = 0;
volatile int value = 0;
volatile unsigned long int last_volta = 0;

String data_s;
int sensor_read;
int last;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int us = uS_MIN;
int incr_us = DEL_uS;
int inc = DEL_uS;
int angle_az = 0;
int angle_max = 180;
int angle_min = 0;
unsigned long int last_osc = 0;

int offset_az = 0;
int offset_alt = 195;

int osc_t = 90;
int vel_e = 25;
bool onoff = false;

uint8_t buffer[50] = "";

hw_timer_t * timer = NULL;

int vel_mode_az[]  = {0,400,300,200,100};
int vel_mode_alt[] = {25, 49, 50, 51, 52};

void initialize_MPU() {
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
  mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
  mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
  mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
  mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
  mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
  if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 

}

void IRAM_ATTR estimation() {
  angle_estimate_alt++;
  angle_estimate_alt = angle_estimate_alt % 360;
}

void wifi_setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
  udp.begin(udpPort);
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

void OTA_setup() {
  ArduinoOTA.setHostname("LIDAR");
  ArduinoOTA.begin();
}

void wifi_send(String data_stream) {
    if (client.connect(host, port)) {
      client.print(data_stream);
    }
    client.stop();
}

void setup_timer() {
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &estimation, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
}


void oscillating_radar(int next){
  if(next > 0 && millis() - last_osc > next) {
    angle_az = map(us, uS_MIN, uS_MAX, angle_min, angle_max);
    if(angle_az >= 180 && inc > 0){
      inc = -incr_us;
    }
    else if(angle_az <= 0 && inc < 0) {
      inc = incr_us;
    }
    if(inc > 0) angle_az -= offset_az;
    us += inc;
    servo_lidar.writeMicroseconds(us);
    last_osc = millis();
  }
}

void get_inclination () {
  if (!dmpReady) return;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  Serial.println(fifoCount);
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  } 
  else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] *= 180/M_PI;
        ypr[1] *= 180/M_PI;
        ypr[2] *= 180/M_PI;  
  }  
}

/*void update_state(String cmd) {
  char opt = cmd.charAt(0);
  int value =  cmd.substring(1).toInt();
  Serial.println(String(opt) + " " + String(value));
  if(opt == 'a') {osc_t = value;}
  else if (opt == 'e'){motor.write(value/10);}
  else if (opt == 'o') {
    if(value) {
      unsigned long int n = millis();
      while(millis()-n < 50) get_inclination();
      wifi_send("c0 " + String(ypr[0])+ " " +String(ypr[1]) + " "+ String(ypr[2]));
      Serial.println("send cal " + String(ypr[0])+ " " +String(ypr[1]) + " "+ String(ypr[2]));
      delay(100);
    }
    onoff = value;
  }
}*/

void update_state(String cmd) {
  char opt = cmd.charAt(0);
  int value =  cmd.substring(1).toInt();
  Serial.println(String(opt) + " " + String(value));
  if(opt == 'a') osc_t = vel_mode_az[value];
  else if (opt == 'e') motor.write(vel_mode_alt[value]);
  else if (opt == 'o') {
    if(value) {
      unsigned long int n = millis();
      while(millis()-n < 50) get_inclination();
      wifi_send("c0 " + String(ypr[0])+ " " +String(ypr[1]) + " "+ String(ypr[2]));
      Serial.println("send cal " + String(ypr[0])+ " " +String(ypr[1]) + " "+ String(ypr[2]));
      delay(100);
    }
    onoff = value;
  }
}

void read_command () {
  if(udp.parsePacket() > 0){
    udp.read(buffer, 50);
    //Serial.print("Server to client: ");
    //Serial.println((char *)buffer);
    update_state((char *)buffer);
  }
}

void setup_sensor() {
  Wire.begin(5, 4);
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
  sensor.setMeasurementTimingBudget(20000);
}

void IRAM_ATTR timming () {
  unsigned long int now_t = millis();
  double tmp = now_t - last_t;
  if (tmp > 0) {
    if(del/tmp < 0.5 && angle_real_alt > 300) {
      angle_real_alt = 0;
    }
    del = tmp;
    last_t = now_t;
    angle_real_alt += deg;
    timerAlarmWrite(timer, del*1000.0/deg, true);
    angle_estimate_alt = angle_real_alt - offset_alt;
  }
}

void setup() {
  Serial.begin(115200);
  motor.attach(MOT_PIN);
  motor.write(25);
  servo_lidar.attach(SERV_LIDAR);
  setup_sensor();
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), timming, CHANGE); 
  delay(1000);
  Serial.println("conectando");
  wifi_setup();
  Serial.println("conectado");
  OTA_setup();
  Serial.println("configurado");
  setup_timer();
  initialize_MPU();
  memset(buffer, 0, 50);
  xTaskCreatePinnedToCore(
                    coreTask,   
                    "coreTask", 
                    10000,      
                    NULL,       
                    0,          
                    NULL,       
                    taskCore);  
}


void coreTask( void * pvParameters ){
    String taskMessage = "Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    while(true){
      if(onoff){
        wifi_send(data_s);
        delay(25);
      }
      else delay(20);
    }
}

void loop() {
  ArduinoOTA.handle();
  read_command();
  //Serial.println(String(vel_e) + " " + String(osc_t) + " " + String(onoff));
  if(onoff){
    sensor_read = sensor.readRangeSingleMillimeters();
    if(sensor_read < OUT_OF_RANGE) data_s = String(angle_estimate_alt) + " " + String(angle_az) + " " + String(sensor_read);
    else data_s = String(angle_estimate_alt) + " " + String(angle_az) + " " + String(-1);
    oscillating_radar(osc_t);
    //Serial.println(data_s +" "+ String(millis()-last));
    last = millis();
  }
}

