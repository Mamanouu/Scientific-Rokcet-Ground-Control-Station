// This is the flight computer software in progress the majority of the function below serve for telemetry and data visualization
// Important we need to add the gps off handling to this code to not crash and stop the rest of the code, currently i am working indoor

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Create instances
MPU6050 mpu; // mpu6050 object
Adafruit_BMP280 bmp; // bmp280 object
TinyGPSPlus gps; // gps object
Servo myServo; // Servo Object

HardwareSerial gpsSerial(1);
float launch_pre;
int deploy_drogue = 0;
int deploy_main = 0;
float apogee = 0.0;
int servoPin = 4;
RF24 radio(27, 5); //ce, csn 
float lastlat = 36.504094601907404;
float lastlon = 2.8773506949226717;
float lastspeed;
float lastsat;
float altitude;
float temperature;
const byte address[6] = "00001";  // radio adresses
const int chipSelect = 15; // Sdcard data logger
File dataFile;

// MPU control/status variables
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

// Quaternion
Quaternion q; 

#define INTERRUPT_PIN 14
#define LED_PIN 2 
#define buzz 26
volatile bool mpuInterrupt = false;  

bool loopRunning = false; // loop status flag

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000); 

    Serial.begin(115200);
    
    myServo.attach(servoPin);
    myServo.write(0);

    // Initialize GPS serial
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
    
    // Initializz MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    pinMode(INTERRUPT_PIN, OUTPUT);
    pinMode(buzz, OUTPUT);
    devStatus = mpu.dmpInitialize();

    // Checking MPU status
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        loopRunning = true;
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println("MPU Initialization failed!");
        digitalWrite(buzz, HIGH);
        delay(250);
        digitalWrite(buzz, LOW);
        delay(250);
        digitalWrite(buzz, HIGH);
        delay(250);
        digitalWrite(buzz, LOW);
        while (1);  
    }

    // Initializing BMP280 sensor
    Serial.println("Initializing BMP280...");
    if (bmp.begin(0x76)) {  
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,  
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        Serial.println("BMP OK");
    } else {
        Serial.println("BMP Initialization failed!");
        digitalWrite(buzz, HIGH);
        delay(250);
        digitalWrite(buzz, LOW);
        delay(250);
        digitalWrite(buzz, HIGH);
        delay(250);
        digitalWrite(buzz, LOW);
        while (1);
    }

    pinMode(LED_PIN, OUTPUT);

    // Initialize the NRF24
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);  // power level set to max for max signal range
    radio.stopListening(); // setting the fc nrf as reciever.

    launch_pre = bmp.readPressure(); // getting launch site pressure
    
    digitalWrite(LED_PIN, HIGH); // a hint to see if the setup run correctly
    
    // Initialize the SD card
    if (!SD.begin(chipSelect)) {
      Serial.println("SD card initialization failed!");
      digitalWrite(buzz, HIGH);
      delay(250);
      digitalWrite(buzz, LOW);
      delay(250);
      digitalWrite(buzz, HIGH);
      delay(250);
      digitalWrite(buzz, LOW);
      return;
    }
    Serial.println("SD card initialized.");

    String fileName = "log.csv";
    dataFile = SD.open(fileName.c_str(), FILE_WRITE);

    // Writing the CSV header
    if (dataFile) {
      dataFile.println("Q, Qx, Qy, Qz, Altitude, Temperature, Latitude, Longitude, Apogee");
      dataFile.close();
      Serial.println("CSV file created with header.");
      digitalWrite(buzz, HIGH);
      myServo.write(90);
      delay(1000);
      digitalWrite(buzz, LOW);
      myServo.write(0);
      delay(1000);
      digitalWrite(buzz, HIGH);
      myServo.write(90);
      delay(1000);
      digitalWrite(buzz, LOW);
      myServo.write(0);
      delay(1000);
      digitalWrite(buzz, HIGH);
      delay(1000);
      digitalWrite(buzz, LOW);
      delay(1000);
      digitalWrite(buzz, HIGH);
      delay(1000);
      digitalWrite(buzz, LOW);
      delay(1000);
      digitalWrite(buzz, HIGH);
      delay(1000);
      digitalWrite(buzz, LOW);
    } else {
      Serial.println("Error creating file.");
      digitalWrite(buzz, HIGH);
      delay(250);
      digitalWrite(buzz, LOW);
      delay(250);
      digitalWrite(buzz, HIGH);
      delay(250);
      digitalWrite(buzz, LOW);
    }
}

void loop() {
    if (loopRunning && dmpReady) {
        // Read a packet from mpu FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
            mpu.dmpGetQuaternion(&q, fifoBuffer); //reading quaterionos from mpu6050

            // altitude and temperature from BMP280
            altitude = bmp.readAltitude(launch_pre / 100.0);
            temperature = bmp.readTemperature();
 

            if (altitude > apogee) {
              apogee = altitude;
            }

            if (altitude < apogee - 5) {
              // need to be changed since we use an electromagnetic lock
              deploy_drogue = 1;
              myServo.write(90);
              //digitalWrite(LED_PIN, LOW); visual hint for deployement signal test if detected
              Serial.println("Drogue Parachute Deployed !");
              
              if (altitude < 200 & altitude > 190) {
                //myservo2.write(90);
                deploy_main = 1;
                Serial.println("Main parachute Deployed !");
              }
            }
            
            while (gpsSerial.available()) {
                gps.encode(gpsSerial.read());
                if (gps.location.isValid()) {
                  float latitude = gps.location.lat();
                  float longitude =  gps.location.lng();
                  float speed = gps.speed.mps();
                  float satellites = gps.satellites.value();
                  lastlat = latitude;
                  lastlon = longitude;
                  lastspeed = speed;
                  lastsat = satellites;
                }}


            //Serial.print(lastlat);Serial.print(lastlon);Serial.print(lastspeed);Serial.println(lastsat);

            char quat[32]; // first data packet for quaternions
            char altemp[32]; // second data packetfor altitude temp sat and speed
            char coor[32]; // thrid data pakcet for latitude longitude deploy signals and apogee

            // Prepare and format the data to send in three differrent packets
            snprintf(quat, sizeof(quat), "@%.2f,%.2f,%.2f,%.2f,", q.w, q.x, q.y, q.z); //@
            snprintf(altemp, sizeof(altemp), "&%.2f,%.2f,%.2f,%.2f,", altitude, temperature, lastsat, lastspeed); //&
            snprintf(coor, sizeof(coor), "#%.6f,%.6f,%d,%d,%.2f", lastlat, lastlon, deploy_drogue, deploy_main, apogee); //#
            
            bool success = radio.write(&quat, sizeof(quat)); // first pachet send
            bool success2 = radio.write(&altemp, sizeof(altemp)); // second packet send
            bool success3 = radio.write(&coor, sizeof(coor)); // third packet send
            
            // small serial monitor check to see if data is sent or not
            if (success) {
                Serial.println("Transmission successful");
                Serial.print(quat);Serial.print(altemp);Serial.println(coor);
              } else {
                Serial.println("Transmission failed");
              }
            // write data to the csv file
          dataFile = SD.open("log.csv", FILE_WRITE);
            if (dataFile) {
              dataFile.print(q.w);
              dataFile.print(",");
              dataFile.print(q.x);
              dataFile.print(",");
              dataFile.print(q.y);
              dataFile.print(",");
              dataFile.print(q.z);
              dataFile.print(",");
              dataFile.print(altitude);
              dataFile.print(",");
              dataFile.print(temperature);
              dataFile.print(",");
              dataFile.print(lastlat);
              dataFile.print(",");
              dataFile.print(lastlon);
              dataFile.print(",");
              dataFile.print(apogee);
              dataFile.println();
              dataFile.close(); 
              Serial.println("Data saved to CSV file.");
              } else {
                Serial.println("Error opening CSV file.");
              }
        }
        }
}
