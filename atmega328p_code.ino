#include <Wire.h>
#include <SoftwareSerial.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
int Araw1,Araw2;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
String Condition,Data;
#define led_pin 8

// Define connections to USS sensor
int pinRX = 10;
int pinTX = 11;
 
// Array to store incoming serial data
unsigned char data_buffer[4] = {0};
 
// Integer to store distance
int distance = 0;
 
// Variable to hold checksum
unsigned char CS;
 
// Object to represent software serial port PF USS
SoftwareSerial mySerial(pinRX, pinTX);

 

void setup() {

  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  mySerial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);

  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  //delay(20);
}

void loop() {

  Data = (get_gyro_value()+','+get_battery_value()+','+get_USS_value());
  delay(1000);
  Serial.flush();
  delay(1000);
  //Serial.println(Condition);
  Serial.print(Data);
  delay(2000);
}

String get_battery_value(){  

  int sensorValue = analogRead(A0); //read the A0 pin value
  float voltage = sensorValue * (5.00 / 1023.00)  ; //convert the value to a true voltage.
  float precentage = (voltage-3.3)*100/0.4;
  
  
  if (voltage < 3.40) //set the voltage considered low battery here
  {
    digitalWrite(led_pin, HIGH);
  }
    delay(1000);
     return String(precentage);  
}  

String get_gyro_value(){  

  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value
 
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
     
  float ACC = pow(pow(AccX, 2) + pow(AccY, 2) + pow(AccZ, 2), 0.5);   // calculating Amplitute vactor for 3 axis

  
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime)/ 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 32.8;

  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)


  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = (GyroX * elapsedTime) + gyroAngleX; // deg/s * s = deg
  gyroAngleY = (GyroY * elapsedTime) + gyroAngleY;
  gyroAngleZ = (GyroZ * elapsedTime) + gyroAngleZ;
  // Print the values on the serial monitor
  //Serial.print(AccX);
  //Serial.print('/');
  //Serial.print(AccY);
  //Serial.print('/');
  //Serial.print(AccZ);
  //Serial.print('/');
  //Serial.println(ACC);

  if ((AccZ > 0.95 && AccZ < 1.05) && (AccX > 0.05 || AccX < -0.05) && (AccY > 0.05 || AccY < -0.05) ) {
    Condition="true";
  } 

  else {
    Condition="false";
  }
    delay(1000);
    return String(Condition); 
    
}  

String get_USS_value() {
 
  // Run if data available
  if (mySerial.available() > 0) {
 
    delay(4);
 
    // Check for packet header character 0xff
    if (mySerial.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }
 
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        // Print to serial monitor
        //Serial.print("distance: ");
        //Serial.flush();
        //delay(1000);
        //Serial.println(distance);
        //Serial.println(" mm");
        
        delay(1000);
        //espSerial.write(distance);
        return String(distance)
  
      }
    }
  }
}



