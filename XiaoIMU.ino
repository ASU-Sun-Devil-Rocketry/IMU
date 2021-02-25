#include <Wire.h>
#include <SPI.h>
#include <SD.h>

//LED PIN DEFINITION
#define R 0
#define G 1
#define B 2

//MPU VARIABLES
const int MPU = 0x68; // MPU6050 I2C address
double AccX, AccY, AccZ;
float yaw, pitch, roll;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime;
unsigned long currentTime, previousTime;
int c = 0;

//FILE WRITE
const String FILE_NAME = "inc.txt"; //file with next file name (unique naming per file)
File myFile;
String fileNm = "";
uint8_t num;


void setup() {
  Serial.begin(115200);

  //INITIALIZE LEDS + SET TO OFF
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);

  digitalWrite(R, LOW);
  digitalWrite(G, LOW);
  digitalWrite(B, HIGH);

  //INITIALIZE ACCELEROMETER
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


  //USED TO WAIT FOR SERiAL CONNECTION
  //while(!Serial){ }
  //Serial.println("CONNECTED");

  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error(1000);
  delay(100);

  //BLINK INDICATOR: Indicates IMU calibration complete
  for (int i = 0; i < 3; i++) {
    digitalWrite(R, HIGH);
    digitalWrite(G, LOW);
    digitalWrite(B, LOW);
    delay(100);
    digitalWrite(R, LOW);
    digitalWrite(G, HIGH);
    digitalWrite(B, LOW);
    delay(100);
    digitalWrite(R, LOW);
    digitalWrite(G, LOW);
    digitalWrite(B, HIGH);
    delay(100);
  }

  //INITIALIZE SD CARD
  Serial.print("Initializing SD card...");
  if (!SD.begin(3)) {
    Serial.println("initialization failed!");
    return;
  }

  //READING FILE_NAME to name current file
  myFile = SD.open(FILE_NAME);
  if (myFile) {
    while (myFile.available()) {
      num = myFile.parseInt();
      Serial.println(num);
    }
    myFile.close();
  } else {
    Serial.println("error opening");
  }

  //.txt or .dat file
  fileNm = String(num) + ".txt";
  Serial.println("File Name: " + String(fileNm));
  num++;

  //WRITING TO FILE
  myFile = SD.open(FILE_NAME, FILE_WRITE);
  if (myFile) {
    myFile.println();
    myFile.print(int(num));
    myFile.close();
  } else {
    Serial.println("error opening");
  }

  //INITIALIZE NEW FILE
  myFile = SD.open(String(fileNm), FILE_WRITE);
  myFile.println("Time\tAX\tAY\tAZ\tYaw\tPitch\tRoll");
  myFile.close();

}

void loop() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

  if (AccX > 8) {
    AccX -= 16;
  }
  if (AccY > 8) {
    AccY -= 16;
  }
  if (AccZ > 8) {
    AccZ -= 16;
  }

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  // Read gyroscope data
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = micros();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000000.0; // Divide by 1000000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  if (GyroX > 250) {
    GyroX -= 500;
  }
  if (GyroY > 250) {
    GyroY -= 500;
  }
  if (GyroZ > 250) {
    GyroZ -= 500;
  }

  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // raw values deg/s, multiply by s to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ * elapsedTime;

  // filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  Serial.print(String(currentTime / 1000) + '\t');
  Serial.print(F("AcX = "));
  Serial.print(String(AccX) + "\t");
  Serial.print(F("AcY = "));
  Serial.print(String(AccY) + "\t");
  Serial.print(F("AcZ = "));
  Serial.print(String(AccZ) + "\t");
  Serial.print(F("Yaw = "));
  Serial.print(String(yaw) + "\t");
  Serial.print(F("Pitch = "));
  Serial.print(String(pitch) + "\t");
  Serial.print(F("Roll = "));
  Serial.print(String(roll) + "\t");

  //WRITE TO SD-CARD IF AVAILABLE
  myFile = SD.open(String(fileNm), O_CREAT | O_WRITE);
  if (myFile) {
    myFile.println(String(currentTime / 1000) + "\t" + String(AccX) + "\t" + String(AccY) + "\t" + String(AccZ) + "\t" + String(yaw) + "\t" + String(pitch) + "\t" + String(roll));
    digitalWrite(R, LOW);
    digitalWrite(G, HIGH);
    digitalWrite(B, LOW);
    myFile.close();
  } else {
    Serial.print("\terror opening");
    digitalWrite(R, HIGH);
    digitalWrite(G, LOW);
    digitalWrite(B, LOW);
  }


  Serial.println();
}

void calculate_IMU_error(int numVal) {
  // Note accel should be flat for calibration (standstill) for calibration period
  // Read accelerometer values numVal times
  while (c < numVal) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

    if (AccX > 8) {
      AccX -= 16;
    }
    if (AccY > 8) {
      AccY -= 16;
    }
    if (AccZ > 8) {
      AccZ -= 16;
    }

    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  //Divide the sum by numVal to get the error value
  AccErrorX = AccErrorX / numVal;
  AccErrorY = AccErrorY / numVal;
  c = 0;

  // Read gyro values numVal times
  while (c < numVal) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    if (GyroZ > 250) {
      GyroZ -= 500;
    }
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  //Divide the sum by numVal to get the error value
  GyroErrorX = GyroErrorX / numVal;
  GyroErrorY = GyroErrorY / numVal;
  GyroErrorZ = GyroErrorZ / numVal;

  // Print the error values on the Serial Monitor
  Serial.print(F("AccErrorX: "));
  Serial.println(AccErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ);
}
