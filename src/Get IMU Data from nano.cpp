#include <Arduino.h>
#include <Wire.h>

////A5-SCL A4-SDA for nano

const int IMU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float AccErrorX, AccErrorY, AccErrorZ;
float elapsedTime, currentTime, previousTime;
String AccX_str, AccY_str, AccZ_str;
int c = 0;
float AFS = 16384.0;
int g_range = 0x00;
int g = 0;
int go = 0;

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 500) {
    Wire.beginTransmission(IMU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / AFS;
    AccY = (Wire.read() << 8 | Wire.read()) / AFS;
    AccZ = (Wire.read() << 8 | Wire.read()) / AFS;
    // Sum all readings
    AccErrorX += AccX;
    AccErrorY += AccY;
    AccErrorZ += AccZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 500;
  AccErrorY = AccErrorY / 500;
  AccErrorZ = AccErrorZ / 500;
  c = 0;
 
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("AccErrorZ: ");
  Serial.println(AccErrorZ);
}

void send_IMU_data(){
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > 16){
    // === Read acceleromter data === //
    Wire.beginTransmission(IMU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / AFS; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / AFS; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / AFS; // Z-axis value
  /*
    AccX_str = String(AccX, 3);
    AccY_str = String(AccY, 3);
    AccZ_str = String(AccZ + 0.06, 3);

    Serial.print(AFS);
    Serial.print(", ");
    Serial.println(g_range, HEX);
  */
    //Serial.print(AccX);// + " " + AccY_str + " " + AccZ_str);
    //Serial.print(" ");
    Serial.println(AccY);
    //Serial.print(" ");
    //Serial.println(AccZ);
 
    prev_ms = millis();
  }
}





void set_g_range(int g){
  if(g == 2){
    g_range = 0x00;
    AFS = 16384.0;
  }
  if(g == 4){
    g_range = 0x08;
    AFS = 8192.0;
  }
  if(g == 8){
    g_range = 0x10;
    AFS = 4096.0;
  }
  if(g == 16){
    g_range = 0x18;
    AFS = 2048.0;
  }
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(IMU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(g_range);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  delay(20);
}

void setup() {
  Serial.begin(115200);
  //while(!Serial);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(IMU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer Clock speed
  Wire.beginTransmission(IMU);
  Wire.write(0x24); //I2C master Control
  Wire.write(0x0D); //400kHz
  Wire.endTransmission(true);
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(IMU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  delay(20);

  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  //delay(20);
  Serial.println("Ready");
}

void loop() {
 /* while(Serial.available() > 1){
    g = Serial.parseInt();
    if(g != 0)
      go = 1;
    else{
      go = 0;
      Serial.println("Ready");
    }
    set_g_range(g);
  }

  if(go){
    send_IMU_data();
  }*/
  send_IMU_data();
}


