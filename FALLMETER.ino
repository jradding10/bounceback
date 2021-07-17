//FALLMETER
//BY JACQUELINE RADDING
// FALLMETER is a wireless device that detects a fall or pressed emergency button and sends the recent fall data to a bluetooth connected device.
#include <SoftwareSerial.h> // bluetooth serial reading

SoftwareSerial BTserial(10, 11); // RX | TX for bluetooth



#include <Wire.h> // library to help read acceleration data
#include <MPU6050.h> // library for acceleration sensor 

MPU6050 mpu; // this is the accelerometer


#include <Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // the gathered accel
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; // the calculated accel
int ledpin = 5;
int recentf = 0; // this variable will indicate a recent fall, and is set to zero b/c a fall has not occured
boolean fall = false; //stores if a fall has occurred
int emerg = 0; //indicates if button was pressed
int potpin = A0; // this will detect the PWM on the potentiometer
int beeper = 4; // this is the beeper pin that will go off when fall occurs
int val; // this will be the analog pot value that will decide tone of beeper

int fallcount = 0; // this will be the time/ fall counter

int button1 = 3; // emergency button

void setup() {
  //pinMode(13, OUTPUT); // this is supplyon 5v to transmitter

  pinMode(ledpin, OUTPUT); //led pin is an output if device is on
  pinMode(button1, INPUT_PULLUP); //sets to button input pullup
  Wire.begin(); // begins the MPU
  Wire.beginTransmission(MPU_addr); // gets ready to recieve data
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero to start accel
  Wire.endTransmission(true); // wireless tranmission just in case
  Serial.begin(9600); // for debug
  BTserial.begin(9600); // for bluetooth device
  attachInterrupt(digitalPinToInterrupt(button1), button, LOW); // interupt button 1 and sends emergency fall alert infinitely
  // button interupt for emergency fall button


}


// make switch cases for triggers for fall
void loop() {


  //digitalWrite(13,HIGH); // turns on tranmitter
  mpu_read(); // reads accel

  // values may be different for you
  mpu_val(); // computes accel


  // calculating Amplitute vector for 3 axis
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5); // this is the total mangitude of acceleration
  int AM = (Raw_AM * 10) - 2; // get this value to the four values I want for my switch case (2,4,8,16)
  // it by for using if else conditions

  //sendonline();
  switch (AM) // determines the severity of the fall
  {
    case 16:            Serial.println("16 g"); fall = true; break; //this is the fall threshold of the magnitude of acceleration send fall
    case 8:             Serial.println("8 g"); fall = false; break; // this value is not enough to be considered a hard fall
    case 4:             Serial.println("4 g"); fall = false; break; // this value is not enough to be considered a hard fall
    case 2:             Serial.println("2 g"); fall = false; break;// this value is not enough to be considered a hard fall
  }


  if (fall == true) {
    for (int i = 0; i < 3; i++) { // counting loop makes sure person does or does not get up
      int newax = ax; // new value has a greater acceleration on the x-asic
      mpu_read(); // reads accel
      mpu_val(); // calc accel
      digitalWrite(ledpin, HIGH); // led indicates fall movement
      if (newax > ax) { // if person does get up
        fallcount = fallcount + 1; // counts the acceleration threshold hits
        digitalWrite(ledpin, LOW); //turns off LED if no fall is detected
        // sendfall();
        delay(200); // fall data within .2 seconds so they have time to get up

      }
      if (fallcount == 3) { // if person does not get up
        int var = analogRead(potpin);
        sendfall(); // sends emergency response if person does not get up
        //Mapping the Values between 0 to 255 because we can give output
        //from 0 -255 using the analogwrite funtion
        int tonemap = map(var, 0, 1023, 40, 100); // tone is based on the potenitometer so it can warn the workers if there is a fall
        sendfall();
        tone(beeper, tonemap, 100); // sends a tone
        recentf = recentf + 1; // adds to recent falls

      }

    }
  }



  else {
    digitalWrite(ledpin, LOW); //turns off LED if no fall is detected
    // turns off beeper once fall stops
    fallcount = 0;  // resets fall counter

    sendonline(); // if no recent fall, send fall-free status
    noTone(beeper); // turns off beeper once fall stops
    digitalWrite(ledpin, LOW); //turns off LED if no fall is detected


  }
  while (recentf >= 10) { // if the 10 less severe fall counter threshold is exceeded, infinitely send emergency reading
    sendfall();

  }

  delay(100);

}



void sendfall() { // sends fall alert
  // recentf = 1; // this variable will tell the user if there has been a fall that was missed
  BTserial.print("1234"); // this is the device #

  BTserial.print(",");


  BTserial.print("FALL"); // indicates fall
  BTserial.print(",");
  BTserial.print("EMERG"); // indicates fall
  BTserial.print(",");
  BTserial.print(recentf); // fall count that has happened

  BTserial.print(";");
  //fallcount = fallcount +1;
  //message to the receiving device

  delay(20);
}


void sendonline() { // sends fall alert
  // recentf = 1; // this variable will tell the user if there has been a fall that was missed
  BTserial.print("1234"); // this is the device #

  BTserial.print(",");


  BTserial.print("NORMAL"); // indicates fall
  BTserial.print(",");
  BTserial.print("NORMAL"); // indicates fall
  BTserial.print(",");
  BTserial.print(recentf); // fall has happened

  BTserial.print(";");
  //fallcount = fallcount +1;
  //message to the receiving device

  delay(20);
}


void button() { // button action

  BTserial.print("1234"); // this is the device #

  BTserial.print(",");


  BTserial.print("FALL"); // indicates fall
  BTserial.print(",");
  BTserial.print("EMERG"); // indicates fall
  BTserial.print(",");
  BTserial.print("BUTTON"); // fall has happened

  BTserial.print(";");
  //fallcount = fallcount +1;
  //message to the receiving device
  tone(beeper, 275 , 500); // sends a tone
  delay(20);

}



void mpu_read() { // this reads the acceleration values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
int mpu_val() { // this calculates acceleration values
  ax = (AcX - 16588) / 16384.00; // calibration of acceleration I got to fit my accelerometer to make values 0 at start
  ay = (AcY + 4) / 16384.00;
  az = (AcZ + 1988) / 16384.00;

  gx = (GyX + 3352) / 131.07;
  gy = (GyY - 100) / 131.07;
  gz = (GyZ + 50) / 131.07;
}
