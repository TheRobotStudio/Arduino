////////////////////////////////////////////////////////////////////////////
//
//  This file is part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69

#define DEVICE_TO_USE     0
#define NB_SAMPLES_MEDIAN 5
#define NB_ADC            6 //6

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20) //(20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10) //(10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40 //40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200  //115200

double acc_x = 0;
double acc_y = 0;
double acc_z = 0;

//FIFO var to store data to be sent to the median filter
double buff_Qw[NB_SAMPLES_MEDIAN] = {0};
double buff_Qx[NB_SAMPLES_MEDIAN] = {0};
double buff_Qy[NB_SAMPLES_MEDIAN] = {0};
double buff_Qz[NB_SAMPLES_MEDIAN] = {0};

double buff_Ax[NB_SAMPLES_MEDIAN] = {0};
double buff_Ay[NB_SAMPLES_MEDIAN] = {0};
double buff_Az[NB_SAMPLES_MEDIAN] = {0};

double medQw = 0;
double medQx = 0;
double medQy = 0;
double medQz = 0;
double medAx = 0;
double medAy = 0;
double medAz = 0;

const int adcPin[NB_ADC] = {A0, A1, A2, A3, A4, A5};  // Analog input pin for feet sensors

int buffAdcVal[NB_ADC][NB_SAMPLES_MEDIAN] = {0};
int medAdcVal[NB_ADC] = {0};

/*** IMU Functions ***/

void convertAcc()
{
  acc_x = -((MPU.m_rawAccel[0])*2*0.3048)/1000;
  acc_y = -((MPU.m_rawAccel[1])*2*0.3048)/1000;
  acc_z = -((MPU.m_rawAccel[2])*2*0.3048)/1000;  
}

void readIMUVal()
{
  convertAcc();  
    
  //write into the FIFO buffer
  buff_Qw[0] = MPU.m_dmpQuaternion[0];
  buff_Qx[0] = MPU.m_dmpQuaternion[1];
  buff_Qy[0] = MPU.m_dmpQuaternion[2];
  buff_Qz[0] = MPU.m_dmpQuaternion[3];
  buff_Ax[0] = acc_x;
  buff_Ay[0] = acc_y;
  buff_Az[0] = acc_z;
}

double getMedianVal(const double buffer[NB_SAMPLES_MEDIAN]) 
{         
    double sortVal[NB_SAMPLES_MEDIAN];
    
    for(int i=0; i<NB_SAMPLES_MEDIAN; i++)
    {          
        sortVal[i] = buffer[i];
    }
    
    //sort values
    for(int m=1; m<NB_SAMPLES_MEDIAN; ++m)
    {
        double n = sortVal[m];
        int p;

        for(p = m-1; (p >=0) && (n<sortVal[p]); p--)
        {
            sortVal[p+1] = sortVal[p];
        }
        sortVal[p+1] = n;
    }    
       
    return sortVal[2];
}

void shiftBuffer(double buffer[NB_SAMPLES_MEDIAN])
{
  for(int i=NB_SAMPLES_MEDIAN-1; i>0; i--)
  {
     buffer[i] = buffer[i-1]; 
  }  
}

const int d_pin_1 = 7; //A0
const int d_pin_2 = 6; //A1
const int d_pin_3 = 5; //A2

/*** ADC Functions ***/

void selectADCChannel(int chan)
{
  switch(chan)
  {
     case 0:        
       digitalWrite(d_pin_1, LOW); 
       digitalWrite(d_pin_2, LOW);
       digitalWrite(d_pin_3, LOW);
     break;
     
     case 1:    
       digitalWrite(d_pin_1, HIGH); 
       digitalWrite(d_pin_2, LOW);
       digitalWrite(d_pin_3, LOW);
     break;
     
     case 2:        
       digitalWrite(d_pin_1, LOW); 
       digitalWrite(d_pin_2, HIGH);
       digitalWrite(d_pin_3, LOW);
     break;
     
     case 3:       
       digitalWrite(d_pin_1, HIGH); 
       digitalWrite(d_pin_2, HIGH);
       digitalWrite(d_pin_3, LOW);
     break;
     
     case 4:         
       digitalWrite(d_pin_1, LOW); 
       digitalWrite(d_pin_2, LOW);
       digitalWrite(d_pin_3, HIGH);
     break;
     
     case 5:        
       digitalWrite(d_pin_1, HIGH); 
       digitalWrite(d_pin_2, LOW);
       digitalWrite(d_pin_3, HIGH);
     break;
     
     case 6:        
       digitalWrite(d_pin_1, LOW); 
       digitalWrite(d_pin_2, HIGH);
       digitalWrite(d_pin_3, HIGH);
     break;
     
     case 7:        
       digitalWrite(d_pin_1, HIGH); 
       digitalWrite(d_pin_2, HIGH);
       digitalWrite(d_pin_3, HIGH);
     break;

    default:
    break;    
  }
}

void readADCVal()
{
  //left foot
  //heel - White - Y7  
  selectADCChannel(7);
  buffAdcVal[0][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
  //outside - Purple - Y5  
  selectADCChannel(5);
  buffAdcVal[1][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
  //inside - grey - Y6  
  selectADCChannel(6);
  buffAdcVal[2][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
  
  //right foot
  //heel - brown - Y0  
  selectADCChannel(0);
  buffAdcVal[3][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
  //outside - Yellow - Y2  
  selectADCChannel(2);
  buffAdcVal[4][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
  //inside - Orange - Y1  
  selectADCChannel(1);
  buffAdcVal[5][0] = analogRead(adcPin[0]); //adcPin[i]);
  delayMicroseconds(100);
    
    /*
  for(int i=0; i<NB_ADC; i++)
  {    
    buffAdcVal[i][0] = analogRead(i); //adcPin[i]);
    delayMicroseconds(100);
  }*/
}

int getAdcMedianVal(const int buffer[NB_SAMPLES_MEDIAN]) 
{         
    int sortVal[NB_SAMPLES_MEDIAN];
    
    for(int i=0; i<NB_SAMPLES_MEDIAN; i++)
    {          
        sortVal[i] = buffer[i];
    }
    
    //sort values
    for(int m=1; m<NB_SAMPLES_MEDIAN; ++m)
    {
        int n = sortVal[m];
        int p;

        for(p = m-1; (p >=0) && (n<sortVal[p]); p--)
        {
            sortVal[p+1] = sortVal[p];
        }
        sortVal[p+1] = n;
    }    
       
    return sortVal[2];
}

void shiftAdcBuffer(int buffer[NB_SAMPLES_MEDIAN])
{
  for(int i=NB_SAMPLES_MEDIAN-1; i>0; i--)
  {
     buffer[i] = buffer[i-1]; 
  }  
}

/*** Functions for ALL ***/

//Function that send all the sensor data over serial, with a separation character, to be read and split in ROS on the PC side
void sendDataOverSerial()
{
  //send median values
  //Quaternion
  Serial.print("<");
  Serial.print(medQw); //w
  Serial.print("#");
  Serial.print(medQx); //x
  Serial.print("#");
  Serial.print(medQy); //y
  Serial.print("#");
  Serial.print(medQz); //z
  Serial.print("#");
  //Acceleration
  Serial.print(medAx); //x
  Serial.print("#");
  Serial.print(medAy); //y
  Serial.print("#");
  Serial.print(medAz); //z    
  //ADCs
  for(int i=0; i<NB_ADC; i++)
  {
    Serial.print("#");
    Serial.print(medAdcVal[i]); //presure sensor      
  } 
  Serial.print("#");  
  //New line
  Serial.println();    
}

//get median values
void getADCMedVal()
{
  //ADC
  for(int i=0; i<NB_ADC; i++)
  {
    medAdcVal[i] = getAdcMedianVal(buffAdcVal[i]);
  }
}

//get all median values
void getIMUMedVal()
{ 
  //IMU
  medQw = getMedianVal(buff_Qw);
  medQx = getMedianVal(buff_Qx);
  medQy = getMedianVal(buff_Qy);
  medQz = getMedianVal(buff_Qz);
  medAx = getMedianVal(buff_Ax);
  medAy = getMedianVal(buff_Ay);
  medAz = getMedianVal(buff_Az);    
}

//shift all
void shiftAllBuffers()
{  
  //ADC
  for(int i=0; i<NB_ADC; i++)
  {
    shiftAdcBuffer(buffAdcVal[i]);
  }
  
  //IMU
  shiftBuffer(buff_Qw);
  shiftBuffer(buff_Qx);
  shiftBuffer(buff_Qy);
  shiftBuffer(buff_Qz);
  shiftBuffer(buff_Ax);
  shiftBuffer(buff_Ay);
  shiftBuffer(buff_Az);    
}

//Setup Arduino program
void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  //analogReference(EXTERNAL);
  //Serial1.begin(SERIAL_PORT_SPEED);
  //Serial.print("Arduino9150 starting using device "); 
  //Serial.println(DEVICE_TO_USE);
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
  /*
  for(int k=0; k<NB_ADC; k++)
  {
    pinMode(adcPin[k], INPUT);  
  }
  */
  
  pinMode(d_pin_1, OUTPUT);
  pinMode(d_pin_2, OUTPUT);
  pinMode(d_pin_3, OUTPUT);
  
  //fill the data FIFO before launching the loop
  for(int i=0; i<NB_SAMPLES_MEDIAN; i++)
  { 
    //ADC
    for(int j=0; j<NB_ADC; j++)
    {
      buffAdcVal[j][i] = analogRead(adcPin[j]);
      delayMicroseconds(300);
    }
    
    //IMU
    MPU.selectDevice(DEVICE_TO_USE);
    if (MPU.read()) 
    {
      convertAcc();
      
      buff_Qw[i] = MPU.m_dmpQuaternion[0];
      buff_Qx[i] = MPU.m_dmpQuaternion[1];
      buff_Qy[i] = MPU.m_dmpQuaternion[2];
      buff_Qz[i] = MPU.m_dmpQuaternion[3];
      
      buff_Ax[i] = acc_x;
      buff_Ay[i] = acc_y;
      buff_Az[i] = acc_z;      
    }
  }   
}

//MAIN Function
void loop()
{  
  //Get ADC values
  readADCVal();
  /*
  //Get IMU values
  MPU.selectDevice(DEVICE_TO_USE); //only needed if device has changed since init but good form anyway
  if (MPU.read()) 
  {      
    //fill the IMU buffer with one new data
    readIMUVal();
    
    //fill median variables with new values, before send to serial
    //getAllMedVal();   
    getIMUMedVal();
    getADCMedVal();
        
    //send filtered data over serial
    sendDataOverSerial();
        
    //shift data in buffer for the next reading
    shiftAllBuffers();
  }
  */
  /*
  else
  {
      Serial.println("ERROR");  
      medQw = 0;
      medQx = 0;
      medQy = 0;
      medQz = 0;
      medQ
  }
  */
  
  getADCMedVal();
  //TEST HACKED with 1 acc 1 gyro
  selectADCChannel(3);
  medQw = analogRead(adcPin[0]);
  delayMicroseconds(100);
  selectADCChannel(4);
  medQx = analogRead(adcPin[0]);
  //delayMicroseconds(100);  
  //TEST
  
  medQy = 0;
  medQz = 0;
  medAx = 0;
  medAy = 0;
  medAz = 0;
  
  //send filtered data over serial
  sendDataOverSerial();
        
  //shift data in buffer for the next reading
  //ADC
  for(int i=0; i<NB_ADC; i++)
  {
    shiftAdcBuffer(buffAdcVal[i]);
  }
  
  //delay(3);
}
