/* Analog Input 

   This example code is in the public domain.
*/
#include <ADC.h>
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <arm_math.h>
#include <wchar.h>
#include "ff.h"

#define Pressure_BUFFSIZE 100*4 //want 100 from each of the 4 sensors or 50 from 8 
float32_t Pressure_buff[Pressure_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
uint16_t Pressure_buff_idx = 0; //starts at zero index

#define BUFFSIZE 4*Pressure_BUFFSIZE //because its uint32 so its broken down into 4 uint8_ts
#define NUMSAMPLES 100*8 //needs the you since all four sensors are incrementing it 
uint8_t buffer[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );


// ***** MicroSD Card Stuff ***** //
FRESULT rc;        /* Result code */
FATFS fatfs;      /* File system object */
FIL fil;        /* File object */
DIR dir;        /* Directory object */
FILINFO fno;      /* File information object */
UINT wr;
FATFS *fs;

// ***** Sensor Sampling Stuff ***** //
ADC *adc = new ADC(); // adc object
const int P11_PIN = A0;// only A0-A14 are declares in header 
const int P12_PIN = A1;//
const int P13_PIN = A2;//
const int P14_PIN = A3;//

const int P21_PIN = A6;
const int P22_PIN = A7;
const int P23_PIN = A8;
const int P24_PIN = A9;
int myLed = 13;


const float scaler = 2.56/4095;
const float biasScaler = 0.6;
int biasP1=0;
int biasP2=0;
int biasP3=0;
int biasP4=0;

void setup()
{            
pinMode(myLed, OUTPUT);
digitalWrite(myLed, HIGH);
  
  f_mount(&fatfs, (TCHAR*)_T("/"), 0); /* Mount/Unmount a logical drive */
  rc = f_open(&fil, (TCHAR*)_T("Pressure_test.bin"), FA_WRITE | FA_CREATE_ALWAYS);
  rc = f_close(&fil);
  Serial.begin(38400);
    
    // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
    adc->setReference(ADC_REF_EXT, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2u88t
    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    // see the documentation for more information
    adc->setAveraging(1);
    adc->setResolution(13);
    adc->setConversionSpeed(ADC_HIGH_SPEED);
    adc->setSamplingSpeed(ADC_HIGH_SPEED);


}

void loop()                     
{
 
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P11_PIN));//-biasP1);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P12_PIN)-biasP2);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P13_PIN)-biasP3);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P14_PIN)-biasP4);
 
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P21_PIN));//-biasP1);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P22_PIN)-biasP2);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P23_PIN)-biasP3);
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P24_PIN)-biasP4);
  Serial.print("analog 1 in volts is: ");
//note the addition of the scaler and the reprinting of analog 1 
  Serial.println(Pressure_buff[Pressure_buff_idx-4]*scaler);
     Serial.print("analog 1 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-4]);
    Serial.print("analog 2 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-3]);
    Serial.print("analog 3 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-2]);
    Serial.print("analog 4 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-1]);
  Serial.println(Pressure_buff_idx);
  
  Serial.print("analog 1 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-8]);
    Serial.print("analog 2 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-7]);
    Serial.print("analog 3 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-6]);
    Serial.print("analog 4 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-5]);
  Serial.println(Pressure_buff_idx);
  digitalWrite(myLed, !digitalRead(myLed));
  
  delay(10);//delay in ms
//if (Pressure_buff_idx<5)
//{
//biasP1=Pressure_buff[Pressure_buff_idx-4]*biasScaler;
//biasP2=Pressure_buff[Pressure_buff_idx-3]*biasScaler; 
//biasP3=Pressure_buff[Pressure_buff_idx-2]*biasScaler;
//biasP4=Pressure_buff[Pressure_buff_idx-1]*biasScaler; 
//delay(2050);
//}

if (Pressure_buff_idx==400)
{
biasP1=0;
biasP2=0;
biasP3=0;
biasP4=0;
 
            Pressure_buff_idx=0;
            rc = f_open(&fil, (TCHAR*)_T("Pressure_test.bin"), FA_WRITE | FA_OPEN_EXISTING);
            //rc = f_open(&fil, wfname, FA_WRITE | FA_OPEN_EXISTING);
            rc = f_lseek(&fil, f_size(&fil));
            memcpy(buffer, (uint8_t*)Pressure_buff, 4*Pressure_BUFFSIZE);
            rc = f_write(&fil, buffer, BUFFSIZE, &wr);
            rc = f_close(&fil);
}
}
  


