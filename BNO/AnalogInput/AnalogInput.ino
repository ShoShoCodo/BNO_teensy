/* Analog Input Example, Teensyduino Tutorial #4
   http://www.pjrc.com/teensy/tutorial4.html

   After uploading this to your board, use Serial Monitor
   to view the message.  When Serial is selected from the
   Tools > USB Type menu, the correct serial port must be
   selected from the Tools > Serial Port AFTER Teensy is
   running this code.  Teensy only becomes a serial device
   while this code is running!  For non-Serial types,
   the Serial port is emulated, so no port needs to be
   selected.

   This example code is in the public domain.
*/
#include <ADC.h>
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <arm_math.h>

#define Pressure_BUFFSIZE 1000
float32_t Pressure_buff[Pressure_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
uint16_t Pressure_buff_idx = 0;


// ***** Sensor Sampling Stuff ***** //
ADC *adc = new ADC(); // adc object
const int P1_PIN = A0;
const int P2_PIN = A1;
const int P3_PIN = A2;
const int P4_PIN = A3;


void setup()
{                
  Serial.begin(38400);
    adc->setAveraging(1);
    adc->setResolution(13);
    adc->setConversionSpeed(ADC_HIGH_SPEED);
    adc->setSamplingSpeed(ADC_HIGH_SPEED);
  
}

int val;

void loop()                     
{
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P1_PIN));
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P2_PIN));
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P3_PIN));
 Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P4_PIN));
  Serial.print("analog 1 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-4]);
    Serial.print("analog 2 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-3]);
    Serial.print("analog 3 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-2]);
    Serial.print("analog 4 is: ");
  Serial.println(Pressure_buff[Pressure_buff_idx-1]);
  delay(550);
}

