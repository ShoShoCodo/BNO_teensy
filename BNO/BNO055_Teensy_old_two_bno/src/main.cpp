/* BNO055_MS5637_t3 Basic Example Code
 Modified version for 3.6 with two BNO055 breakout by:Shoa Russell
 Modified from Teensy 3.2 for 1 BNO055/Pressure breakout by: Kris Winer
 date: October 19, 2014
modified:11_29_16
 Demonstrates basic BNO055 functionality including parameterizing the register addresses,
 initializing the sensor
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Added display functions to allow display to on breadboard monitor.

 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms.
 Can compare results to hardware 9 DoF sensor fusion carried out on the BNO055.
 It uses SDA/SCL on pins 17/16(18,19), respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 All sensors communicate via I2C at 400 Hz or higher.
 SDA and SCL should have external pull-up resistors (to 3.3V).
 47k resistors are on the BNO055 breakout board.

 Hardware setup:
 Breakout Board --------- Teensy 3.6
 3V3 ---------------------- 3.3V
 SDA -----------------------A4/18 *note the smaller pin number goes here
 SCL -----------------------A5/19
 GND ---------------------- GND
 ADD-----------------------22 & 23 *note these are toggled high and low

 data is collected in the form
 ax1,ay1,az1,ax2,ay2,az2,gx1,gy1,gz1,gx2,gy2,gz2,mx1,my1,mz1,mx2,my2,mz2
 where 1 and 2 are the two different BNOs connected
 Note:
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 The Teensy has no internal pullups and we are using the Wire.begin function of the i2c_t3.h library
 to select 400 Hz i2c speed.

 */

#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <arm_math.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <ADC.h>
#include <TimerThree.h>
#include <Adafruit_GFX.h>
#include "BNO055_Teensy.h"
#include "modes.h"
#include <wchar.h>
#include "ff.h"
#include "clocks.h"
#define ACC_BUFFSIZE 3*1000
#define GYR_BUFFSIZE 3*1000
#define MAG_BUFFSIZE 3*1000
#define Pressure_BUFFSIZE 4*1000
#define NUMSAMPLES 1000

float32_t outputbuff[ACC_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t acc_raw_buff[ACC_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t gyr_raw_buff[GYR_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t mag_raw_buff[MAG_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t acc_buff[ACC_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t gyr_buff[GYR_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t mag_buff[MAG_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
float32_t Pressure_buff[Pressure_BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
uint16_t acc_buff_idx = 0;
uint16_t gyr_buff_idx = 0;
uint16_t mag_buff_idx = 0;
uint16_t Pressure_buff_idx = 0;
uint16_t uSD_counter = 0;

// ***** MicroSD Card Stuff ***** //
FRESULT rc;        /* Result code */
FATFS fatfs;      /* File system object */
FIL fil;        /* File object */
DIR dir;        /* Directory object */
FILINFO fno;      /* File information object */
UINT wr;

//Storage buffer (4* because they are float 32 in size )
#define BUFFSIZE 4*ACC_BUFFSIZE+4*GYR_BUFFSIZE+4*MAG_BUFFSIZE+4*Pressure_BUFFSIZE
uint8_t buffer[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );

// ***** Sensor Sampling Stuff ***** //
ADC *adc = new ADC(); // adc object
const int P1_PIN = A0;
const int P2_PIN = A1;
const int P3_PIN = A2;
const int P4_PIN = A3;

// Timer Control
const float scale_timer = 15.98;
const uint32_t TIMER_TS = (uint32_t)(1000000/250.0/scale_timer); // Sampling Period in Microseconds
// ISR Flags
volatile bool a_flag = 0;
volatile bool g_flag = 0;
volatile bool m_flag = 0;
volatile bool p_flag = 0;
//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
// I2C read/write functions for the BNO055 sensor

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
/********************* I2C processes to read single Bytes from BNO *********************/
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}


/********************* I2C processes to read mujltiple bytes from BNO *********************/
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

/********************* read the Accel data**********************/
void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

/********************* read the Gyro data**********************/
void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}
int8_t readGyroTempData()
{
  return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array
}

/********************* read the megnetometer data**********************/
void readMagData(int16_t * destination)//, float32_t * outputbuff)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;

}

void readQuatData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
   // Select BNO055 config mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   // Select page 1 to configure sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
   // Configure ACC
   writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale );
   // Configure GYR
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale );
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
   // Configure MAG
   writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, MPwrMode << 5 | MOpMode << 3 | Modr );

   // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

   // Select BNO055 gyro temperature source
   writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

   // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );

   // Select BNO055 system power mode
   writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );

   // Select BNO055 system operation mode
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
   delay(25);

}

//===================================================================================================================
//====== Calibration functions for the BNO
//===================================================================================================================
void accelgyroCalBNO055(float * dest1, float * dest2)
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  Serial.println("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
  delay(4000);

  // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
   // Select BNO055 system operation mode as AMG for calibration
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG);

 // In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
   writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | AFS_4G );
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    accel_bias[0]  += (int32_t) accel_temp[0];
    accel_bias[1]  += (int32_t) accel_temp[1];
    accel_bias[2]  += (int32_t) accel_temp[2];
    delay(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
   }
    accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
    accel_bias[1]  /= (int32_t) sample_count;
    accel_bias[2]  /= (int32_t) sample_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) 1000;}

    dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
    dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
    dest1[2] = (float) accel_bias[2];

 // In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | GFS_2000DPS );
   writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
   for(ii = 0; ii < sample_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
    delay(35);  // at 32 Hz ODR, new gyro data available every 31 ms
   }
    gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
    gyro_bias[1]  /= (int32_t) sample_count;
    gyro_bias[2]  /= (int32_t) sample_count;

    dest2[0] = (float) gyro_bias[0]/16.;  // save gyro biases in dps for use in main program
    dest2[1] = (float) gyro_bias[1]/16.;  // gyro data is 16 LSB/dps
    dest2[2] = (float) gyro_bias[2]/16.;

  // Return to config mode to write accelerometer biases in offset register
  // This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);

  //write biases to accelerometer offset registers ad 16 LSB/dps
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
//  Serial.println("Average accelerometer bias = ");
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

   //write biases to gyro offset registers
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);

  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

 // Check that offsets were properly written to offset registers
//  Serial.println("Average gyro bias = ");
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

   Serial.println("Accel/Gyro Calibration done!");
}

void magCalBNO055(float * dest1)
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
   // Select BNO055 system operation mode as AMG for calibration
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
   delay(25);
   writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG );

 // In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if (ii == 0) {
        mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar!
        mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
      } else {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

 //   Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
 //   Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
 //   Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
    dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
    dest1[2] = (float) mag_bias[2] / 1.6;

  // Return to config mode to write mag biases in offset register
  // This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);

  //write biases to magnetometer offset registers as 16 LSB/microTesla
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
//  Serial.println("Average magnetometer bias = ");
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB)));
//  Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

   Serial.println("Mag Calibration done!");
}

//===================================================================================================================
//====== I2C scanning functions
//===================================================================================================================
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}




// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void Madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

//this is where the mahony function was before moving it
void Mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
  {
      float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
      float norm;
      float hx, hy, bx, bz;
      float vx, vy, vz, wx, wy, wz;
      float ex, ey, ez;
      float pa, pb, pc;

      // Auxiliary variables to avoid repeated arithmetic
      float q1q1 = q1 * q1;
      float q1q2 = q1 * q2;
      float q1q3 = q1 * q3;
      float q1q4 = q1 * q4;
      float q2q2 = q2 * q2;
      float q2q3 = q2 * q3;
      float q2q4 = q2 * q4;
      float q3q3 = q3 * q3;
      float q3q4 = q3 * q4;
      float q4q4 = q4 * q4;

      // Normalise accelerometer measurement
      norm = sqrt(ax * ax + ay * ay + az * az);
      if (norm == 0.0f) return; // handle NaN
      norm = 1.0f / norm;        // use reciprocal for division
      ax *= norm;
      ay *= norm;
      az *= norm;

      // Normalise magnetometer measurement
      norm = sqrt(mx * mx + my * my + mz * mz);
      if (norm == 0.0f) return; // handle NaN
      norm = 1.0f / norm;        // use reciprocal for division
      mx *= norm;
      my *= norm;
      mz *= norm;

      // Reference direction of Earth's magnetic field
      hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
      hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
      bx = sqrt((hx * hx) + (hy * hy));
      bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

      // Estimated direction of gravity and magnetic field
      vx = 2.0f * (q2q4 - q1q3);
      vy = 2.0f * (q1q2 + q3q4);
      vz = q1q1 - q2q2 - q3q3 + q4q4;
      wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
      wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
      wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

      // Error is cross product between estimated direction and measured direction of gravity
      ex = (ay * vz - az * vy) + (my * wz - mz * wy);
      ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
      ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
      if (Ki > 0.0f)
      {
          eInt[0] += ex;      // accumulate integral error
          eInt[1] += ey;
          eInt[2] += ez;
      }
      else
      {
          eInt[0] = 0.0f;     // prevent integral wind up
          eInt[1] = 0.0f;
          eInt[2] = 0.0f;
      }

      // Apply feedback terms
      gx = gx + Kp * ex + Ki * eInt[0];
      gy = gy + Kp * ey + Ki * eInt[1];
      gz = gz + Kp * ez + Ki * eInt[2];

      // Integrate rate of change of quaternion
      pa = q2;
      pb = q3;
      pc = q4;
      q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
      q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
      q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
      q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

      // Normalise quaternion
      norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
      norm = 1.0f / norm;
      q[0] = q1 * norm;
      q[1] = q2 * norm;
      q[2] = q3 * norm;
      q[3] = q4 * norm;

  }

  //===================================================================================================================
  //====== Timer interrupt service routine
  //===================================================================================================================

void TIMER_TS_ISR(void) {
    a_flag = true;
    g_flag = true;
    m_flag = true;
    p_flag = true;
    //write pins out here to check that the time is actually what we want
}

//===================================================================================================================
//====== SETUP LOOP Begin of actual code
//===================================================================================================================

void setup()
{
    f_mount(&fatfs, (TCHAR*)_T("/"), 0); /* Mount/Unmount a logical drive */

  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(1000);
  Serial.begin(38400);
  Serial.println("serial passed");

  // ADC setup for pressure sensors
  adc->setAveraging(1);
  adc->setResolution(13);
  adc->setConversionSpeed(ADC_HIGH_SPEED);
  adc->setSamplingSpeed(ADC_HIGH_SPEED);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(BNO1, OUTPUT);
  pinMode(BNO2, OUTPUT);
  digitalWrite(BNO1, HIGH);
  digitalWrite(BNO2, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  rc = f_open(&fil, (TCHAR*)_T("BNO_test.bin"), FA_WRITE | FA_CREATE_ALWAYS);
  rc = f_close(&fil);
    Timer3.initialize(TIMER_TS);
    Timer3.attachInterrupt(TIMER_TS_ISR);
for (int BNOcali_count=1;BNOcali_count <= 2;BNOcali_count++)
{
  I2Cscan(); // check for I2C devices on the bus8

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BNO055 9-axis motion sensor...");
  byte c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055
  Serial.print("BNO055 Address = 0x");
  Serial.println(BNO055_ADDRESS, HEX);
  Serial.print("BNO055 WHO_AM_I = 0x");
  Serial.println(BNO055_CHIP_ID, HEX);
  Serial.print("BNO055 ");
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.println(" I should be 0xA0");

    // Read the WHO_AM_I register of the accelerometer, this is a good test of communication
  byte d = readByte(BNO055_ADDRESS, BNO055_ACC_ID);  // Read WHO_AM_I register for accelerometer
  Serial.print("BNO055 ACC ");
  Serial.print("I AM ");
  Serial.print(d, HEX);
  Serial.println(" I should be 0xFB");

  delay(100);

  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte e = readByte(BNO055_ADDRESS, BNO055_MAG_ID);  // Read WHO_AM_I register for magnetometer
  Serial.print("BNO055 MAG ");
  Serial.print("I AM ");
  Serial.print(e, HEX);
  Serial.println(" I should be 0x32");

  delay(100);

  // Read the WHO_AM_I register of the gyroscope, this is a good test of communication
  byte f = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL
  Serial.print("BNO055 GYRO ");
  Serial.print("I AM ");
  Serial.print(f, HEX);
  Serial.println(" I should be 0x0F");

  delay(2000);

  if (c == 0xA0) // BNO055 WHO_AM_I should always be 0xA0
  {
    Serial.println("BNO055 is online...");

    // Check self-test results
    byte selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);

    if(selftest & 0x01) {
      Serial.println("accelerometer passed selftest");
    } else {
      Serial.println("accelerometer failed selftest");
    }
    if(selftest & 0x02) {
      Serial.println("magnetometer passed selftest");
    } else {
      Serial.println("magnetometer failed selftest");
    }
    if(selftest & 0x04) {
      Serial.println("gyroscope passed selftest");
    } else {
      Serial.println("gyroscope failed selftest");
    }
    if(selftest & 0x08) {
      Serial.println("MCU passed selftest");
    } else {
      Serial.println("MCU failed selftest");
    }


//implement calibration
  // delay(1000);
  // accelgyroCalBNO055(accelBias, gyroBias);
  //
  // Serial.println("Average accelerometer bias (mg) = ");
  // Serial.println(accelBias[0]);
  // Serial.println(accelBias[1]);
  // Serial.println(accelBias[2]);
  // Serial.println("Average gyro bias (dps) = ");
  // Serial.println(gyroBias[0]);
  // Serial.println(gyroBias[1]);
  // Serial.println(gyroBias[2]);
  //
  // delay(100);
  //
  // magCalBNO055(magBias);
  //
  // Serial.println("Average magnetometer bias (mG) = ");
  // Serial.println(magBias[0]);
  // Serial.println(magBias[1]);
  // Serial.println(magBias[2]);
  //
  // delay(100);

  // Check calibration status of the sensors
  uint8_t calstat = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
  Serial.println("Not calibrated = 0, fully calibrated = 3");
  Serial.print("System calibration status ");
  Serial.println( (0xC0 & calstat) >> 6);
  Serial.print("Gyro   calibration status ");
  Serial.println( (0x30 & calstat) >> 4);
  Serial.print("Accel  calibration status ");
  Serial.println( (0x0C & calstat) >> 2);
  Serial.print("Mag    calibration status ");
  Serial.println( (0x03 & calstat) >> 0);

  initBNO055(); // Initialize the BNO055
  Serial.println("BNO055 initialized for sensor mode...."); // Initialize BNO055 for sensor read

  }
  else
  {
    Serial.print("Could not connect to BNO055: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  digitalWrite(BNO1, !digitalRead(BNO1));
  digitalWrite(BNO2, !digitalRead(BNO2));
  delay(500); //toggles the addresses of the BNOs to calibrate the second

}
   //ends the for loop to calibrate both sensors
}


//===================================================================================================================
//====== LOOP FUNCTION
//===================================================================================================================
void loop()
{
 while(true){

  noInterrupts();
if (a_flag){
    readAccelData(accelCount);//, acc_buff);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual mg's
    ax = (float)accelCount[0]; // - accelBias[0];  // subtract off calculated accel bias
    ay = (float)accelCount[1]; // - accelBias[1];
    az = (float)accelCount[2]; // - accelBias[2];

  digitalWrite(BNO1, !digitalRead(BNO1));
  digitalWrite(BNO2, !digitalRead(BNO2));
//  delay(500); //toggles the addresses of the BNOs to calibrate the second

    readAccelData(accelCount);//, acc_buff);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual mg's
    ax2 = (float)accelCount[0]; // - accelBias[0];  // subtract off calculated accel bias
    ay2 = (float)accelCount[1]; // - accelBias[1];
    az2 = (float)accelCount[2]; // - accelBias[2];

            acc_buff[acc_buff_idx+0] = ax;
            acc_buff[acc_buff_idx+1] = ay;
            acc_buff[acc_buff_idx+2] = az;
            acc_buff[acc_buff_idx+3] = ax2;
            acc_buff[acc_buff_idx+4] = ay2;
            acc_buff[acc_buff_idx+5] = az2;
            acc_buff_idx += 6;
            }

   if (g_flag){
    readGyroData(gyroCount);//, gyr_raw_buff);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
    gy = (float)gyroCount[1]/16.; // - gyroBias[1];
    gz = (float)gyroCount[2]/16.; // - gyroBias[2];

  digitalWrite(BNO1, !digitalRead(BNO1));
  digitalWrite(BNO2, !digitalRead(BNO2));
  //delay(500); //toggles the addresses of the BNOs to calibrate the second

    readGyroData(gyroCount);//, gyr_raw_buff);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx2 = (float)gyroCount[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
    gy2 = (float)gyroCount[1]/16.; // - gyroBias[1];
    gz2 = (float)gyroCount[2]/16.; // - gyroBias[2];

            gyr_buff[gyr_buff_idx+0] = gx;
            gyr_buff[gyr_buff_idx+1] = gy;
            gyr_buff[gyr_buff_idx+2] = gz;
            gyr_buff[gyr_buff_idx+3] = gx2;
            gyr_buff[gyr_buff_idx+4] = gy2;
            gyr_buff[gyr_buff_idx+5] = gz2;
            gyr_buff_idx += 6;

            }
            if (m_flag){
    readMagData(magCount);//, mag_raw_buff);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    mx = (float)magCount[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss
    my = (float)magCount[1]/1.6; // - magBias[1];
    mz = (float)magCount[2]/1.6; // - magBias[2];

  digitalWrite(BNO1, !digitalRead(BNO1));
  digitalWrite(BNO2, !digitalRead(BNO2));
//  delay(500); //toggles the addresses of the BNOs to calibrate the second

      readMagData(magCount);//, mag_raw_buff);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    mx2 = (float)magCount[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss
    my2 = (float)magCount[1]/1.6; // - magBias[1];
    mz2 = (float)magCount[2]/1.6; // - magBias[2];

            mag_buff[mag_buff_idx+0] = mx;
            mag_buff[mag_buff_idx+1] = my;
            mag_buff[mag_buff_idx+2] = mz;
            mag_buff[mag_buff_idx+3] = mx2;
            mag_buff[mag_buff_idx+4] = my2;
            mag_buff[mag_buff_idx+5] = mz2;
            mag_buff_idx += 6;
            }
if(p_flag){
Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P1_PIN));
Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P2_PIN));
Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P3_PIN));
Pressure_buff[Pressure_buff_idx++] = (float32_t)(adc->adc0->analogRead(P4_PIN));
 // Serial.print("analog 1 is: ");
 // Serial.println(Pressure_buff[Pressure_buff_idx-4]);
 //   Serial.print("analog 2 is: ");
 // Serial.println(Pressure_buff[Pressure_buff_idx-3]);
 //   Serial.print("analog 3 is: ");
 // Serial.println(Pressure_buff[Pressure_buff_idx-2]);
 //   Serial.print("analog 4 is: ");
 // Serial.println(Pressure_buff[Pressure_buff_idx-1]);
// delay(550);
}
/*store the data to the SD card if the timer is up*/
        if (acc_buff_idx%NUMSAMPLES == 0) {
              acc_buff_idx = 0;
              gyr_buff_idx = 0;
              mag_buff_idx = 0;
              Pressure_buff_idx = 0;
             rc = f_open(&fil, (TCHAR*)_T("BNO_test.bin"), FA_WRITE | FA_OPEN_EXISTING);
            //rc = f_open(&fil, wfname, FA_WRITE | FA_OPEN_EXISTING);
            rc = f_lseek(&fil, f_size(&fil));

            memcpy(buffer, (uint8_t*)acc_buff, 4*ACC_BUFFSIZE);
            memcpy(buffer+4*ACC_BUFFSIZE, (uint8_t*)gyr_buff, 4*GYR_BUFFSIZE);
            memcpy(buffer+4*ACC_BUFFSIZE+4*GYR_BUFFSIZE, (uint8_t*)mag_buff, 4*MAG_BUFFSIZE);
            memcpy(buffer+4*ACC_BUFFSIZE+4*GYR_BUFFSIZE, (uint8_t*)Pressure_buff, 4*Pressure_BUFFSIZE);
            rc = f_write(&fil, buffer, BUFFSIZE, &wr);
            rc = f_close(&fil);

        }


    readQuatData(quatCount);  // Read the x/y/z adc values
    // Calculate the quaternion values
    quat[0] = (float)(quatCount[0])/16384.;
    quat[1] = (float)(quatCount[1])/16384.;
    quat[2] = (float)(quatCount[2])/16384.;
    quat[3] = (float)(quatCount[3])/16384.;

    readEulData(EulCount);  // Read the x/y/z adc values
    // Calculate the Euler angles values in degrees
    Yaw = (float)EulCount[0]/16.;
    Roll = (float)EulCount[1]/16.;
    Pitch = (float)EulCount[2]/16.;

    readLIAData(LIACount);  // Read the x/y/z adc values
    // Calculate the linear acceleration (sans gravity) values in mg
    LIAx = (float)LIACount[0];
    LIAy = (float)LIACount[1];
    LIAz = (float)LIACount[2];

    readGRVData(GRVCount);  // Read the x/y/z adc values
    // Calculate the linear acceleration (sans gravity) values in mg
    GRVx = (float)GRVCount[0];
    GRVy = (float)GRVCount[1];
    GRVz = (float)GRVCount[2];

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  // Sensors x, y, and z-axes  for the three sensor: accel, gyro, and magnetometer are all aligned, so
  // no allowance for any orientation mismatch in feeding the output to the quaternion filter is required.
  // For the BNO055, the sensor forward is along the x-axis just like
  //This rotation can be modified to allow any convenient orientation convention.
  // Pass gyro rate as rad/s
Madgwick(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my,  mz);
//Mahony(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

       // check BNO-055 error status at 2 Hz rate
    uint8_t sysstat = readByte(BNO055_ADDRESS, BNO055_SYS_STATUS); // check system status
    Serial.print("System Status = 0x");
    Serial.println(sysstat, HEX);
    if(sysstat == 0x05) Serial.println("Sensor fusion algorithm running");
    if(sysstat == 0x06) Serial.println("Sensor fusion not algorithm running");

    if(SerialDebug) {
    Serial.print("ax = ");
    Serial.print((int)ax);
    Serial.print(" ay = ");
    Serial.print((int)ay);
    Serial.print(" az = ");
    Serial.print((int)az);
    Serial.println(" mg");
    Serial.print("gx = ");
    Serial.print( gx, 2);
    Serial.print(" gy = ");
    Serial.print( gy, 2);
    Serial.print(" gz = ");
    Serial.print( gz, 2);
    Serial.println(" deg/s");
    Serial.print("mx = ");
    Serial.print( (int)mx );
    Serial.print(" my = ");
    Serial.print( (int)my );
    Serial.print(" mz = ");
    Serial.print( (int)mz );
    Serial.println(" mG");
    }


  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.

    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
 //   yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
//
//    if(SerialDebug) {
//    Serial.print("Software Yaw, Pitch, Roll: ");
//    Serial.print(yaw, 2);
//    Serial.print(", ");
//    Serial.print(pitch, 2);
//    Serial.print(", ");
//    Serial.println(roll, 2);
//
//    Serial.print("Hardware Yaw, Pitch, Roll: ");
//    Serial.print(Yaw, 2);
//    Serial.print(", ");
//    Serial.print(Pitch, 2);
//    Serial.print(", ");
//    Serial.println(Roll, 2);
//
//    Serial.print("Hardware x, y, z linear acceleration: ");
//    Serial.print(LIAx, 2);
//    Serial.print(", ");
//    Serial.print(LIAy, 2);
//    Serial.print(", ");
//    Serial.println(LIAz, 2);
//
//    Serial.print("Hardware x, y, z gravity vector: ");
//    Serial.print(GRVx, 2);
//    Serial.print(", ");
//    Serial.print(GRVy, 2);
//    Serial.print(", ");
//    Serial.println(GRVz, 2);
//

    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    //}

    digitalWrite(myLed, !digitalRead(myLed));
    count = millis();
    sumCount = 0;
    sum = 0;
    }
    interrupts(); //allows the timer interrupt to trigger
}//ends while
}//ends loop
