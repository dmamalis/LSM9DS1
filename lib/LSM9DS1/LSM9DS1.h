#ifndef LSM9DS1_h
#define LSM9DS1_h
/* LSM9DS1_MS5611_t3 Basic Example Code
 by: Kris Winer
 date: November 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic LSM9DS1 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 This sketch is intended specifically for the LSM9DS1+MS5611 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5611 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption od 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the LSM9DS1+MS5611 Teensy 3.1 add-on shield/breakout board.

 Hardware setup:
 LSM9DS1Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The LSM9DS1 is an I2C sensor and can use the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using either a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include "Wire.h"
#include <stdio.h>

//#include <i2c_t3.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See MS5611-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
#define MS5611_RESET      0x1E
#define MS5611_CONVERT_D1 0x40
#define MS5611_CONVERT_D2 0x50
#define MS5611_ADC_READ   0x00

// See also LSM9DS1 Register Map and Descriptions, http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00103319.pdf
//
// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS	    0x04
#define LSM9DS1XG_ACT_DUR	    0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
//#define LSM9DS1XG_STATUS_REG        0x27 // duplicate of 0x17!
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
//
// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

// Using the LSM9DS1+MS5611 Teensy 3.1 Add-On shield, ADO is set to 1
// Seven-bit device address of accel/gyro is 110101 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define LSM9DS1XG_ADDRESS 0x6B  //  Device address when ADO = 1
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define MS5611_ADDRESS    0x77  //  Address of altimeter
#else
#define LSM9DS1XG_ADDRESS 0x6A   //  Device address when ADO = 0
#define LSM9DS1M_ADDRESS  0x1D   //  Address of magnetometer
#define MS5611_ADDRESS    0x77   //  Address of altimeter
#endif

#ifndef SerialDebug
#define SerialDebug true  // set to true to get Serial output for debugging
#endif

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_D1   0x40
#define ADC_D2   0x50
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
	AFS_2G = 0,
	AFS_16G,
	AFS_4G,
	AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
	AODR_PowerDown = 0,
	AODR_10Hz,
	AODR_50Hz,
	AODR_119Hz,
	AODR_238Hz,
	AODR_476Hz,
	AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
	ABW_408Hz = 0,
	ABW_211Hz,
	ABW_105Hz,
	ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
	GFS_245DPS = 0,
	GFS_500DPS,
	GFS_NoOp,
	GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
	GODR_PowerDown = 0,
	GODR_14_9Hz,
	GODR_59_5Hz,
	GODR_119Hz,
	GODR_238Hz,
	GODR_476Hz,
	GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
	GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
	GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
	GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
	GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
	MFS_4G = 0,
	MFS_8G,
	MFS_12G,
	MFS_16G
};

enum Mmode {
	MMode_LowPower = 0,
	MMode_MedPerformance,
	MMode_HighPerformance,
	MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
	MODR_0_625Hz = 0,
	MODR_1_25Hz,
	MODR_2_5Hz,
	MODR_5Hz,
	MODR_10Hz,
	MODR_20Hz,
	MODR_80Hz
};

struct LSM9DS1_data
{
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	float q0, qx, qy, qz;
	float altitude;
	double temperature, pressure;
	float pitch, yaw, roll;
};

class LSM9DS1
{
	public:
		LSM9DS1();
		void init();
		LSM9DS1_data capture();

	private:
		float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
		float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
		uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
		float pitch, yaw, roll;
		float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
		uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
		uint32_t Now = 0;                         // used to calculate integration interval
		float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

		uint16_t Pcal[8];         // calibration constants from MS5611 PROM registers
		unsigned char nCRC;       // calculated check sum to ensure PROM integrity
		uint32_t D1 = 0, D2 = 0;  // raw MS5611 pressure and temperature data
		double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
		int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
		float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
		int16_t tempCount;            // temperature raw count output
		float   altitude, temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius
		double Temperature, Pressure; // stores MS5611 pressures sensor pressure and temperature

		// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
		float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
		float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
		// There is a tradeoff in the beta parameter between accuracy and response speed.
		// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
		// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
		// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
		// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
		// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
		// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
		// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
		float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
		float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

		float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
		// Specify sensor full scale
		uint8_t OSR = ADC_4096;      // set pressure amd temperature oversample rate
		uint8_t Gscale = GFS_245DPS; // gyro full scale
		uint8_t Godr = GODR_238Hz;   // gyro data sample rate
		uint8_t Gbw = GBW_med;       // gyro data bandwidth
		uint8_t Ascale = AFS_2G;     // accel full scale
		uint8_t Aodr = AODR_238Hz;   // accel data sample rate
		uint8_t Abw = ABW_50Hz;      // accel data bandwidth
		uint8_t Mscale = MFS_4G;     // mag full scale
		uint8_t Modr = MODR_10Hz;    // mag data sample rate
		uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode

		void getMres();
		void getGres();
		void getAres();
		void readAccelData(int16_t * destination);
		void readGyroData(int16_t * destination);
		void readMagData(int16_t * destination);
		int16_t readTempData();
		void initLSM9DS1();
		void selftestLSM9DS1();
		void accelgyrocalLSM9DS1(float * dest1, float * dest2);
		void magcalLSM9DS1(float * dest1);
		void MS5611Reset();
		void MS5611PromRead(uint16_t * destination);
		uint32_t MS5611Read(uint8_t CMD, uint8_t OSR);
		unsigned char MS5611checkCRC(uint16_t * n_prom);
		void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
		uint8_t readByte(uint8_t address, uint8_t subAddress);
		void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
		void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
};
#endif
