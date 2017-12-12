#ifndef LSM9DS1_cpp
#define LSM9DS1_cpp
#include<LSM9DS1.h>
#define I2C_NOSTOP true
#define Serial SerialUSB
//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

LSM9DS1::LSM9DS1() {
	Serial.begin(38400);
	Wire.begin();
}

void LSM9DS1::init() {
	// Read the WHO_AM_I registers, this is a good test of communication
	Serial.println("LSM9DS1 9-axis motion sensor...");
	byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 accel/gyro
	Serial.print("LSM9DS1 accel/gyro"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x68, HEX);
	byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 magnetometer
	Serial.print("LSM9DS1 magnetometer"); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x3D, HEX);

	if (c == 0x68 && d == 0x3D) // WHO_AM_I should always be 0x0E for the accel/gyro and 0x3C for the mag
	{
		Serial.println("LSM9DS1 is online...");

		// get sensor resolutions, only need to do this once
		getAres();
		getGres();
		getMres();
		Serial.print("accel sensitivity is "); Serial.print(1./(1000.*aRes)); Serial.println(" LSB/mg");
		Serial.print("gyro sensitivity is "); Serial.print(1./(1000.*gRes)); Serial.println(" LSB/mdps");
		Serial.print("mag sensitivity is "); Serial.print(1./(1000.*mRes)); Serial.println(" LSB/mGauss");

		Serial.println("Perform gyro and accel self test");
		selftestLSM9DS1(); // check function of gyro and accelerometer via self test

		Serial.println(" Calibrate gyro and accel");
		accelgyrocalLSM9DS1(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
		Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

		//magcalLSM9DS1(magBias);
		//Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]);
		//delay(2000); // add delay to see results before serial spew of data

		initLSM9DS1();
		Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
	}
	else
	{
		Serial.print("Could not connect to LSM9DS1: 0x");
		Serial.println(c, HEX);
		while(1) ; // Loop forever if communication doesn't happen
	}
}

LSM9DS1_data LSM9DS1::capture() {
	LSM9DS1_data lsm9ds1_data = {
		0,0,0,
		0,0,0,
		0,0,0,
		0,0,0,0,
		0.0,0.0,0.0,
		0.0,0.0,0.0
	};

	if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) {  // check if new accel data is ready
		readAccelData(accelCount);  // Read the x/y/z adc values

		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float)accelCount[1]*aRes - accelBias[1];
		az = (float)accelCount[2]*aRes - accelBias[2];
	}

	if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) {  // check if new gyro data is ready
		readGyroData(gyroCount);  // Read the x/y/z adc values

		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1]*gRes - gyroBias[1];
		gz = (float)gyroCount[2]*gRes - gyroBias[2];
	}

	if (readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M) & 0x08) {  // check if new mag data is ready
		readMagData(magCount);  // Read the x/y/z adc values

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float)magCount[0]*mRes  - magBias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1]*mRes  - magBias[1];
		mz = (float)magCount[2]*mRes  - magBias[2];
	}

	Now = micros();
	deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	// Sensors x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer
	// the magnetometer z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the magnetometer
	// x-axis is aligned with the -x axis of the gyro and the magnetometer y axis is aligned with the y axis of the gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	// This is ok by aircraft orientation standards!
	// Pass gyro rate as rad/s
	//MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  -mx,  my, mz);
	//MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, my, mz);

	// Serial print and/or display at 0.5 s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 500) { // update LCD once per half-second independent of read rate

		if(SerialDebug) {
			//Serial.print("ax = "); Serial.print((int)1000*ax);
			//Serial.print(" ay = "); Serial.print((int)1000*ay);
			//Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
			//Serial.print("gx = "); Serial.print( gx, 2);
			//Serial.print(" gy = "); Serial.print( gy, 2);
			//Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
			//Serial.print("mx = "); Serial.print( (int)1000*mx );
			//Serial.print(" my = "); Serial.print( (int)1000*my );
			//Serial.print(" mz = "); Serial.print( (int)1000*mz ); Serial.println(" mG");

			//Serial.print("q0 = "); Serial.print(q[0]);

			//Serial.print(" qx = "); Serial.print(q[1]);
			//Serial.print(" qy = "); Serial.print(q[2]);
			//Serial.print(" qz = "); Serial.println(q[3]);
			char info[128];
			sprintf(info, "A:\t%.2f\t%.2f\t%.2f \tG:\t%.2f\t%.2f\t%.2f\r\n", 1000*ax,1000*ay,1000*az,1000*gx,1000*gy,1000*gz);
			Serial.print(info);
			//Serial.print("A ");Serial.print((int)1000*ax);Serial.print("\t");Serial.print((int)1000*ay);Serial.print("\t");Serial.print((int)1000*az,1);Serial.print("\t\t");
			//Serial.print("G ");Serial.print((int)1000*gx,3);Serial.print("\t");Serial.print((int)1000*gy);Serial.print("\t");Serial.print((int)1000*gz);Serial.print("\t\t");
			//Serial.print("M ");Serial.print((int)1000*mx);Serial.print("\t");Serial.print((int)1000*my);Serial.print("\t");Serial.print((int)1000*mz);Serial.print("\t\t");
			//Serial.print("Q "); Serial.print(q[0]);Serial.print("\t");Serial.print(q[1]);Serial.print("\t");Serial.print(q[2]);Serial.print("\t");Serial.println(q[3]);
		}

		tempCount = readTempData();  // Read the gyro adc values
		temperature = ((float) tempCount/256. + 25.0); // Gyro chip temperature in degrees Centigrade
		// Print temperature in degrees Centigrade
		//Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C

		if(SerialDebug) {
			//Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
			//Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
			//Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
			//Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
		}

		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI;
		//yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		yaw -= 4.36f;
		roll  *= 180.0f / PI;

		if(SerialDebug) {
			//Serial.print("Yaw, Pitch, Roll: ");
			//Serial.print(yaw, 2);
			//Serial.print(", ");
			//Serial.print(pitch, 2);
			//Serial.print(", ");
			//Serial.println(roll, 2);

			//Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
		}

		// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
		// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
		// The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
		// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
		// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
		// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
		// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
		// This filter update rate should be fast enough to maintain accurate platform orientation for
		// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
		// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
		// The 3.3 V 8 MHz Pro Mini is doing pretty well!

		count = millis();
		sumCount = 0;
		sum = 0;
	}

	lsm9ds1_data = (LSM9DS1_data){
		ax, ay, az,
		gx, gy, gz,
		mx, my, mz,
		q[0], q[1], q[2], q[3],
		altitude, Pressure, temperature,
		pitch, yaw, roll
	};

	return lsm9ds1_data;
}

void LSM9DS1::getMres() {
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
		case MFS_4G:
			mRes = 4.0/32768.0;
			break;
		case MFS_8G:
			mRes = 8.0/32768.0;
			break;
		case MFS_12G:
			mRes = 12.0/32768.0;
			break;
		case MFS_16G:
			mRes = 16.0/32768.0;
			break;
	}
}

void LSM9DS1::getGres() {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 245 DPS (00), 500 DPS (01), and 2000 DPS  (11).
		case GFS_245DPS:
			gRes = 245.0/32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0/32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0/32768.0;
			break;
	}
}

void LSM9DS1::getAres() {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11).
		case AFS_2G:
			aRes = 2.0/32768.0;
			break;
		case AFS_16G:
			aRes = 16.0/32768.0;
			break;
		case AFS_4G:
			aRes = 4.0/32768.0;
			break;
		case AFS_8G:
			aRes = 8.0/32768.0;
			break;
	}
}


void LSM9DS1::readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


void LSM9DS1::readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void LSM9DS1::readMagData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int16_t LSM9DS1::readTempData()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}

void LSM9DS1::initLSM9DS1()
{
	// enable the 3-axes of the gyroscope
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
	// configure the gyroscope
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
	delay(200);
	// enable the three axes of the accelerometer
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
	// configure the accelerometer-specify bandwidth selection with Abw
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
	delay(200);
	// enable block data update, allow auto-increment during multiple byte read
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
	// configure the magnetometer-enable temperature compensation of mag data
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
}

void LSM9DS1::selftestLSM9DS1()
{
	float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
	float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
	accelgyrocalLSM9DS1(gyro_noST, accel_noST);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
	accelgyrocalLSM9DS1(gyro_ST, accel_ST);

	float gyrodx = (gyro_ST[0] - gyro_noST[0]);
	float gyrody = (gyro_ST[1] - gyro_noST[1]);
	float gyrodz = (gyro_ST[2] - gyro_noST[2]);

	Serial.println("Gyro self-test results: ");
	Serial.print("x-axis = "); Serial.print(gyrodx); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
	Serial.print("y-axis = "); Serial.print(gyrody); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
	Serial.print("z-axis = "); Serial.print(gyrodz); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");

	float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
	float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
	float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

	Serial.println("Accelerometer self-test results: ");
	Serial.print("x-axis = "); Serial.print(accdx); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
	Serial.print("y-axis = "); Serial.print(accdy); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
	Serial.print("z-axis = "); Serial.print(accdz); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");

	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
	delay(200);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void LSM9DS1::accelgyrocalLSM9DS1(float * dest1, float * dest2)
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	uint16_t samples, ii;

	// enable the 3-axes of the gyroscope
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
	// configure the gyroscope
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
	delay(200);
	// enable the three axes of the accelerometer
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
	// configure the accelerometer-specify bandwidth selection with Abw
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
	delay(200);
	// enable block data update, allow auto-increment during multiple byte read
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

	// First get gyro bias
	byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO
	delay(50);                                                       // Wait for change to take effect
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

	for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
		int16_t gyro_temp[3] = {0, 0, 0};
		readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
		gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
		gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
		gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

		gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];
	}

	gyro_bias[0] /= samples; // average the data
	gyro_bias[1] /= samples;
	gyro_bias[2] /= samples;

	dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
	dest1[1] = (float)gyro_bias[1]*gRes;
	dest1[2] = (float)gyro_bias[2]*gRes;

	c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO
	delay(50);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

	// now get the accelerometer bias
	c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO
	delay(50);                                                       // Wait for change to take effect
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

	for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
		int16_t accel_temp[3] = {0, 0, 0};
		readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
		accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
		accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
	}

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples;
	accel_bias[2] /= samples;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) (1.0/aRes);}

	dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
	dest2[1] = (float)accel_bias[1]*aRes;
	dest2[2] = (float)accel_bias[2]*aRes;

	c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO
	delay(50);
	writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void LSM9DS1::magcalLSM9DS1(float * dest1)
{
	uint8_t data[6]; // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

	// configure the magnetometer-enable temperature compensation of mag data
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);

	sample_count = 128;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = {0, 0, 0};
		readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
		mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	}

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1]*mRes;
	dest1[2] = (float) mag_bias[2]*mRes;

	//write biases to accelerometermagnetometer offset registers as counts);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
	writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);

	Serial.println("Mag Calibration done!");
}

// I2C read/write functions for the LSM9DS1and AK8963 sensors

void LSM9DS1::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void LSM9DS1::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
	while (Wire.available()) {
		dest[i++] = Wire.read();          // Put read results in the Rx buffer
	}
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void LSM9DS1::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
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

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void LSM9DS1::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
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
#endif
