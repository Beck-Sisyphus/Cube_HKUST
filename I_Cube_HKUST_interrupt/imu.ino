// Beck Pang, Nov. 21st, 2016
// Reading raw data out of DMP of IMU

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


// In the setup, test if DMP is enabled and attach interrupt for data reading
bool test_dmp_connection(MPU6050 mpu, uint16_t &packetSize) {
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	bool dmpReady;

	// load and configure the DMP
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else
	{
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		Serial.print("DMP Initialization failed (code ");
		Serial.print(devStatus);
		Serial.println(")");
		dmpReady = false;
	}
	return dmpReady;
}

void process_mpu_data(MPU6050 mpu, uint16_t &packetSize, uint16_t &fifoCount, int number, VectorInt16 *aa, int32_t *gyro) {
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	mpuIntStatus = mpu.getIntStatus();
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		// Serial.print("FIFO overflow! mpu");
    // Serial.println(number);

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// TODO: compare the result from DMP to two IMU readings
		mpu.dmpGetGyro(gyro, fifoBuffer);

		mpu.dmpGetAccel(aa, fifoBuffer);

		#ifdef OUTPUT_READABLE_WORLDACCEL
			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			Serial.print("aworld\t");
			Serial.print(aaWorld.x);
			Serial.print("\t");
			Serial.print(aaWorld.y);
			Serial.print("\t");
			Serial.println(aaWorld.z);
		#endif

		#ifdef OUTPUT_TEAPOT
			// display quaternion values in InvenSense Teapot demo format:
			teapotPacket[2] = fifoBuffer[0];
			teapotPacket[3] = fifoBuffer[1];
			teapotPacket[4] = fifoBuffer[4];
			teapotPacket[5] = fifoBuffer[5];
			teapotPacket[6] = fifoBuffer[8];
			teapotPacket[7] = fifoBuffer[9];
			teapotPacket[8] = fifoBuffer[12];
			teapotPacket[9] = fifoBuffer[13];
			Serial.write(teapotPacket, 14);
			teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		#endif

	}
}

// Combined function to tune a MPU 6050
void setMPUofffset(MPU6050 mpu,int acelX,int acelY,int acelZ,int gyroX,int gyroY,int gyroZ) {
	mpu.setXAccelOffset(acelX);
	mpu.setYAccelOffset(acelY);
	mpu.setZAccelOffset(acelZ);
	mpu.setXGyroOffset(gyroX);
	mpu.setYGyroOffset(gyroY);
	mpu.setZGyroOffset(gyroZ);
}
