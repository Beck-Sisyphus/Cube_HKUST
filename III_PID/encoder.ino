void getEncoderFeedback(float *wheel_angle_dot) {
	int encoder_feedback = 0;
	String incomeByte;
	// Get info from encoder Serial

	if (Serial1.available()) { incomeByte = Serial1.readStringUntil('\r'); }

	encoder_feedback = incomeByte.toInt();
	*wheel_angle_dot = (float)(2 * M_PI * encoder_feedback * 100) / 1024;
  return 0;
}
