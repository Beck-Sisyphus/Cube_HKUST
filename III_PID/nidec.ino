// Initialize the Nidec motor and set the speed to zero
// For stop pin, low for stop, high for release
// For direction pin, low for ccw, high for cw
// For PWM pin, minimum frequency 330Hz, recommand 20-30KHz
// 255 is stop, 253 slowliest. 0 fastest
void nidec_motor_init() {
	pinMode(NIDEC_STOP_PIN, OUTPUT);
	pinMode(NIDEC_SPEED_MODE_SLOW, OUTPUT);
	pinMode(NIDEC_PWM_PIN, OUTPUT);

	digitalWrite(NIDEC_STOP_PIN, HIGH);
	digitalWrite(NIDEC_SPEED_MODE_SLOW, HIGH);

	analogWrite(NIDEC_PWM_PIN, 255);
}

// Set the motor speed from -255 to 255 with continous connection
// 0 stops, 255 as the fastest, -255 the slowliest
// TODO: Change to rad.s^-1 after feedback
void nidec_speed(int speed) {
	if (speed > 0 ) {
		digitalWrite(NIDEC_STOP_PIN, HIGH);
		digitalWrite(NIDEC_SPEED_MODE_SLOW, HIGH);
		if (speed > 255) {
			analogWrite(NIDEC_PWM_PIN, 0);
		}
		else {
			analogWrite(NIDEC_PWM_PIN, 255 - speed);
		}
	}
	else if (speed < 0) {
		digitalWrite(NIDEC_STOP_PIN, HIGH);
		digitalWrite(NIDEC_SPEED_MODE_SLOW, LOW);
		if (speed > 255) {
			analogWrite(NIDEC_PWM_PIN, 0);
		}
		else {
			analogWrite(NIDEC_PWM_PIN, 255 - speed);
		}
	}
	else {
		digitalWrite(NIDEC_STOP_PIN, LOW);
		digitalWrite(NIDEC_SPEED_MODE_SLOW, HIGH);
		analogWrite(NIDEC_PWM_PIN, 255);
	}
}
