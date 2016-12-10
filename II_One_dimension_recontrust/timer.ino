void enableTimer1Interrupt()
{
	TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0

    // count from 45535, for 16-bit counter, 45535 to 65535 get 20000 count
    // minus 1480 more to compensate the calculation time, 0.74 ms
    TCNT1 = 0xB1DF;
    // Set CS10 bit so timer runs at clock speed: 100Hz, 0.01 sec
    // Prescaling: clk/8, CS11 set to 1
    TCCR1B |= (1 << CS11);  // Similar: TCCR1B |= _BV(CS11);

    // and input capture interrupt enable
    TIMSK1 |= (1 << TOIE1);
}

void enableTimer4EdgeDetection()
{
	TCCR4A = 0;    // set entire TCCR1A register to 0
    TCCR4B = 0;    // set entire TCCR1B register to 0

    ICR4H = 0;
    ICR4L = 0;

    // count from 45535, for 16-bit counter, 45535 to 65535 get 20000 count
    TCNT4 = 0xB1DF;

    // Set CS10 bit so timer runs at clock speed: 100Hz, 0.01 sec
    // Prescaling: clk/8, CS11 set to 1
    TCCR4B |= (1 << CS41);  // Similar: TCCR1B |= _BV(CS11);

    // ICES1 Input Capture Edge Select flag, 1 for rising, 0 for falling
    TCCR4B |= (1 << ICES4);

    // and input capture interrupt enable
    TIMSK4 |= (1 << ICIE4);
    // enable Timer overflow interrupt:
    TIMSK4 |= (1 << TOIE4);
}
