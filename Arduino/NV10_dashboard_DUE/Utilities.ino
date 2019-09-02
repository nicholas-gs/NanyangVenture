
void setDebounce(const unsigned int pins[], uint8_t numPins, uint16_t waitTimeMultiplier)
{
	/*
	http://ww1.microchip.com/downloads/en/devicedoc/atmel-11057-32-bit-cortex-m3-microcontroller-sam3x-sam3a_datasheet.pdf
	page 630 lists help on which PIO registers to fiddle to enable debouncing
	need to find out clock divider value, max value is 2^14 (PIO_SCDR)
	https://www.arduino.cc/en/Hacking/PinMappingSAM3X
	to help convert pins into PIOxyy (x = A, B, C, D) (y = a number in range [0, 31])
	*/
	uint32_t pinsBitMask_A = 0, pinsBitMask_B = 0, pinsBitMask_C = 0, pinsBitMask_D = 0;
	for (uint8_t i = 0; i < numPins; i++)
	{
		switch ((uint32_t)digitalPinToPort(pins[i]))
		{
		case (uint32_t)PIOA:
			pinsBitMask_A |= digitalPinToBitMask(pins[i]);
			break;
		case (uint32_t)PIOB:
			pinsBitMask_B |= digitalPinToBitMask(pins[i]);
			break;
		case (uint32_t)PIOC:
			pinsBitMask_C |= digitalPinToBitMask(pins[i]);
			break;
		case (uint32_t)PIOD:
			pinsBitMask_D |= digitalPinToBitMask(pins[i]);
			break;
		}
	}
	// IFER - Input Filter Enable Register:					Enables these bits to enable inupt filtering
	PIOA->PIO_IFER = pinsBitMask_A;
	PIOB->PIO_IFER = pinsBitMask_B;
	PIOC->PIO_IFER = pinsBitMask_C;
	PIOD->PIO_IFER = pinsBitMask_D;
	// DIFSR - Debouncing Input Filter Select Register:		We want Debounce filter, not Glitch filter! Debounce = 1, Glitch = 0
	PIOA->PIO_DIFSR = pinsBitMask_A;
	PIOB->PIO_DIFSR = pinsBitMask_B;
	PIOC->PIO_DIFSR = pinsBitMask_C;
	PIOD->PIO_DIFSR = pinsBitMask_D;
	// SCDR - Slow Clock Divider Register:					Too big = unreponsive, Too small = can't feel the debounce
	PIOA->PIO_SCDR = waitTimeMultiplier;
	PIOB->PIO_SCDR = waitTimeMultiplier;
	PIOC->PIO_SCDR = waitTimeMultiplier;
	PIOD->PIO_SCDR = waitTimeMultiplier;
}
// ONLY FOR DEBUGGING
//void debugPrint(char* toPrint, int len)
//{
//	// DEBUG printing that prints out special bytes as uint
//	int counter = 0;
//	while (counter < len)
//	{
//		char tmp = toPrint[counter++];
//		if (isPrintable(tmp))
//			Serial.print(tmp);
//		else
//		{
//			Serial.print("<");
//			Serial.print((uint8_t)tmp);
//			Serial.print(">");
//		}
//	}
//	Serial.print("[");
//	Serial.print(Serial.available());
//	Serial.print("]");
//	Serial.println("__");
//}