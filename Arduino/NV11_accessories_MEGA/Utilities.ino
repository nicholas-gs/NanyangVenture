/// <summary>
/// Initializes SD card if present. Also creates a new folder to store this session's data into.
/// </summary>
/// <param name="path">Generated folder path is stored into here.</param>
/// <returns></returns>
bool initSD(SdFat& card)
{
	//TODO: Change SD Pin based on hardware
	if (!card.begin(SD_SPI_CS_PIN))
		return false;

	// Obtain number of existing entries in SD card
	uint16_t folderEntries = 0;
	char folderPath[18] = "/NV11_";	// max 8 chars. len("NV10_") = 5, pad 3 digits to the new folder
	do {
		sprintf(folderPath + 6, "%03d/", ++folderEntries);	// pad '0' on the front if number contains less than 3 digits
		debugSerialPort.print("Checking: "); debugSerialPort.println(folderPath);
	} while (card.exists(folderPath));
	debugSerialPort.print(folderPath); debugSerialPort.println(" does not exist!");
	// Wipe when too full, or on user request ('~' char)
	if (false)//folderEntries > 30)
	{
		Serial.println(F("SD card is nearing threshold to crash Arduino."));
		if (initiateWipe(card))
		{
			sprintf(folderPath + 6, "%03d/", 1);
		}
	}
	else
	{
		for (int wipeWindow = 10; wipeWindow >= 0; wipeWindow--)
		{
			if (Serial.read() == '~')
			{
				Serial.println(F("Requesting to wipe SD card."));
				if (initiateWipe(card))
				{
					sprintf(folderPath + 6, "%03d/", 1);
					break;
				}
			}
			delay(200);
		}
	}

	// Make new directory and operate within it.
	card.mkdir(folderPath);
	card.chdir(folderPath);
	debugSerialPort.print("Created dir "); debugSerialPort.println(folderPath);
	//File f;
	//// initialize FC column text
	//f = card.open("FC.txt", FILE_WRITE);
	//f.println(F("	Millis	Watt	P	Tmp	Status"));
	//f.close();
	//// initialize CS column text
	//f = card.open("CS.txt", FILE_WRITE);
	//f.println(F("	Millis	 Volt	CapIn	CapOut	Motor"));
	//f.close();
	//// initialize SM column text
	//f = card.open("SM.txt", FILE_WRITE);
	//f.println(F("	Millis	km/h"));
	//f.close();

	return true;
}
bool initiateWipe(SdFat& card)
{
	File sub2;
	bool willWipe;
	Serial.println(F("Wipe the card? (y/n)"));

	char response = '\0';
	while (response != 'y' && response != 'n')
	{
		delay(100);
		response = Serial.read();
	}
	willWipe = response == 'y';
	if (willWipe)
	{
		// wipe card
		Serial.print(F("Wiping card"));
		card.wipe(&Serial);
		Serial.println(F("done"));
		delay(1000);
		if (!card.begin(SD_SPI_CS_PIN))
		{
			Serial.println(F("Card corrupted. Please format manually.\nArduino will reset in 5 seconds..."));
			delay(5000);
			softReset();
		}
	}
	flushRX();
	return willWipe;
}
void flushRX()
{
	// clean up any trailing characters in serial RX buffer
	while (Serial.available())
		Serial.read();
}
void softReset()
{
	asm volatile ("  jmp 0");
}