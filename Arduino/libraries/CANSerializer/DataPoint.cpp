// 
// 
// 

#include "DataPoint.h"
const char* STRING_HEADER[] = { "FC","CS","Cs","SM","H2","ST","CM","HB","US"};
//DataPoint::DataPoint(eEncodingPreset (&presets)[8])
//{
//	uint8_t overFlow = 0;
//	for (int i = 0; i < sizeof(presets); i++)
//	{
//		this->presets[i] = presets[i];
//		switch (presets[i])
//		{
//		case preset_uint8:
//		case preset_uint8x10:
//			overFlow += 1;
//			break;
//		case preset_uint16:
//		case preset_uint16x10:
//			overFlow += 2;
//			break;
//		case preset_float:
//		case preset_uint32:
//			overFlow += 4;
//			break;
//		default:
//			break;
//		}
//	}
//	
//	if (overFlow > 8)
//		errorFlag = 1;
//}
// TODO: see DataPoint.h file

void DataPoint::setCanId(uint8_t id)
{
	CANId = id;
}
uint8_t DataPoint::getCanId()
{
	return CANId;
}
DataPoint::DataPoint(const char* strHeader, uint8_t CANId, const uint8_t CANLength):CANId(CANId), CANLength(CANLength)
{
	strcpy(this->strHeader, strHeader);
}

const char * DataPoint::getStringHeader()
{
	return strHeader;
}

bool DataPoint::checkMatchCAN(const CANFrame * f)
{
	//Serial.print("ID: "); Serial.print(f->id); Serial.print(" "); Serial.println(CANId);
	//Serial.print("Len: "); Serial.print(f->length); Serial.print(" "); Serial.println(CANLength);
	if (f->id == this->CANId && f->length == this->CANLength)
	{
		return true;
	}
	return false;
}
void DataPoint::packCAN(CANFrame *f)
{
	f->id = CANId;
	f->length = this->CANLength;
	memcpy(f->payload, data.Byte, CANLength);
}
void DataPoint::unpackCAN(const CANFrame *f)
{
	memcpy(data.Byte, f->payload, CANLength);
}

char* DataPoint::packStringDefault(char * str)
{
	int charsPrinted = sprintf(str, "%s\t%lx\t", getStringHeader(), timeStamp);
	str += charsPrinted;// incr counter of the calling function
	return str;
}
char* DataPoint::unpackStringDefault(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	return ptr;
}
bool DataPoint::checkMatchString(char * str)
{
	if (!strncmp(str, getStringHeader(), 2))
	{
		return true;
	}
	return false;
}
void DataPoint::packString(char *str)
{
	char* shiftedStr = DataPoint::packStringDefault(str);
	// param1 = 4, param2 = 2
	sprintf(shiftedStr, "%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x", data.Byte[0], data.Byte[1], data.Byte[2], data.Byte[3], data.Byte[4], data.Byte[5], data.Byte[6], data.Byte[7]);
}
void DataPoint::unpackString(char * str)
{
	char* ptr = strtok(str, "\t");
	ptr = strtok(NULL, "\t");
	timeStamp = strtoul(ptr, NULL, 16);

	for (int i = 0; i < 8; i++)
	{
		ptr = strtok(NULL, "\t");
		data.Byte[i] = strtoul(ptr, NULL, 16);
	}
}

bool DataPoint::dataHasChanged()
{
	bool changed = memcmp(data.Byte, oldData.Byte, 8) != 0;
	memcpy(oldData.Byte, data.Byte, 8);
	return changed;
}

void DataPoint::printRaw()
{
	for (int i = 0; i < CANLength; i++)
	{
		Serial.print(" 0x");
		Serial.print(data.Byte[i], HEX);
	}
}
void debugPrint(char * toPrint, int len)
{
#if DEBUG
	for (int i = 0; i < len; i++)
	{
		char c = *(toPrint + i);
		if (isPrintable(c))
			Serial.print(c);
		else
		{
			Serial.print("<");
			Serial.print((uint8_t)c);
			Serial.print(">");
		}
	}
	Serial.println();
#endif
}

void print(char * toPrint)
{
	Serial.println(toPrint);
}
