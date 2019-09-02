#pragma once

#ifdef DEBUG
#define debug_(str) Serial.print(str)
#define debug(str)  Serial.println(str)
#else
#define debug_(...)
#define debug(...)
#endif