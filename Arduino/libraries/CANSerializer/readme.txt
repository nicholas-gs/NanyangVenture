CAN Serializer library provides a base class for users to write their custom classes. The custom classes will ultimately serve as "Data Points" between CAN frames and whatever data struct deemed proper for a particular telemetry.

The custom classes define their own data structs as class variables, which are aliases to a pre-existing byte-array of length 8 in the base class.
These "alias class variables" will be rolled into a CAN frame when packCAN() is called.
Every custom class should also provide their method to convert their data struct into a readable String via packString().


Each custom class preserves a unique 2-letter ID and CAN ID so they are distinguishable in their packed format, be it CAN or String.
When a CAN frame / String arrives, checkMatchCAN() / checkMatchString() should be called to identify which Data Point it belongs to. The matching Data Point should then call its unpackCAN() / unpackString() to populate its own class variables with the incoming values, which can then be accessed via getter functions.



-> BYTE ARRAY OF LENGTH 8 IS A HARD LIMIT IN THIS VERSION OF CANSERIALIZER
-> Raw data processing can be bundled into a Data Point subclass via insertData()
-> Logic for gathering data should not be bundled into a Data Point subclass (e.g. performing Serial reads or obtaining data from I2C bus should be handled separately by the main sketch)