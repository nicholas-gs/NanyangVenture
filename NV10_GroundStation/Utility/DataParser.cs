using Speedometer.DataPoints;
using Speedometer.Model;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Speedometer.Utility {
    /// <summary>
    /// This class has 2 static methods that is responsible to parsing the two string into either a SpeedDataPoint object 
    /// or a FuelCellDataPoint object
    /// </summary>
    class DataParser {
        
        /// <summary>
        /// Converts raw string into a SpeedDataPoint object
        /// </summary>
        /// <param name="rawString"></param>
        /// <returns></returns>
        public static SpeedDataPoint ParseSpeedometerData(string rawString) {

            Console.WriteLine("ParseSpeedometerData() called");

            int timeStamp;
            float speed;
            SpeedDataPoint speedDataPoint;
            string groundStationTimeStamp = "";

            if(rawString != null && rawString.Length != 0) {
                // Split up the incoming raw string up at the tab character (should get 3 substrings)
                string[] splitStrings = rawString.Trim().Split(' ');
                Console.WriteLine("Number of split strings - " + splitStrings.Length);
                foreach(string s in splitStrings) {
                    Console.WriteLine("ss - " + s);
                }

                // Parse the time stamp from string HEX into Int32
                try {
                    timeStamp = Convert.ToInt32(splitStrings[2].Trim(), 16);
                } catch { timeStamp = 0; }

                // Parse the Speed (km/h) string into float
                try {    
                  /*  var culture = (CultureInfo)CultureInfo.CurrentCulture.Clone();
                    culture.NumberFormat.NumberDecimalSeparator = "."; */// The parse uses the culture settings by default -- some places use comma as seperator like 5,2 to represent 5.2
                    speed = float.Parse(splitStrings[4].Trim(), CultureInfo.InvariantCulture.NumberFormat);
                } catch { speed = 0f; }

                speedDataPoint = new SpeedDataPoint(timeStamp, speed);

                try {
                    groundStationTimeStamp = DateTime.Now.ToString("dddd, dd MMMM yyyy HH:mm:ss").Trim();
                } catch { }

            } else {
                speedDataPoint = new SpeedDataPoint(0, 0);
                speedDataPoint.groundStationTimeStamp = groundStationTimeStamp;
            }

            Console.WriteLine("Parsed SpeedDataPoint object, TimeStamp - " + speedDataPoint.timeStamp + ", Speed - " + speedDataPoint.getSpeed());

            return speedDataPoint;
        }

        /// <summary>
        /// Converts raw string into a FuelCellDataPoint object
        /// </summary>
        /// <param name="rawString"></param>
        /// <returns></returns>
        public static FuelCellDataPoint ParseFuelCellData(string rawString) {

            Console.WriteLine("ParseFuelCellData() called");

            float voltage = 0f;
            float current = 0f;
            float energy = 0f;
            float[] temperatures = new float[] { 0f, 0f, 0f, 0f };
            float pressure = 0f;
            float watt = 0f;
            string status = "No status";
            string groundStationTimeStamp = "";

            if (rawString != null && rawString.Length != 0) {
                // Split up the incoming raw string up at the space character (should get 15 substrings)
                string[] splitStrings = rawString.Split(' ');

                // Get current,
                try {
                    string temp = splitStrings[1].Trim();
                    int length = temp.Length;
                    current = float.Parse(temp.Substring(0, length - 1), CultureInfo.InvariantCulture.NumberFormat); // Length - 1 in order to get rid of the 'A' char
                } catch { }

                // Get Watt
                try {
                    string temp = splitStrings[2].Trim();
                    int length = temp.Length;
                    watt = float.Parse(temp.Substring(0, length - 1), CultureInfo.InvariantCulture.NumberFormat); // Length - 1 in order to get rid of the 'W' char
                } catch { }

                // Get Energy
                try {
                    string temp = splitStrings[3].Trim();
                    int length = temp.Length;
                    energy = float.Parse(temp.Substring(0, length - 2), CultureInfo.InvariantCulture.NumberFormat); // Length - 2 in order to get rid of the 'Wh' char
                } catch { }

                // Get the 4 temperatures
                try {
                    for (int i = 0; i < 4; i++) {
                        string temp = splitStrings[i + 4].Trim();
                        int length = temp.Length;
                        temperatures[i] = float.Parse(temp.Substring(0, length - 1), CultureInfo.InvariantCulture.NumberFormat); // Length - 1 in order to get rid of the 'C' char
                    }
                } catch { }

                // Get the pressure
                try {
                    string temp = splitStrings[8].Trim();
                    int length = temp.Length;
                    pressure = float.Parse(temp.Substring(0, length - 1), CultureInfo.InvariantCulture.NumberFormat); // Length - 1 in order to get rid of the 'B' char
                } catch { }

                // Get the voltage
                try {
                    string temp = splitStrings[9].Trim();
                    int length = temp.Length;
                    voltage = float.Parse(temp.Substring(0, length - 1), CultureInfo.InvariantCulture.NumberFormat); // Length - 1 in order to get rid of the 'V' char
                } catch { }

                try {
                    status = splitStrings[11].Trim();
                } catch { }

                try {
                    groundStationTimeStamp = DateTime.Now.ToString("dddd, dd MMMM yyyy HH:mm:ss").Trim();
                } catch { }

            }

            FuelCellDataPoint fuelCellDataPoint = new FuelCellDataPoint(0, voltage, current, watt, energy, temperatures, pressure, status);
            fuelCellDataPoint.groundStationTimeStamp = groundStationTimeStamp;
            return fuelCellDataPoint;
        }
     
    }
}
