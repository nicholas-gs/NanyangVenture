using Speedometer.DataPoints;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Web.Script.Serialization;

namespace Speedometer.Model {
    class DataLogger {
        // Path to file
        private String filePath;
        private JavaScriptSerializer serializer;

        public DataLogger() {
            createDirectory();
            createTextFile();
            serializer = new JavaScriptSerializer();
        }

        public void logData(BaseDataPoint baseDataPoint) {
            String jsonStr;
            if (baseDataPoint is SpeedDataPoint) {
                jsonStr = serializer.Serialize((SpeedDataPoint)baseDataPoint);
                jsonStr = jsonStr + ", ";
                Console.WriteLine("jsonStr - " + jsonStr);
            } else {
                jsonStr = serializer.Serialize((FuelCellDataPoint)baseDataPoint);
                jsonStr = jsonStr + ", ";
                Console.WriteLine("jsonStr - " + jsonStr);
            }
            try {
                using (StreamWriter sw = File.AppendText(filePath)) {
                    sw.WriteLine(jsonStr);
                }
            } catch (IOException) {
                Console.WriteLine("Error Writing to file - " + filePath);
            }
           
        }

        /// <summary>
        /// Create a folder called "LoggedData". If it already exists, then no duplicate will be created
        /// </summary>
        private void createDirectory() {
            try {
                System.IO.Directory.CreateDirectory(@".\LoggedData");
            } catch {

            }
        }

        /// <summary>
        /// Create a file with the name as the starting time
        /// </summary>
        private void createTextFile() {
            // Create a file with the name as the starting time
            filePath = DateTime.Now.ToString("dddd, dd MMMM yyyy HH:mm:ss").Trim();
            filePath = filePath.Replace(":", ".");
            filePath = @".\LoggedData\" + filePath + ".txt";
            try {
               /* File.Create(filePath);*/
                Console.WriteLine(filePath + " created.");
                using (StreamWriter sw = File.CreateText(filePath)) {
                    sw.WriteLine("[");
                    sw.Close();
                }
            } catch (IOException e) {
                Console.WriteLine(e.Message);
            }
        }

        public void closeDataLogger() {
            using(StreamWriter sw = File.AppendText(filePath)) {
                sw.WriteLine("]");
            }
        }

    }
}
