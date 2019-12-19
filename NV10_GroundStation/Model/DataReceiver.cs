using Speedometer.DataPoints;
using Speedometer.Model;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using static Speedometer.Utility.SerialPortHelper;

namespace Speedometer.Utility {
    /// <summary>
    /// This is the Model in the MVVM Architecture
    /// </summary>
    class DataReceiver {

        /// <summary>
        /// Delegate for passing the parsed data point object to the ViewModel
        /// </summary>
        /// <param name="baseDataPoint"></param>
        public delegate void DataPointReceivedCallback(BaseDataPoint baseDataPoint);
        private static string _comPortName;
        public string comPortName { get { return _comPortName; } set { _comPortName = value; serialPortHelper.portName = value; } }
        private static SerialPortHelper serialPortHelper;
        private static DataPointReceivedCallback dataPointReceivedCallback;
        private static SerialPortDataReceivedCallBack dataReceivedCallBack;


        public DataReceiver(string comPortName, DataPointReceivedCallback dataPointReceivedCallback1) {
            // Delegate for callback from SerialPortHelper class -- triggered when data is received from serial port
            if(dataReceivedCallBack == null) {
                dataReceivedCallBack = new SerialPortDataReceivedCallBack(dataReceived);
            }
       
            if(dataPointReceivedCallback == null) {
                dataPointReceivedCallback = dataPointReceivedCallback1;
            }
            // Instantiate a SerialPortHelper object
            if(serialPortHelper == null) {
                serialPortHelper = new SerialPortHelper(comPortName, dataReceivedCallBack);
            }

            // Testing DELETE LATER
            // sendSampleSpeedData();

        }

        /// <summary>
        /// This method is called whenever the delegate is invoked by the SerialPortHelper class. 
        /// The parameter is the incoming string from the com port.
        /// </summary>
        /// <param name="dataStr"></param>
        void dataReceived(string dataStr) {
            Console.WriteLine("TAG : String incoming from com port - " + dataStr);
            BaseDataPoint dataPoint;
            if (dataStr.Trim().StartsWith("SM")) {
                dataPoint = DataParser.ParseSpeedometerData(dataStr);
            } else if (dataStr.Trim().StartsWith(">>")) {
                dataPoint = DataParser.ParseFuelCellData(dataStr);
            } else {
                dataPoint = null;
            }

            // Pass the data point object to the ViewModel using the delegate
            if (dataPointReceivedCallback != null && dataPoint != null) {
                dataPointReceivedCallback.Invoke(dataPoint);
            }
        }

        private void sendSampleSpeedData() {
            float[] randomSpeeds = new float[] {
                5f,10f,7.7f,8.8f,9.9f,10f,11f,15f,21f, 19f, 24f,30f,34f,32f,34f,10f,4f,2f,28f,25f,22f,17f,10f,8f,6f,1f
            };

            int i = 0;
            var timer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            timer.Start();
            timer.Tick += (sender, args) => {
                timer.Stop();

                dataPointReceivedCallback.Invoke(new SpeedDataPoint(0, randomSpeeds[i]));
                i += 1;
                timer.Start();
            };
        }
    }
}


