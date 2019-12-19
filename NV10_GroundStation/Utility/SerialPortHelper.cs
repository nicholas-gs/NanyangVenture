using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;

namespace Speedometer.Utility {
    /// <summary>
    /// Helper class for opening the com port and receiving the data. Used by the DataReceiver class
    /// </summary>
    class SerialPortHelper {

        // Use this delegate to pass the incoming string back to the DataReceiver class
        public delegate void SerialPortDataReceivedCallBack(string dataText);

        private string _portName;

        /// <summary>
        /// Whenever the port selection is changed, we call the initSerialPort() method
        /// </summary>
        public string portName { get { return _portName; }
        set {
                if(_portName != null) {
                    closeSerialPort(); // If there was already a serial port opened, we make sure to close it so that only 1 port is ever opened at any one time
                }
                if (value.Length == 0) {
                    _portName = " ";
                } else { _portName = value; }
             initSerialPort(); }
        }
        private static SerialPortDataReceivedCallBack dataReceivedCallBack;
        public const int BAUD_RATE = 9600;
        public const int NUMBER_OF_DATA_BITS = 8;
        public const int PARITY_BIT = (int)Parity.None;
        public const int STOP_BIT = (int)StopBits.One;
        public const int IIME_OUT = 6000;

        private static SerialPort serialPort;

        public SerialPortHelper(string portName1, SerialPortDataReceivedCallBack dataReceivedCallBack1) {
            portName = portName1;
            dataReceivedCallBack = dataReceivedCallBack1;
        }

        /// <summary>
        /// Intialise the Serial Port with the right parameters and the callback method
        /// </summary>
        private void initSerialPort() {
        
            serialPort = new SerialPort(_portName, BAUD_RATE, PARITY_BIT, NUMBER_OF_DATA_BITS, (StopBits)STOP_BIT);
            serialPort.ReadTimeout = IIME_OUT; // Timeout after 1 minute
            serialPort.DataReceived += SerialPort_DataReceived; // Pass the com port data received callback method
            // Open the serial port
            openSerialPort();
        }
       
        /// <summary>
        /// Callback method when data is received on the port
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e) {
            string dataText;

            // Read in the string
            try {
                dataText = serialPort.ReadLine();
            } catch {
                dataText = "";
            }
        
            // If the delegate is not null, then invoke it and pass the input string
            if(dataReceivedCallBack != null && dataText.Length != 0) {
                dataReceivedCallBack.Invoke(dataText);
            }
        }

        /// <summary>
        /// Open the serial port
        /// </summary>
        private void openSerialPort() {
            if (!serialPort.IsOpen) {
                try {
                    serialPort.Open();
                    Console.WriteLine("TAG : " + this.portName + " port opened successfully.");
                } catch {
                    Console.WriteLine("TAG : " + this.portName + " cannot be opened.");
                }
            } else {
                Console.WriteLine("TAG : " + this.portName + " is already open.");
            }
        }

        /// <summary>
        /// Close the serial port
        /// </summary>
        private void closeSerialPort() {
            try {
                serialPort.Close();
                Console.WriteLine("TAG : " + this.portName + " port closed successfully.");
            } catch {
                Console.WriteLine("TAG : " + this.portName + " cannot be closed.");
            }
        }
    }
}
