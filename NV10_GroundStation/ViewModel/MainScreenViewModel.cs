using Speedometer.DataPoints;
using Speedometer.Model;
using Speedometer.Utility;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static Speedometer.Utility.DataReceiver;

namespace Speedometer.ViewModel {
    /// <summary>
    /// This is the ViewModel in the MVVM Architecture
    /// </summary>
    class MainScreenViewModel {

        public delegate void ViewModelDataPointReceivedCallback(BaseDataPoint baseDataPoint);

        private static ViewModelDataPointReceivedCallback viewModelDataPointReceivedCallback;
        // DataReceiver model object
        public static DataReceiver dataReceiver;
        // String of the COM Port name
        private string _comPortName { get; set; }

        public string comPortName { get { return _comPortName; }
            set { _comPortName = value; dataReceiver.comPortName = value; }
        }
     
        // Callback method that is triggered by the DataReceiver object and passes the DataPoint object
        private static DataPointReceivedCallback dataPointReceiverCallback;

        public MainScreenViewModel(string comPortName, ViewModelDataPointReceivedCallback viewModelDataPointReceivedCallback1) {
            if (dataPointReceiverCallback == null) {
                dataPointReceiverCallback = new DataPointReceivedCallback(dataPointReceived);
            }

            if (dataReceiver == null) {
                dataReceiver = new DataReceiver(comPortName, dataPointReceiverCallback);
            }
         
            if (viewModelDataPointReceivedCallback == null) {
                viewModelDataPointReceivedCallback = viewModelDataPointReceivedCallback1;
            }
            this.comPortName = comPortName;

        }

        /// <summary>
        /// Callback method triggered by DataReceiver class when a data point is passed to this viewmodel class
        /// </summary>
        /// <param name="baseDataPoint"></param>
        private void dataPointReceived(BaseDataPoint baseDataPoint) {
            Console.WriteLine("ViewModel - Data Point received");
            if(baseDataPoint == null) {
                Console.WriteLine("ViewModel  - DataPoint is null");
            } else {
                Console.WriteLine("ViewModel - DataPoint not null");
            }
            if(viewModelDataPointReceivedCallback != null) {
                viewModelDataPointReceivedCallback(baseDataPoint);
            }
        }

    }
}
