using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Speedometer.DataPoints {
    /// <summary>
    /// Base entity class for data
    /// </summary>
    abstract class BaseDataPoint {
        // TimeStamp assigned by the ground station
        public string groundStationTimeStamp { get; set; }
        // TimeStamp from the vehicle logging system (NOTE - The FuelCell data does not have a timeStamp)
        private int _timeStamp;
        public int timeStamp { get { return _timeStamp; } set { if (value < 0) _timeStamp = 0; else _timeStamp = value; } }
        public string dataType { get; set; }

        public BaseDataPoint(string dataType, int timeStamp) {
            this.timeStamp = timeStamp;
            this.dataType = dataType;
        }
    }
}
