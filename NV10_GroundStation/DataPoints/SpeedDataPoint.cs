using Speedometer.DataPoints;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Speedometer.Model {
    class SpeedDataPoint : BaseDataPoint {

        public static string SPEED_DATA_TYPE_KEY = "SPEED_DATA_POINT";

        // Speed in Km/h
        public float speed { get; set; }

        public SpeedDataPoint(int timeStampMilliSeconds, float speed) : base(SPEED_DATA_TYPE_KEY, timeStampMilliSeconds) {
            this.speed = speed;
        }

        public float getSpeed() {
            return this.speed;
        }

       

    }
}
