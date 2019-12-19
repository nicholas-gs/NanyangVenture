using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Speedometer.DataPoints {
    /// <summary>
    /// Object passed to Cartesian graph for graphing x and y axis values
    /// </summary>
    public class MeasureModel {
        // X-Axis value
        public DateTime DateTime { get; set; }
        // Y-Axis value
        public double Value { get; set; }

    }
}
