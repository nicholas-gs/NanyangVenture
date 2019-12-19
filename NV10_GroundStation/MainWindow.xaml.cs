using LiveCharts;
using LiveCharts.Configurations;
using LiveCharts.Defaults;
using Speedometer.DataPoints;
using Speedometer.Model;
using Speedometer.Utility;
using Speedometer.ViewModel;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using static Speedometer.ViewModel.MainScreenViewModel;

namespace Speedometer {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged {
  
        private static string comPortStr;

        // MainScreenViewModel object
        private static MainScreenViewModel mainScreenViewModel;
        // Callback method from the ViewModel
        private static ViewModelDataPointReceivedCallback dataPointReceivedCallback;
        // Data Logger class that saves incoming data into a text file
        private static DataLogger dataLogger;

        private double _axisMax;
        private double _axisMin;
  
        // Saving logging data
        private static Boolean saveData = false;

        private ObservableValue VoltageValues;
        private ObservableValue CurrentValues;
        private ObservableValue WattValues;
        private ObservableValue EnergyValues;

        private static int cartesianItemMaxCount = 15; // The maximum number of items shown in the cartesian charts before clearing it -- To prevent stuttering
        private static int voltageItemCount = 0; // Keep track of the number of data points shown in the voltage chart
        private static int currentItemCount = 0; // Keep track of the number of data points shown in the current chart
        private static int wattItemCount = 0; // Keep track of the number of data points shown in the watt chart
        private static int energyItemCount = 0; // Keep track of the number of data points shown in the energy chart

        public ChartValues<MeasureModel> VoltageChartValues { get; set; } // Values of the Voltage Cartesian Chart
        public ChartValues<MeasureModel> CurrentChartValues { get; set; } // Values of the Current Cartesian Chart
        public ChartValues<MeasureModel> WattChartValues { get; set; } // Values of the Watt Cartesian Chart
        public ChartValues<MeasureModel> EnergyChartValues { get; set; } // Values of the Energy Cartesian Chart

        private static float highestSpeed = 0.0f; // The highest speed recorded so far
        private static float highestTemp1 = 0.0f; // The highest temp1 recorded so far
        private static float highestTemp2 = 0.0f; // The highest temp2 recorded so far
        private static float highestTemp3 = 0.0f; // The highest temp3 recorded so far
        private static float highestTemp4 = 0.0f; // The highest temp4 recorded so far

        public Func<double, string> DateTimeFormatter { get; set; }
        public double AxisStep { get; set; }
        public double AxisUnit { get; set; }
        public bool IsReading { get; set; }

        public MainWindow() {
            InitializeComponent();

            this.DataContext = this; IsReading = true;
      
            VoltageValues = new ObservableValue(0);
            CurrentValues = new ObservableValue(1);
            WattValues = new ObservableValue(2);
            EnergyValues = new ObservableValue(3);

            var mapper = Mappers.Xy<MeasureModel>()
           .X(model => model.DateTime.Ticks/TimeSpan.FromTicks(1).Ticks)   //use DateTime.Ticks as X (Cannot be FromSeconds() else the graph won't show anything!)
           .Y(model => model.Value);                                        //use the value property as Y

            //lets set how to display the X Labels (mm:ss)
            DateTimeFormatter = value => new DateTime((long)value).ToString("mm:ss");

            //lets save the mapper globally.
            Charting.For<MeasureModel>(mapper);

            //the values property will store our values array
            VoltageChartValues = new ChartValues<MeasureModel>(); // For the voltage cartesian graph values
            CurrentChartValues = new ChartValues<MeasureModel>(); // For the current cartesian graph values
            WattChartValues = new ChartValues<MeasureModel>();
            EnergyChartValues = new ChartValues<MeasureModel>();

            //AxisStep forces the distance between each separator in the X axis
            AxisStep = TimeSpan.FromSeconds(1).Ticks;
            //AxisUnit forces lets the axis know that we are plotting seconds
            //this is not always necessary, but it can prevent wrong labeling
            AxisUnit = TimeSpan.TicksPerSecond;

            SetAxisLimits(DateTime.Now);
        }

        /// <summary>
        /// Listener for when the window is loaded
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Window_Loaded(object sender, RoutedEventArgs e) {
            dataPointReceivedCallback = new ViewModelDataPointReceivedCallback(dataPointReceived);
            mainScreenViewModel = new MainScreenViewModel(" ", dataPointReceivedCallback);   
        }

        /// <summary>
        /// Callback method for when the comPort selection comboBox is opened
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PortSelectionComboBox_DropDownOpened(object sender, EventArgs e) {
            getAllComPorts(); // Get all the serial ports on the computer
        }

        /// <summary>
        /// Callback method for when the comPort ComboBox is closed
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void PortSelectionComboBox_DropDownClosed(object sender, EventArgs e) {
            try {
                comPortStr = ((ComboBox)sender).SelectedItem.ToString(); // Save the selection
            } catch { }

            Console.WriteLine("TAG - Port " + comPortStr + " selected");

            if (comPortStr != null && comPortStr.Length != 0) {
                try {
                    mainScreenViewModel.comPortName = comPortStr; // Send the new selection to the viewmodel
                    ((ComboBox)sender).SelectedItem = comPortStr;
                } catch { }
             
            } 
        }

        /// <summary>
        /// Set the y-axis min and max values for the cartesian live graph
        /// </summary>
        /// <param name="now"></param>
        private void SetAxisLimits(DateTime now) {
            AxisMax = now.Ticks + TimeSpan.FromSeconds(1).Ticks; // lets force the axis to be 1 second ahead
            AxisMin = now.Ticks - TimeSpan.FromSeconds(8).Ticks; // and 10 seconds behind
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged(string propertyName = null) {
            if (PropertyChanged != null) {
                PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
            } 
        }

        /// <summary>
        /// CartesianChart x-axis max value
        /// </summary>
        public double AxisMax {
            get { return _axisMax; }
            set {
                _axisMax = value;
                OnPropertyChanged("AxisMax");
            }
        }
        /// <summary>
        /// CartesianChart x-axis min value
        /// </summary>
        public double AxisMin {
            get { return _axisMin; }
            set {
                _axisMin = value;

                OnPropertyChanged("AxisMin");
            }
        }

        /// <summary>
        /// Set the Speed Value for the speed gauge, the max speed & average speed textblock
        /// </summary>
        /// <param name="speed"></param>
        private void updateSpeedValues(float speed) {
            // Update Speed Gauge
            this.Dispatcher.Invoke(() => {
                Console.WriteLine("Updating speed gauge - " + speed);
                speedGauge.Value = (int)speed;
            });

            // Update max speed
            this.Dispatcher.Invoke(() => {
                if (speed > highestSpeed) {
                    highestSpeed = speed;
                    this.HighestSpeedTextBlock.Text = speed.ToString().Trim();
                }
            });
        }

        /// <summary>
        /// Set the fuel cell status indicator background color and text
        /// </summary>
        /// <param name="status"></param>
        private void setFuelCellStatus(string status) {
            if(status == "IN") {
                this.Dispatcher.Invoke(() => {
                    this.statusIndicatorBackground.Background = Brushes.DarkOrange;
                    this.statusTextBlock.Text = "Initialiating";
                });
            } else if(status == "SS") {
                this.Dispatcher.Invoke(() => {
                    this.statusIndicatorBackground.Background = Brushes.Yellow;
                    this.statusTextBlock.Text = "SS";
                });
            } else if(status == "OP") {
                this.Dispatcher.Invoke(() => {  
                    this.statusIndicatorBackground.Background = Brushes.Green;
                    this.statusTextBlock.Text = "Operating";
                });
            } else if(status == "SD") {
                this.Dispatcher.Invoke(() => {
                    this.statusIndicatorBackground.Background = Brushes.DarkRed;
                    this.statusTextBlock.Text = "Shutting Down";
                });
            } else {
                this.Dispatcher.Invoke(() => {
                    this.statusIndicatorBackground.Background = Brushes.DarkGray;
                    this.statusTextBlock.Text = "No status";
                });
            }
        }

        /// <summary>
        /// Get all the COM Ports on the 
        /// </summary>
        private void getAllComPorts() {

            this.portSelectionComboBox.Items.Clear();

            // Get all ports & display them in the combo box
            this.Dispatcher.Invoke(() => {
                string[] allPorts = SerialPort.GetPortNames();
                if (allPorts.Length == 0) {
                    this.portSelectionComboBox.Items.Add("No COM Ports");
                } else {
                    // Put all the strings into the combobox
                    foreach (string s in allPorts) {
                        this.portSelectionComboBox.Items.Add(s);
                    }
                }
            });
         
        }

        /// <summary>
        /// This method is called by the ViewModel whenever a new data point object is received
        /// </summary>
        /// <param name="baseDataPoint"></param>
        private void dataPointReceived(BaseDataPoint baseDataPoint) {
            Console.WriteLine("Main Window - DataPoint received");

            if (baseDataPoint != null && dataLogger != null && saveData == true) {
                this.Dispatcher.Invoke(() => {
                    dataLogger.logData(baseDataPoint);
                });

            }

            if (baseDataPoint is SpeedDataPoint) {
                Console.WriteLine("SpeedDataPoint received by main window");
                updateSpeedWidgets((SpeedDataPoint)baseDataPoint);
            } else if (baseDataPoint is FuelCellDataPoint) {
                Console.WriteLine("FuelCellDataPoint received by main window");
                updateFuelCellWidgets((FuelCellDataPoint)baseDataPoint);
            }
        }

        /// <summary>
        /// Update all the widgets related to the SpeedDataPoint object
        /// </summary>
        /// <param name="speedDataPoint"></param>
        private void updateSpeedWidgets(SpeedDataPoint speedDataPoint) {
            Console.WriteLine("Updating Speed Widgets");
            Console.WriteLine(" Speed - " + speedDataPoint.getSpeed());
            updateSpeedValues(speedDataPoint.getSpeed());
        }

        /// <summary>
        /// Update all the widgets related to the FuelCellDataPoint object
        /// </summary>
        /// <param name="fuelCellDataPoint"></param>
        private void updateFuelCellWidgets(FuelCellDataPoint fuelCellDataPoint) {
            Console.WriteLine("Updating Fuel Cell Widgets");

            try {
                Console.WriteLine("Voltage - " + fuelCellDataPoint.voltage, ", current - " + fuelCellDataPoint.current + ", watt - " + fuelCellDataPoint.watt
                    + ", energy - " + fuelCellDataPoint.energy + ", temp1 - " + fuelCellDataPoint.temperatures[0] + ", temp2 - " + fuelCellDataPoint.temperatures[1] +
                    ", temp3 - " + fuelCellDataPoint.temperatures[2] + ", temp4 - " + fuelCellDataPoint.temperatures[3] + ", pressure - " + fuelCellDataPoint.pressure
                    + ", status - " + fuelCellDataPoint.status);
            } catch { }

            DateTime now = DateTime.Now;
            SetAxisLimits(now);
            setFuelCellStatus(fuelCellDataPoint.status);
            updateVoltageValues(fuelCellDataPoint.voltage, now);
            updateCurrentValues(fuelCellDataPoint.current, now);
            updateWattValues(fuelCellDataPoint.watt, now);
            updateEnergyValues(fuelCellDataPoint.energy, now);
            updateTemperaturesValues(fuelCellDataPoint.temperatures, now);
            updatePressureValues(fuelCellDataPoint.pressure);
        }

        /// <summary>
        ///  Update the voltage cartesian graph
        /// </summary>
        /// <param name="voltageLevel"></param>
        /// <param name="now"></param>
        private void updateVoltageValues(float voltageLevel, DateTime now) {
            Console.WriteLine("Updating voltage widget - Value of voltage is " + voltageLevel);
            this.Dispatcher.Invoke(() => {

                if(voltageItemCount > cartesianItemMaxCount) {
                    VoltageChartValues.Clear();
                    voltageItemCount = 0;
                } else {
                    voltageItemCount += 1;
                }
               
                VoltageChartValues.Add(new MeasureModel {
                    Value = voltageLevel,
                    DateTime = now,
                });
                this.voltageValueTextBlock.Text = "Voltage : " + voltageLevel;
            });
        }

        /// <summary>
        /// Update the current cartesian graph
        /// </summary>
        /// <param name="currentLevel"></param>
        /// <param name="now"></param>
        private void updateCurrentValues(float currentLevel, DateTime now) {
            Console.WriteLine("Updating current widget");
            this.Dispatcher.Invoke(() => {

                if(currentItemCount > cartesianItemMaxCount) {
                    CurrentChartValues.Clear();
                    currentItemCount = 0;
                } else {
                    currentItemCount += 1;
                }

                CurrentChartValues.Add(new MeasureModel {
                    Value = currentLevel,
                    DateTime = now
                });
                this.currentValueTextBlock.Text = "Current : " + currentLevel;
            });

        }

        /// <summary>
        /// Update the watt cartesian graph
        /// </summary>
        /// <param name="wattLevel"></param>
        /// <param name="now"></param>
        private void updateWattValues(float wattLevel, DateTime now) {
            Console.WriteLine("Updating watt widget - " + wattLevel);
            this.Dispatcher.Invoke(() => {

                if(wattItemCount > cartesianItemMaxCount) {
                    WattChartValues.Clear();
                    wattItemCount = 0;
                } else {
                    wattItemCount += 1;
                }

                WattChartValues.Add(new MeasureModel {
                    Value = wattLevel,
                    DateTime = now
                });
                this.wattValueTextBlock.Text = "Watt : " + wattLevel;
            });
         
        }

        /// <summary>
        ///  Update the energy cartesian graph
        /// </summary>
        /// <param name="energyLevel"></param>
        /// <param name="now"></param>
        private void updateEnergyValues(float energyLevel, DateTime now) {
            Console.WriteLine("Updating enerty widget");
            this.Dispatcher.Invoke(() => {

                if(energyItemCount > cartesianItemMaxCount) {
                    EnergyChartValues.Clear();
                    energyItemCount = 0;
                } else {
                    energyItemCount += 1;
                }

                EnergyChartValues.Add(new MeasureModel {
                    Value = energyLevel,
                    DateTime = now
                });
                this.energyValueTextBlock.Text = "Energy/WattHour : " + energyLevel;
            });
        }

        /// <summary>
        /// Update the pressure gauge
        /// </summary>
        /// <param name="pressureLevel"></param>
        private void updatePressureValues(float pressureLevel) {
            Console.WriteLine("Updating pressure widget");
            this.Dispatcher.Invoke(() => {
                this.fuelCellPressureGauge.Value = (int)(pressureLevel * 100);
            });
        }

        /// <summary>
        /// Update the 4 temperatures values
        /// </summary>
        /// <param name="temperatures"></param>
        /// <param name="now"></param>
        private void updateTemperaturesValues(float[] temperatures, DateTime now) {
            // Update all 4 temperatures
            this.Dispatcher.Invoke(() => {
                this.TemperatureOneTextBlock.Text = temperatures[0].ToString();
                this.TemperatureTwoTextBlock.Text = temperatures[1].ToString();
                this.TemperatureThreeTextBlock.Text = temperatures[2].ToString();
                this.TemperatureFourTextBlock.Text = temperatures[3].ToString();
            });

            this.Dispatcher.Invoke(() => {
                if(temperatures[0] > highestTemp1) {
                    highestTemp1 = temperatures[0];
                    this.HighestTemperatureOneTextBlock.Text = temperatures[0].ToString();
                }
                if (temperatures[1] > highestTemp2) {
                    highestTemp2 = temperatures[1];
                    this.HighestTemperatureTwoTextBlock.Text = temperatures[1].ToString();
                }
                if (temperatures[2] > highestTemp3) {
                    highestTemp3 = temperatures[2];
                    this.HighestTemperatureThreeTextBlock.Text = temperatures[2].ToString();
                }
                if (temperatures[3] > highestTemp4) {
                    highestTemp4 = temperatures[3];
                    this.HighestTemperatureFourTextBlock.Text = temperatures[3].ToString();
                }

            });
        }

        /// <summary>
        /// Save button clicked
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void StartSaveButton_Click(object sender, RoutedEventArgs e) {
            saveData = true;
            dataLogger = new DataLogger(); // Create a new DataLogger Object
            this.startSaveButton.Content = "Saving ...";
            this.startSaveButton.IsEnabled = false;
            this.stopSaveButton.IsEnabled = true;
            this.loggedDataTextBox.Background = Brushes.DarkGreen;
            this.loggingDataStatusTextBlock.Text = "Logging Data ... ";
        }

        /// <summary>
        /// Stop Save button clicked
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void StopSaveButton_Click(object sender, RoutedEventArgs e) {
            saveData = false;
            dataLogger.closeDataLogger();
            dataLogger = null; // Set data logger object to null
            this.stopSaveButton.IsEnabled = false;
            this.startSaveButton.IsEnabled = true;
            this.startSaveButton.Content = "Save";
            this.loggedDataTextBox.Background = Brushes.LightGray;
            this.loggingDataStatusTextBlock.Text = "Logging Off";
        }
    }
}
