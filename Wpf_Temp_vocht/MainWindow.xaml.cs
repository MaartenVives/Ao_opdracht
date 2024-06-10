using System;
using System.IO.Ports;
using System.Windows;

namespace Wpf_Temp_vocht
{
    public partial class MainWindow : Window
    {
        private SerialCommunication? serialCommunication;

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPort();
        }

        private void InitializeSerialPort()
        {
            string[] ports = SerialPort.GetPortNames();
            foreach (string port in ports)
            {
                comPortComboBox.Items.Add(port);
            }
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (comPortComboBox.SelectedItem != null)
            {
                try
                {
                    string? selectedPort = comPortComboBox.SelectedItem.ToString();
                    if (!string.IsNullOrEmpty(selectedPort))
                    {
                        serialCommunication = new SerialCommunication(selectedPort, 9600);
                        serialCommunication.Open();
                        serialCommunication.DataReceived += SerialCommunication_DataReceived;
                        connectButton.IsEnabled = false;
                        disconnectButton.IsEnabled = true;
                    }
                    else
                    {
                        MessageBox.Show("Invalid COM port selected.");
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error opening serial port: " + ex.Message);
                }
            }
            else
            {
                MessageBox.Show("Please select a COM port.");
            }
        }

        private void DisconnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (serialCommunication != null && serialCommunication.IsOpen())
            {
                serialCommunication.DataReceived -= SerialCommunication_DataReceived;
                serialCommunication.Close();
                connectButton.IsEnabled = true;
                disconnectButton.IsEnabled = false;
            }
        }

        private void SerialCommunication_DataReceived(object? sender, string data)
        {
            if (sender != null)
            {
                Dispatcher.Invoke(() =>
                {
                    ParseAndDisplayData(data);
                });
            }
        }

        private void ParseAndDisplayData(string data)
        {
            string[] lines = data.Split('\r');
            foreach (string line in lines)
            {
                if (line.StartsWith("Temperatuur:"))
                {
                    string[] parts = line.Split(':');
                    if (parts.Length == 2)
                    {
                        temperatureTextBox.Text = parts[1].Trim() + " °C";
                    }
                }
                else if (line.StartsWith("Vochtigheid:"))
                {
                    string[] parts = line.Split(':');
                    if (parts.Length == 2)
                    {
                        humidityTextBox.Text = parts[1].Trim() + " %";
                    }
                }
            }
        }

       
    }
}
