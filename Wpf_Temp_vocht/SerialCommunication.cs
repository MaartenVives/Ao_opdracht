using System;
using System.IO.Ports;

namespace Wpf_Temp_vocht
{
    public class SerialCommunication
    {
        private SerialPort serialPort;
        public event EventHandler<string>? DataReceived; // Markeer het event als nullable
        SerialDataReceivedEventHandler myHandler;
        public SerialCommunication(string portName, int baudRate)
        {
            serialPort = new SerialPort(portName, baudRate);
            myHandler = new SerialDataReceivedEventHandler(SerialPort_DataReceived);
        }

        public void Open()
        {
            if (!serialPort.IsOpen)
            {
                serialPort.Open();
                serialPort.DataReceived += myHandler;
            }
        }

        public void Close()
        {
            if (serialPort.IsOpen)
            {
                serialPort.DataReceived -= myHandler;
                serialPort.Close();
            }
        }

        public string ReadLine()
        {
            return serialPort.ReadLine();
        }

        public bool IsOpen()
        {
            return serialPort.IsOpen;
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = serialPort.ReadLine();
                DataReceived?.Invoke(this, data); // Roep het DataReceived event aan en stuur de ontvangen gegevens mee
            }
            catch (OperationCanceledException)
            {
            }
        }
    }
}