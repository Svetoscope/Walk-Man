from PyQt5.QtCore import QThread, pyqtSignal
import serial as pyserial
import serial.tools.list_ports

class ComArduino(QThread):
    text_received = pyqtSignal(str)
    START_MARKER = '<'
    END_MARKER = '>'
    data_started = False
    data_buf = ""
    message_complete = False
    connected = True
    msg_received = "XXX"
    new_msg_flag = False

    def __init__(self, com_port: str = "COM1", baud_rate: int = 115200):
        """
        Initialisation of the serial communication

        :param com_port:
        :param baud_rate:
        """
        QThread.__init__(self)
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print("Serial port desc :", port.description)
            if "Arduino" in port.description or "OpenRB" in port.description or "USB" in port.description:
                com_port = port.device
        
        try:
            self.serial = pyserial.Serial(com_port, baud_rate, timeout=0, rtscts=True)
            print("Serial port " + com_port + " opened  Baudrate " + str(baud_rate))
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.connected = False

    def run(self):
        """
        Continuous loop reading the messages sent by the Arduino

        :return:
        """

        while True:
            if self.connected:
                arduino_reply = self.recv_like_arduino()
                if not (arduino_reply == 'XXX'):
                    self.msg_received = arduino_reply
                    self.new_msg_flag = True

    def kill(self):
        """
        Terminating function

        :return:
        """
        self.terminate()

    def send_to_arduino(self, string_to_send):
        """
        Frames the messages to respect the communication protocol established

        :param string_to_send:
        :return:
        """

        string_with_markers = self.START_MARKER
        string_with_markers += string_to_send
        string_with_markers += self.END_MARKER

        self.serial.write(string_with_markers.encode('utf-8'))  # encode needed for Python3

    def recv_like_arduino(self):
        """
        Function analyses the message received from the Arduino

        :return:
        """

        if self.serial.inWaiting() > 0 and self.message_complete is False:  # type: ignore
            x = self.serial.read().decode("utf-8")  # decode needed for Python3

            if self.data_started is True:
                if x != self.END_MARKER:
                    self.data_buf = self.data_buf + x
                else:
                    self.data_started = False
                    self.message_complete = True
            elif x == self.START_MARKER:
                self.data_buf = ''
                self.data_started = True

        if self.message_complete is True:
            self.message_complete = False
            return self.data_buf
        else:
            return "XXX"


