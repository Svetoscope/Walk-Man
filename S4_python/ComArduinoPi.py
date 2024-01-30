from PyQt5.QtCore import QThread, pyqtSignal
import serial as pyserial


class ComArduinoPi(QThread):
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
        Initialisation de la communication sérielle et du fil d'exécution

        :param com_port:
        :param baud_rate:
        """
        QThread.__init__(self)

        try:
            self.serial = pyserial.Serial(com_port, baud_rate, timeout=0, rtscts=True)
            print("Serial port " + com_port + " opened  Baudrate " + str(baud_rate))
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.connected = False

    def run(self):
        """
        Boucle du fil, s'exécute en continu
        Ici on lit les messages reçu de l'arduino et on notifie l'interface lors d'un changement

        :return:
        """

        while True:
            if self.connected:
                arduino_reply = self.recv_like_arduino()
                if not (arduino_reply == 'XXX'):
                    self.msg_received = arduino_reply
                    print("Arduino reply IN: ", arduino_reply)
                    self.new_msg_flag = True
                    # self.text_received.emit(f'arduinoreply : {arduino_reply}')

    def kill(self):
        """
        Fonction qui met fin au fil

        :return:
        """
        self.terminate()

    def send_to_arduino(self, string_to_send):
        """
        Ajoute des marqueurs de début et de fin et envoit le message à l'arduino

        :param string_to_send:
        :return:
        """

        # print(string_to_send)
        # string_with_markers = self.START_MARKER
        string_with_markers = string_to_send
        # string_with_markers += self.END_MARKER

        self.serial.write(string_with_markers.encode('utf-8'))  # encode needed for Python3

    def recv_like_arduino(self):
        """
        Fonction permettant de lire la réponse de l'arduino

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


