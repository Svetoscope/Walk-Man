import sys
import threading
import time
from queue import Queue
from ComArduinoPi import ComArduinoPi
from UserInterface import UserInterface
from PyQt5.QtWidgets import QApplication


class Control:
    def __init__(self):
        # Initialisation for the communication
        self.comArduinoPi = ComArduinoPi(com_port="COM5")
        self.comArduinoPi.start()

    def send_message(self, message: str):
        # List of valid messages :
        # <R> : Start a right leg sequence
        # <L> : Start a left leg sequence
        # <S> : Start a standing sequence
        # <F> : Ask for feedback
        self.comArduinoPi.send_to_arduino(message)

    def get_answer(self) -> str:
        # List of valid answers
        # <R> : Executing a right leg sequence
        # <L> : Executing a left leg sequence
        # <S> : Executing a standing sequence
        # <N> : No sequence being executed
        return self.comArduinoPi.msg_received


def msg_switch_case(arduino_answer) -> int:
    if arduino_answer == 'R' or arduino_answer == 'L' or arduino_answer == 'S':
        # Wait for the sequence to be finished
        return 0
    elif arduino_answer == 'N':
        # Ready to start another sequence
        return 1
    else:
        # Error
        return -1


def scan_ui_queue(self, my_queue):
    while True:
        if self.UI_queue.empty():
            break
        else:
            my_queue.put(self.UI_queue.get())


def run_user_interface(ui_closed_flag, ui_queue, ui_start_flag, ui_update_flag):
    # Initialisation of the user interface
    app = QApplication([])
    window = UserInterface(ui_queue, ui_start_flag, ui_update_flag)
    window.show()
    app.exec_()
    ui_closed_flag.set()


def main1() -> None:
    # main1 is a test main implementing the ui and the communication arduino-pi

    # Creation of variables linked to the thread of the ui
    ui_queue = Queue()
    ui_update_flag = threading.Event()
    ui_start_flag = threading.Event()
    ui_closed_flag = threading.Event()
    ui_thread = threading.Thread(target=run_user_interface, args=(ui_closed_flag, ui_queue, ui_start_flag, ui_update_flag))
    ui_thread.start()

    controller = Control()

    error_count = 0

    # INIT COM
    message = 'F'
    controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)
    message = 'N'
    controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)
    answer = controller.get_answer()
    print("Arduino reply OUT: ", answer)

    while not ui_closed_flag.is_set():
        if ui_start_flag.is_set() and not ui_queue.empty() and controller.comArduinoPi.new_msg_flag:
            controller.comArduinoPi.new_msg_flag = False
            answer = controller.get_answer()
            print("Arduino reply OUT: ", answer)
            state = msg_switch_case(answer)

            if message != 'F' and answer != message:  # Means last sequence asked didn't start
                error_count += 1
            else:
                if state == 1:  # Ready to start another sequence
                    error_count = 0
                    message = ui_queue.get()
                    ui_update_flag.set()
                elif state == 0:  # Wait for the sequence to be finished
                    error_count = 0
                    message = 'F'
                else:
                    print('Invalid Arduino answer')
                    error_count += 1
                    message = 'F'

            if error_count > 3:
                break

            controller.send_message(message)
            print("Message sent : ", message)
        time.sleep(1)

    print('DONE')
    ui_thread.join()
    return


def main2() -> None:
    # main2 is a test main implementing only the ui

    # Creation of variables linked to the thread of the ui
    ui_queue = Queue()
    ui_update_flag = threading.Event()
    ui_start_flag = threading.Event()
    ui_closed_flag = threading.Event()
    ui_thread = threading.Thread(target=run_user_interface, args=(ui_closed_flag, ui_queue, ui_start_flag, ui_update_flag))
    ui_thread.start()

    # controller = Control()

    error_count = 0

    # INIT COM
    message = 'F'
    # controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)

    while not ui_closed_flag.is_set():
        print('In while')
        print(ui_start_flag.is_set())
        # if start_flag and not ui_queue.empty() and controller.comArduinoPi.new_msg_flag:
        if ui_start_flag.is_set() and not ui_queue.empty():
            """

            controller.comArduinoPi.new_msg_flag = False
            answer = controller.get_answer()
            print("Arduino reply OUT: ", answer)
            state = msg_switch_case(answer)

            if message != 'F' and answer != message:  # Means last sequence asked didn't start
                error_count += 1
            else:
                if state == 1:  # Ready to start another sequence
                    error_count = 0
                    message = ui_queue.get()
                elif state == 0:  # Wait for the sequence to be finished
                    error_count = 0
                    message = 'F'
                else:
                    print('Invalid Arduino answer')
                    error_count += 1
                    message = 'F'

            if error_count > 3:
                break
            """

            print('Processing')
            message = ui_queue.get()
            ui_update_flag.set()
            # controller.send_message(message)
            print("Message sent : ", message)

        time.sleep(1)

    print('DONE')
    ui_thread.join()
    return


def main3() -> None:
    # main1 is a test main implementing only the communication arduino-pi

    # Creation of variables replacing the ui
    controller = Control()
    sequence_queue = Queue()

    sequence_queue.put('R')
    sequence_queue.put('L')
    sequence_queue.put('R')
    sequence_queue.put('L')
    sequence_queue.put('S')
    sequence_queue.put('R')
    sequence_queue.put('L')
    sequence_queue.put('S')

    error_count = 0

    message = 'F'
    controller.send_message(message)
    print("Message sent : ", message)
    time.sleep(1)
    message = sequence_queue.get()

    while True:
        message = sequence_queue.get()
        controller.send_message(message)
        print("Message sent : ", message)

        time.sleep(0.5)

        answer = controller.get_answer()
        print("Arduino reply OUT: ", answer)

        time.sleep(0.5)
        """
        if controller.comArduinoPi.new_msg_flag:
            controller.comArduinoPi.new_msg_flag = False
            answer = controller.get_answer()
            print("Arduino reply OUT: ", answer)
            state = msg_switch_case(answer)

            if message != 'F' and answer != message:  # Means last sequence asked didn't start
                error_count += 1
            else:
                if state == 1:  # Ready to start another sequence
                    error_count = 0
                    message = sequence_queue.get()
                elif state == 0:  # Wait for the sequence to be finished
                    error_count = 0
                    message = 'F'
                else:
                    print('Invalid Arduino answer')
                    error_count += 1
                    message = 'F'

            if error_count > 3:
                break

            controller.send_message(message)
            print("Message sent : ", message)

        time.sleep(0.05)

        if sequence_queue.empty():
            break
        """

    controller.comArduinoPi.terminate()
    return


if __name__ == "__main__":
    main3()
