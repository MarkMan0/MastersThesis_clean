import serial
import time


class SerialCommands:

    def __init__(self, port_no, baud) -> None:
        self.ser = serial.Serial(f'COM{port_no}', baud)
        self.ser.timeout = 1
        self.ser.write_timeout = 1
        self.callback = None
        self.running = True
        self.commands = {}

    def stop(self) -> None:
        self.running = False

    def tick(self) -> None:
        msg = self.ser.read(4)

        if len(msg) < 4:
            # invalid command, clear serial
            try:
                self.ser.write(1)
            except serial.SerialTimeoutException:
                pass
            time.sleep(0.05)
            self.ser.flush()
            return

        motor_id = int(msg[0])
        cmd = msg[1]
        argument = msg[2:4]
        if cmd in self.commands:
            retbytes = self.commands[cmd](motor_id, argument)
            self.ser.write(retbytes)

    def thread(self):

        while self.running:
            self.tick()
        self.ser.close()
