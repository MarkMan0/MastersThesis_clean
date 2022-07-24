import simulation
import serial_commands
import math
import threading
import visualisation


def serial_cmd_1(motor_id: int, arg: bytes, motors: tuple) -> bytes:
    spd = int.from_bytes(arg, 'little', signed=True)
    spd = spd / 32767 * 720
    motors[motor_id].ang_speed_target = spd
    return b'\x01\x00\x00'


def serial_cmd_2(motor_id: int, arg: bytes, motors: tuple) -> bytes:
    angle = int(motors[motor_id].angle / 180 * 32767)
    b = int.to_bytes(angle, 2, 'little', signed=True)
    return b'\x02' + b


def serial_cmd_3(rob: simulation.Robot, vis: visualisation.Visualisation) -> bytes:
    rob.reset()
    vis.reset()
    return b'\x03'


def main():
    commands = serial_commands.SerialCommands(10, 115200)

    sim = simulation.Robot()
    vis = visualisation.Visualisation()
    vis.simulation = sim
    motors = (sim.l_motor, sim.r_motor, )
    print(f"Tl: {motors[0]._time_const}, Tr: {motors[1]._time_const}")

    commands.commands[1] = lambda motor_id, arg: serial_cmd_1(
        motor_id, arg, motors)
    commands.commands[2] = lambda motor_id, arg: serial_cmd_2(
        motor_id, arg, motors)
    commands.commands[3] = lambda _, __: serial_cmd_3(sim, vis)

    tsim = sim.create_threads()
    tserial = threading.Thread(target=commands.thread)
    tvis = threading.Thread(target=vis.thread)

    for t in tsim:
        t.start()
    tserial.start()
    tvis.start()

    for t in tsim:
        t.join()
    tvis.join()
    # we get here only if user clicked close on window
    commands.stop()
    tserial.join()


if __name__ == '__main__':
    main()
