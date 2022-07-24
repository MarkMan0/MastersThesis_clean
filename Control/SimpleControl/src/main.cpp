#include <iostream>
#include <memory>

#include "RobotControl/VirtualServo.h"
#include <thread>
#include "RobotControl/SimpleMotion.h"
#include "RobotControl/Navigation.h"
#include "RobotControl/MotionQueue.h"
#include "RobotControl/PlanExecutor.h"
#include "RobotControl/LoopRate.h"
#include "RobotControl/geometry.h"
#include <tuple>
#include "SerialPortWrapper/SerialPortWrapper.h"

#include "measurement.h"

// #define USE_REAL_ROBOT

using std::this_thread::sleep_for;
using namespace std::chrono_literals;

void simulation_init(std::shared_ptr<SerialPortWrapper>& port);


int main(int argc, char* argv[]) {
  std::shared_ptr<Servo> servo_l, servo_r;


  /********************* INITIALIZE SERVO ********************/

#ifdef USE_REAL_ROBOT
  #error "Drivebot code was deleted from public repository"
#else
  std::shared_ptr<SerialPortWrapper> port;
  try {
    simulation_init(port);
  } catch (const std::runtime_error& ex) {
    std::cout << ex.what() << '\n';
    return 1;
  }
  servo_r = std::make_shared<VirtualServo>(1, port);
  servo_l = std::make_shared<VirtualServo>(0, port);
#endif


  /** ************** INITIALIZE CLASSES *******************/

  SimpleMotion motion(servo_l, servo_r);
#ifdef USE_REAL_ROBOT
  motion.pos_.add_gyro(&gyro);
#endif
  planning::PlanExecutor planner(&motion);

  try {
    motion.init();
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << '\n';
    return 1;
  }


  /** ************ START THREAD *********************/
  bool thread_stop = false;
  auto thr{ std::thread([&motion, &thread_stop]() {
    LoopRate rate(0.05);
    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    while (!thread_stop) {
      try {
        const auto now = clock::now();
        const double dt = std::chrono::duration<double, std::ratio<1, 1>>(now - last).count();
        last = now;
        motion.tick(dt);
      } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        thread_stop = true;
      }
      rate.sleep();
    }
  }) };


  measure::measure_bezier_1(planner);

  /** ************ SHUTDOWN **************/
  using namespace std::chrono_literals;
  sleep_for(200ms);
  thread_stop = true;
  sleep_for(200ms);
  thr.join();

  return 0;
}




static void reset_simulation(SerialPortWrapper& port) {
  uint8_t msg[4]{ 0 };
  msg[1] = 0x03;

  port.write(msg, 4);

  msg[0] = 0;
  using namespace std::chrono_literals;
  const auto start = std::chrono::steady_clock::now();

  while (msg[0] == 0) {
    // read byte-by-byte
    port.read(msg, 1);
    if (std::chrono::steady_clock::now() - start > 500ms) {
      port.flush();
      return;
    }
  }
  port.flush();
}


void simulation_init(std::shared_ptr<SerialPortWrapper>& port) {
  int port_no = 9;

  port = std::make_shared<SerialPortWrapper>(port_no, 115200);

  if (!port->check()) {
    throw std::runtime_error("Failed to open port");
  }

  reset_simulation(*port);
}
