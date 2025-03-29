#include "sensors.h"
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/locale/encoding.hpp>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#define POS 0.0015339807878
#define ANG 0.000214757310

// Yada Yada Bad Code :(

static const double K[4] = {1.0, 2.0, 3.0, 4.0};

/* Sampling time [s], e.g., 1 kHz loop -> T = 0.001 */
static constexpr double T = 0.02;

/* Example saturations for the cart's physical movement or motor torque. */
static const double CART_MAX_POS = 1.0; // 1 meter from center
static const double CART_MIN_POS = -1.0;
static const double MOTOR_MAX_TORQUE = 10.0;
static const double MOTOR_MIN_TORQUE = -10.0;

Eigen::Matrix4d A_d;
Eigen::Matrix4d Q;
Eigen::Matrix4d H;
Eigen::Matrix4d P_est;
Eigen::Matrix4d R;
Eigen::Vector4d B_d;
Eigen::Vector4d x_est;
Eigen::Vector4d K_HAC;


typedef struct {
  double x;      // cart position
  double dx;     // cart velocity
  double theta;  // pendulum angle
  double dtheta; // pendulum angular velocity
} PendulumState;

enum AxisState {
  AXIS_STATE_UNDEFINED = 0,
  AXIS_STATE_IDLE = 1,
  AXIS_STATE_STARTUP_SEQUENCE = 2,
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
  AXIS_STATE_MOTOR_CALIBRATION = 4,
  AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
  AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
  AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
  AXIS_STATE_LOCKIN_SPIN = 9,
  AXIS_STATE_ENCODER_DIR_FIND = 10,
  AXIS_STATE_HOMING = 11,
  AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
  AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13
};

class SimpleODrive {
private:
#if BOOST_VERSION >= 106600    
  boost::asio::io_context io;
#else
  booboost::asio::io_service io;
#endif
  boost::asio::serial_port serial;

  void write(std::string s) {
    std::string ns = boost::locale::conv::to_utf<char>(s + "\n", "UTF-8");
    boost::asio::write(serial, boost::asio::buffer(ns.c_str(), ns.size()));
  }

public:
  SimpleODrive(std::string port, unsigned int baud_rate)
      : io(), serial(io, port) {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
  }

  void init_odrive() {
    enum AxisState calibration = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
    enum AxisState CLC = AXIS_STATE_CLOSED_LOOP_CONTROL;

    std::string command_calibrate =
        "w axis0.requested_state " + std::to_string(calibration);
    std::string command_clc = "w axis0.requested_state " + std::to_string(CLC);

    write(command_calibrate);
    boost::this_thread::sleep_for(boost::chrono::seconds(15));
    write(command_clc);
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
  }

  void set_torque(float tq) {
    std::string temp = "c 0 " + std::to_string(tq);

    write(temp);
  }
};

void estimateState(PendulumState *state, const SensorData *sensor,
                   const double T, double force) {

  // Update previous values
  double prev_position = state->x;
  double prev_angle = state->theta;

  // Construct sensor measurements
  double position = (sensor->cartPosition);
  double velocity =
      (position - prev_position) / T; // (position - previous_position) / dt
  double angle = (sensor->pendulumAngle);
  double angular_velocity =
      (angle - prev_angle) / T; // (angle - previous_angle) / dt

  // Construct the measurement vector
  Eigen::Vector4d y;
  y << position, velocity, angle, angular_velocity;

  Eigen::Vector4d temp_x;

  temp_x << state->x, state->dx, state->theta, state->dtheta;

  // Prediction Step
  Eigen::Vector4d x_pred = A_d * temp_x + B_d * force; // Assume u = 0 for now
  Eigen::Matrix4d P_pred = A_d * P_est * A_d.transpose() + Q;

  // Compute Kalman Gain
  Eigen::Matrix4d K =
      P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();

  // Measurement Update
  Eigen::Vector4d x_updated = x_pred + K * (y - H * x_pred);

  // Assign values back to state
  state->x = x_updated(0);
  state->dx = x_updated(1);
  state->theta = x_updated(2);
  state->dtheta = x_updated(3);

  // Update P_est
  P_est = (Eigen::Matrix4d::Identity() - K * H) * P_pred;
}

double computeControl(const PendulumState *state, double force) {
  Eigen::Vector4d temp_x;

  temp_x << state->x, state->dx, state->theta, state->dtheta;

  auto temp = A_d * temp_x + B_d * (force / 0.014);
  force = temp.dot(K_HAC) * 0.014;
  return force;
}

SimpleODrive com("/dev/ttyACM0", 115200);

/************************************************************
 * Main Function
 ************************************************************/
int main(void) {
/* In a real system, you might initialize hardware,
   set up timers, open ports, etc. */
#ifdef DEBUG
  printf("Initializing Inverted Pendulum Controller...\n");
#endif

  PendulumState currentState = {0};
  SensorData sensorData = {0};

  A_d << 1.0000, 0.0190, 0.0001, -0.0000, 0, 0.9059, 0.0053, -0.0012, 0,
      -0.0022, 1.0047, 0.0189, 0, -0.2151, 0.4612, 0.8926;

  B_d << 0.0001, 0.0094, 0.0002, 0.0215;

  Q = Eigen::Matrix4d::Identity() * 0.1;

  R << 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0.01;

  H = Eigen::Matrix4d::Identity();

  x_est << 0.0, 0.0, 0.0, 0.0;

  P_est = Eigen::Matrix4d::Identity() * 0.1;

  K_HAC << -4, -14, 68, 7;

  double u = 0;

  SensorController<EncoderController, ButtonController> sensor_controller(
      EncoderController("/dev/uio6"),
      ButtonController("/dev/uio4", "/dev/uio5"));

  com.init_odrive();

  for (int iteration = 0; iteration < 10000; iteration++) {
    /************************************************************
     * 1) Read Sensor Data
     ************************************************************/

    if (sensorData.leftBoundary) {
#ifdef DEBUG
      printf("[DEBUG] Left boundary sensor triggered.\n");
#endif
    }
    if (sensorData.rightBoundary) {
#ifdef DEBUG
      printf("[DEBUG] Right boundary sensor triggered.\n");
#endif
    }

    sensor_controller.handle(sensorData);

    /************************************************************
     * 2) Estimate / Update State
     ************************************************************/
    estimateState(&currentState, &sensorData, T, u);

#ifdef DEBUG
    printf("[DEBUG] Measured Position: %f, Angle: %f\n",
           sensorData.cartPosition, sensorData.pendulumAngle);
    printf("[DEBUG] Estimated State -> x: %f, dx: %f, theta: %f, dtheta: %f\n",
           currentState.x, currentState.dx, currentState.theta,
           currentState.dtheta);
#endif

    /************************************************************
     * 3) Compute Control
     ************************************************************/
    u = computeControl(&currentState, u);

    /* Safety check for saturations or boundary conditions */
    if (currentState.x >= CART_MAX_POS && u > 0.0) {
      /* Prevent pushing further right if at boundary */
      u = 0.0;
    } else if (currentState.x <= CART_MIN_POS && u < 0.0) {
      /* Prevent pushing further left if at boundary */
      u = 0.0;
    }

    /* Motor torque saturation check */
    if (u > MOTOR_MAX_TORQUE) {
      u = MOTOR_MAX_TORQUE;
    } else if (u < MOTOR_MIN_TORQUE) {
      u = MOTOR_MIN_TORQUE;
    }

    /************************************************************
     * 4) Actuate
     ************************************************************/
    com.set_torque(u);
  }

#ifdef DEBUG
  printf("Control loop ended. Exiting...\n");
#endif

  return 0;
}
