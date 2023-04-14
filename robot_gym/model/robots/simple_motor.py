import collections
import numpy as np

# Use a PD controller
MOTOR_CONTROL_POSITION = 1
# Apply motor torques directly.
MOTOR_CONTROL_TORQUE = 2
# Apply a tuple (q, qdot, kp, kd, tau) for each motor. Here q, qdot are motor
# position and velocities. kp and kd are PD gains. tau is the additional
# motor torque. This is the most flexible control mode.
MOTOR_CONTROL_HYBRID = 3
# PWM control (only for Minitaur)
MOTOR_CONTROL_PWM = 4

MOTOR_COMMAND_DIMENSION = 5

# These values represent the indices of each field in the motor command tuple
POSITION_INDEX = 0
POSITION_GAIN_INDEX = 1
VELOCITY_INDEX = 2
VELOCITY_GAIN_INDEX = 3
TORQUE_INDEX = 4


class RobotMotorModel:
    """A simple motor model.
      When in POSITION mode, the torque is calculated according to the difference
      between current and desired joint angle, as well as the joint velocity.
      For more information about PD control, please refer to:
      https://en.wikipedia.org/wiki/PID_controller.
      The model supports a HYBRID mode in which each motor command can be a tuple
      (desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain,
      torque).
    """
    def __init__(self,
                 num_motors,
                 kp,
                 kd,
                 torque_limits=None,
                 motor_control_mode=MOTOR_CONTROL_POSITION):
        self._num_motors = num_motors
        self._kp = kp
        self._kd = kd
        self._torque_limits = torque_limits
        if torque_limits is not None:
            if isinstance(torque_limits, (collections.Sequence, np.ndarray)):
                self._torque_limits = np.asarray(torque_limits)
            else:
                self._torque_limits = np.full(self._num_motors, torque_limits)
        self._motor_control_mode = motor_control_mode
        self._strength_ratios = np.full(self._num_motors, 1)

        self.time = 0
        '''
        for num in range(self._num_motors):
          f = open(f"Q:\ROBOTHUND_TEST\plot_test\data\motor{num}.txt", "w") # Glöm ej att ändra
          f.write("")
          f.close()
        '''
        

    def set_strength_ratios(self, ratios):
        """Set the strength of each motors relative to the default value.
        Args:
          ratios: The relative strength of motor output. A numpy array ranging from
            0.0 to 1.0.
        """
        self._strength_ratios = ratios

    def set_motor_gains(self, kp, kd):
        """Set the gains of all motors.
        These gains are PD gains for motor positional control. kp is the
        proportional gain and kd is the derivative gain.
        Args:
          kp: proportional gain of the motors.
          kd: derivative gain of the motors.
        """
        self._kp = kp
        self._kd = kd

    def set_voltage(self, voltage):
        pass

    def get_voltage(self):
        return 0.0

    def set_viscous_damping(self, viscous_damping):
        pass

    def get_viscous_dampling(self):
        return 0.0

    def convert_to_torque(self,
                          motor_commands,
                          motor_angle,
                          motor_velocity,
                          true_motor_velocity,
                          motor_control_mode=None):
        """Convert the commands (position control or torque control) to torque.
        Args:
          motor_commands: The desired motor angle if the motor is in position
            control mode. The pwm signal if the motor is in torque control mode.
          motor_angle: The motor angle observed at the current time step. It is
            actually the true motor angle observed a few milliseconds ago (pd
            latency).
          motor_velocity: The motor velocity observed at the current time step, it
            is actually the true motor velocity a few milliseconds ago (pd latency).
          true_motor_velocity: The true motor velocity. The true velocity is used to
            compute back EMF voltage and viscous damping.
          motor_control_mode: A MotorControlMode enum.
        Returns:
          actual_torque: The torque that needs to be applied to the motor.
          observed_torque: The torque observed by the sensor.
        """
        del true_motor_velocity
        if not motor_control_mode:
            motor_control_mode = self._motor_control_mode

        # No processing for motor torques
        if motor_control_mode is MOTOR_CONTROL_TORQUE:
            assert len(motor_commands) == self._num_motors
            motor_torques = self._strength_ratios * motor_commands
            return motor_torques, motor_torques

        desired_motor_angles = None
        desired_motor_velocities = None
        kp = None
        kd = None
        additional_torques = np.full(self._num_motors, 0)
        if motor_control_mode is MOTOR_CONTROL_POSITION:
            assert len(motor_commands) == self._num_motors
            kp = self._kp
            kd = self._kd
            desired_motor_angles = motor_commands
            desired_motor_velocities = np.full(self._num_motors, 0)
        elif motor_control_mode is MOTOR_CONTROL_HYBRID:
            # The input should be a 60 dimension vector
            #motor_commands = motor_commands[0]
            assert len(motor_commands) == MOTOR_COMMAND_DIMENSION * self._num_motors
            kp = motor_commands[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
            kd = motor_commands[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
            desired_motor_angles = motor_commands[
                                   POSITION_INDEX::MOTOR_COMMAND_DIMENSION]
            desired_motor_velocities = motor_commands[
                                       VELOCITY_INDEX::MOTOR_COMMAND_DIMENSION]
            additional_torques = motor_commands[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION]
        motor_torques = -1 * (kp * (motor_angle - desired_motor_angles)) - kd * (
                motor_velocity - desired_motor_velocities) + additional_torques
        motor_torques = self._strength_ratios * motor_torques

        #The code below prints debug values
        '''
        print("q")
        print(desired_motor_angles)
        print(" ")
        print("qdot")
        print(motor_velocity)
        print(" ")
        
        print("kp")
        print(kp)
        print(" ")
        print(kd)
        print(" ")
        
        print("tau")
        print(motor_torques)
        print(" ")
        print(" ")
        print(" ")
        '''

        '''
        #The code below writes the debug values to files
        for num in range(len(desired_motor_angles)):
          f = open(f"Q:\ROBOTHUND_TEST\plot_test\data\motor{num}.txt", "a") # Glöm ej att ändra
          f.write(f"{self.time} {desired_motor_angles[num]} {motor_velocity[num]} {motor_torques[num]} {kp[num]} {kd[num]}\n")
          f.close()
        '''
        self.time = self.time + 1
		
        if self._torque_limits is not None:
            if len(self._torque_limits) != len(motor_torques):
                raise ValueError(
                    "Torque limits dimension does not match the number of motors.")
            motor_torques = np.clip(motor_torques, -1.0 * self._torque_limits,
                                    self._torque_limits)

        return motor_torques, motor_torques