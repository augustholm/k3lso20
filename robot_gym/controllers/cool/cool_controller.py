import numpy as np
from robot_gym.controllers.controller import Controller
from robot_gym.controllers.cool import kinematics
from robot_gym.model.robots import simple_motor


class CoolController(Controller):

    MOTOR_CONTROL_MODE = simple_motor.MOTOR_CONTROL_HYBRID#MOTOR_CONTROL_POSITION #BYT?

    def __init__(self, robot, get_time_since_reset):
        super(CoolController, self).__init__(robot, get_time_since_reset)
        self._constants = robot.GetCtrlConstants()
        self._commands = [0, 0, 0, 0, 0]
        self._frames = np.asmatrix([[self._constants.x_dist / 2, -self._constants.y_dist / 2, -self._constants.height],
                                    [self._constants.x_dist / 2, self._constants.y_dist / 2, -self._constants.height],
                                    [-self._constants.x_dist / 2, -self._constants.y_dist / 2, -self._constants.height],
                                    [-self._constants.x_dist / 2, self._constants.y_dist / 2, -self._constants.height]])

    def update_controller_params(self, params):
        self._commands = params

    def setup_ui_params(self, pybullet_client):
        hip_button = pybullet_client.addUserDebugParameter("Hip motors:", 0, -1, 0)
        hip_position = pybullet_client.addUserDebugParameter("Position (angle)", -0.5, .5, 0.)
        hip_position_gain = pybullet_client.addUserDebugParameter("Position gain", 0, 150, 0)#5)
        hip_velocity = pybullet_client.addUserDebugParameter("Velocity", 0, 5, 0)
        hip_velocity_gain = pybullet_client.addUserDebugParameter("Velocity gain", 0, 20, 0)#5)
        hip_torque = pybullet_client.addUserDebugParameter("Torque", -20, 20, 0)

        leg_button = pybullet_client.addUserDebugParameter("Leg motors:", 0, -1, 0)
        leg_position = pybullet_client.addUserDebugParameter("Position (angle)", -0.5, .5, 0.)
        leg_position_gain = pybullet_client.addUserDebugParameter("Position gain", 0, 20, 0)#5)
        leg_velocity = pybullet_client.addUserDebugParameter("Velocity", 0, 5, 0)
        leg_velocity_gain = pybullet_client.addUserDebugParameter("Velocity gain", 0, 20, 0)#5)
        leg_torque = pybullet_client.addUserDebugParameter("Torque", -20, 20, 0)

        lower_leg_button = pybullet_client.addUserDebugParameter("Lower leg motors:", 0, -1, 0)
        lower_leg_position = pybullet_client.addUserDebugParameter("Position (angle)", -0.5, .5, 0.)
        lower_leg_position_gain = pybullet_client.addUserDebugParameter("Position gain", 0, 20, 0)#5)
        lower_leg_velocity = pybullet_client.addUserDebugParameter("Velocity", 0, 5, 0)
        lower_leg_velocity_gain = pybullet_client.addUserDebugParameter("Velocity gain", 0, 20, 0)#5)
        lower_leg_torque = pybullet_client.addUserDebugParameter("Torque", -20, 20, 0)

        return hip_position, hip_position_gain, hip_velocity, hip_velocity_gain, hip_torque, leg_position, leg_position_gain, leg_velocity, leg_velocity_gain, leg_torque, lower_leg_position, lower_leg_position_gain, lower_leg_velocity, lower_leg_velocity_gain, lower_leg_torque

    def read_ui_params(self, pybullet_client, ui):
        hip_position, hip_position_gain, hip_velocity, hip_velocity_gain, hip_torque, leg_position, leg_position_gain, leg_velocity, leg_velocity_gain, leg_torque, lower_leg_position, lower_leg_position_gain, lower_leg_velocity, lower_leg_velocity_gain, lower_leg_torque = ui
        hip_data = np.array(
            [
                pybullet_client.readUserDebugParameter(hip_position),
                pybullet_client.readUserDebugParameter(hip_position_gain),
                pybullet_client.readUserDebugParameter(hip_velocity),
                pybullet_client.readUserDebugParameter(hip_velocity_gain),
                pybullet_client.readUserDebugParameter(hip_torque)
            ]
        )

        leg_data = np.array(
            [
                pybullet_client.readUserDebugParameter(leg_position),
                pybullet_client.readUserDebugParameter(leg_position_gain),
                pybullet_client.readUserDebugParameter(leg_velocity),
                pybullet_client.readUserDebugParameter(leg_velocity_gain),
                pybullet_client.readUserDebugParameter(leg_torque)
            ]
        )

        lower_leg_data = np.array(
            [
                pybullet_client.readUserDebugParameter(lower_leg_position),
                pybullet_client.readUserDebugParameter(lower_leg_position_gain),
                pybullet_client.readUserDebugParameter(lower_leg_velocity),
                pybullet_client.readUserDebugParameter(lower_leg_velocity_gain),
                pybullet_client.readUserDebugParameter(lower_leg_torque)
            ]
        )

        return hip_data, leg_data, lower_leg_data

    def reset(self):
        pass

    def get_action(self):
        signal = []

        #(desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain, torque).

        hip_data, leg_data, lower_leg_data = self._commands


        for i in range(4):
            signal.extend(hip_data)
            signal.extend(leg_data)
            signal.extend(lower_leg_data)

            if i == 1:
                j = i*15 + 2*5 + 4
                signal[j] = -signal[j]
                signal[j-4] = -signal[j-4]

            if i in [2, 3]:
                j = i*15 + 1*5 + 4
                signal[j] = -signal[j]
                signal[j-4] = -signal[j-4]

            if i == 3:
                j = i*15 + 2*5 + 4
                signal[j] = -signal[j]
                signal[j-4] = -signal[j-4]
                
        '''signal = [
            front_right_angles[0], front_right_angles[1], front_right_angles[2],
            front_left_angles[0], front_left_angles[1], -front_left_angles[2],
            rear_right_angles[0], -rear_right_angles[1], rear_right_angles[2],
            rear_left_angles[0], -rear_left_angles[1], -rear_left_angles[2]
        ]'''
        print(signal)
        return signal

    """A simple motor model.
      When in POSITION mode, the torque is calculated according to the difference
      between current and desired joint angle, as well as the joint velocity.
      For more information about PD control, please refer to:
      https://en.wikipedia.org/wiki/PID_controller.
      The model supports a HYBRID mode in which each motor command can be a tuple
      (desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain,
      torque).
    """
    #0.20208248215945665, 220.0, 0, 1.0, 0