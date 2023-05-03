"""
An implementation of Model Predictive Control controller using google-research/motion_imitation library.

Credits: https://github.com/google-research/motion_imitation
"""
#hej
from mpc_controller import openloop_gait_generator, com_velocity_estimator, raibert_swing_leg_controller, \
    torque_stance_leg_controller, locomotion_controller

import math
from robot_gym.controllers.controller import Controller
from robot_gym.controllers.mpc.kinematics import Kinematics
from robot_gym.model.robots import simple_motor

DESTINATION_VECTOR = [[0, 0], [0, 0]]
SPEED_VECTOR = [0.5, 0.5]
INDEX = 0

arrive = False
angle = False

class MPCController(Controller):

    MOTOR_CONTROL_MODE = simple_motor.MOTOR_CONTROL_HYBRID

    def __init__(self, robot, get_time_since_reset):
        super(MPCController, self).__init__(robot, get_time_since_reset)
        self._constants = robot.GetCtrlConstants()
        self._mpc_controller = self._setup_controller(self._robot)
        self._kinematics = Kinematics(self._robot)
        self._pybullet_client = robot.pybullet_client
        global START_ANGLE
        global INDEX
        global arrive
        global angle
        INDEX = 0
        arrive = False
        angle = False
        

    @property
    def kinematics_model(self):
        return self._kinematics
    

    def _setup_controller(self, robot, desired_speed=(0.0, 0.0), desired_twisting_speed=0.0):
        """ Build the MPC controller. """
        gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
            robot,
            stance_duration=self._constants.STANCE_DURATION_SECONDS,
            duty_factor=self._constants.DUTY_FACTOR,
            initial_leg_phase=self._constants.INIT_PHASE_FULL_CYCLE,
            initial_leg_state=self._constants.INIT_LEG_STATE)
        state_estimator = com_velocity_estimator.COMVelocityEstimator(robot, window_size=20)

        sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
            robot,
            gait_generator,
            state_estimator,
            desired_speed=desired_speed,
            desired_twisting_speed=desired_twisting_speed,
            desired_height=self._constants.MPC_BODY_HEIGHT,
            foot_clearance=0.01)

        st_controller = torque_stance_leg_controller.TorqueStanceLegController(
            robot,
            gait_generator,
            state_estimator,
            desired_speed=desired_speed,
            desired_twisting_speed=desired_twisting_speed,
            desired_body_height=self._constants.MPC_BODY_HEIGHT,
            body_mass=self._constants.MPC_BODY_MASS,
            body_inertia=self._constants.MPC_BODY_INERTIA
        )

        controller = locomotion_controller.LocomotionController(
            robot=robot,
            gait_generator=gait_generator,
            state_estimator=state_estimator,
            swing_leg_controller=sw_controller,
            stance_leg_controller=st_controller,
            clock=self.get_time_since_reset
        )
        return controller

    @staticmethod
    def setup_ui_params(pybullet_client):
        x_id = pybullet_client.addUserDebugParameter("X-coord", -2., 2., 0.)
        y_id = pybullet_client.addUserDebugParameter("Y-coord", -2., 2., 0.)
        speed_id = pybullet_client.addUserDebugParameter(" VX", 0., 1., 0.5)
        x_id2 = pybullet_client.addUserDebugParameter("X-coord", -2., 2., 0.)
        y_id2 = pybullet_client.addUserDebugParameter("Y-coord", -2., 2., 0.)
        speed_id2 = pybullet_client.addUserDebugParameter(" VX", 0., 1., 0.5)
        started = pybullet_client.addUserDebugParameter("Start/Stop", 1,0,1)
        return x_id, y_id, speed_id, x_id2, y_id2, speed_id2, started

    @staticmethod
    def read_ui_params(pybullet_client, ui): #Setting speeds from UI
        x_id, y_id, speed_id, x_id2, y_id2, speed_id2, started = ui
        xcoord = pybullet_client.readUserDebugParameter(x_id)
        ycoord = pybullet_client.readUserDebugParameter(y_id)
        speed = pybullet_client.readUserDebugParameter(speed_id)
        xcoord2 = pybullet_client.readUserDebugParameter(x_id2)
        ycoord2 = pybullet_client.readUserDebugParameter(y_id2)
        speed2 = pybullet_client.readUserDebugParameter(speed_id2)
        start = pybullet_client.readUserDebugParameter(started)
        return xcoord, ycoord, speed, xcoord2, ycoord2, speed2, start
    
    def arrived(self, base_pos0, base_pos1, destination):
        if (abs(base_pos0 - destination[0]) < 0.1) & (abs(base_pos1 - destination[1]) < 0.1):
            return True
        #else:
            #return False
        
    def isAngled(self, base_or0, angle):
        if (abs(base_or0 - angle) < 0.05):
            return True

    def get_angle(self,base_position0, base_position1, destination ):
        desierdAngle = math.atan((destination[1]-base_position1)/(destination[0]-base_position0))
        if ((desierdAngle < 0) & (destination[1]-base_position1 > 0)):
            desierdAngle+=math.pi
        elif (destination[1]-base_position1 < 0) & (destination[0]-base_position0 < 0):
            desierdAngle-=math.pi

        #desierdAngle -= startOrient
        return desierdAngle


    def update_controller_params(self, params):
        if len(params) == 6:
            DESTINATION_VECTOR[0][0], DESTINATION_VECTOR[0][1], SPEED_VECTOR[0], DESTINATION_VECTOR[1][0], DESTINATION_VECTOR[1][1], SPEED_VECTOR[1] = params
            vx = 0.
            wz = 0.
            vy = 0.
        else:
            DESTINATION_VECTOR[0][0], DESTINATION_VECTOR[0][1], SPEED_VECTOR[0], DESTINATION_VECTOR[1][0], DESTINATION_VECTOR[1][1], SPEED_VECTOR[1], start = params
            vx = 0.
            wz = 0.
            vy = 0.
        
        # get pos and rot
        base_position, base_orientation = self._pybullet_client.getBasePositionAndOrientation(self._robot.GetRobotId)
        base_orientation = self._pybullet_client.getEulerFromQuaternion(base_orientation)
        global arrive 
        global angle
        global INDEX
        desierdAngle = 0
        desierdAngle = self.get_angle(base_position[0], base_position[1], DESTINATION_VECTOR[INDEX])
        #fix getting angle depending on current pos so that you can input more points
        if(start % 2 == 0):
            self._pybullet_client.addUserDebugLine([0,0,0], [DESTINATION_VECTOR[0][0],DESTINATION_VECTOR[0][1],0], lineColorRGB=[1, 1, 0], lineWidth=10.0, lifeTime=0)
            self._pybullet_client.addUserDebugLine([DESTINATION_VECTOR[0][0],DESTINATION_VECTOR[0][1],0], [DESTINATION_VECTOR[1][0],DESTINATION_VECTOR[1][1],0], lineColorRGB=[1, 1, 0], lineWidth=10.0, lifeTime=0)
            #set speed if pos is not desierd pos
            if self.arrived(base_position[0], base_position[1], DESTINATION_VECTOR[INDEX]) is not None:
                arrive = self.arrived(base_position[0], base_position[1], DESTINATION_VECTOR[INDEX])
                #print(arrive)
            if(not arrive):
                if self.isAngled(base_orientation[2], desierdAngle) is not None:
                    angle = self.isAngled(base_orientation[2], desierdAngle)
                    
                vx = SPEED_VECTOR[INDEX]
                if(not angle):
                    if((desierdAngle < base_orientation[0]) & (not ((desierdAngle < 0.) & ((2*math.pi + desierdAngle) > base_orientation[0])))):
                        wz = -SPEED_VECTOR[INDEX]*math.pi/(math.sqrt(DESTINATION_VECTOR[0][0]**2 + DESTINATION_VECTOR[0][1]**2))
                        #print("-------")
                            
                    elif((desierdAngle > base_orientation[0]) | ((desierdAngle < 0.) & ((2*math.pi + desierdAngle) > base_orientation[0]))):
                        wz = SPEED_VECTOR[INDEX]*math.pi/(math.sqrt(DESTINATION_VECTOR[0][0]**2 + DESTINATION_VECTOR[0][1]**2))
            elif(arrive):
                INDEX = (INDEX + 1)% len(DESTINATION_VECTOR)
                desierdAngle = self.get_angle(base_position[0], base_position[1], DESTINATION_VECTOR[INDEX])
                print(desierdAngle)
                print(math.atan((DESTINATION_VECTOR[INDEX][1]-base_position[1])/(DESTINATION_VECTOR[INDEX][0]-base_position[0])))
                angle = False
                arrive = False
                print(base_orientation[2])
                print((desierdAngle > base_orientation[0]) | ((desierdAngle < 0.) & ((2*math.pi + desierdAngle) > base_orientation[0])))
        # add robot ctrl offset
        lin_speed = [
            vx + self._constants.VX_OFFSET,
            vy + self._constants.VY_OFFSET,
            0.
        ]

        ang_speed = wz + self._constants.WZ_OFFSET
        # update ctrl params
        self._mpc_controller.swing_leg_controller.desired_speed = lin_speed
        self._mpc_controller.swing_leg_controller.desired_twisting_speed = ang_speed
        self._mpc_controller.stance_leg_controller.desired_speed = lin_speed
        self._mpc_controller.stance_leg_controller.desired_twisting_speed = ang_speed

    def get_action(self):
        # Needed before every call to get_action().
        self._mpc_controller.update()
        hybrid_action = self._mpc_controller.get_action()
        return hybrid_action

    def reset(self):
        global arrive
        global angle
        arrive = False
        angle = False   
        self._mpc_controller.reset()

    @staticmethod
    def get_standing_action():
        return 0., 0.

#
#
# def _run(max_time=MAX_TIME_SECONDS):
#     """Runs the locomotion controller"""
#
#     # while p.isConnected():
#     #  pos,orn = p.getBasePositionAndOrientation(robot_uid)
#     #  print("pos=",pos)
#     #  p.stepSimulation()
#     #  time.sleep(1./240)
#     current_time = robot.GetTimeSinceReset()
#     # logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "mpc.json")
#
#     while current_time < max_time:
#         # pos,orn = p.getBasePositionAndOrientation(robot_uid)
#         # print("pos=",pos, " orn=",orn)
#         p.submitProfileTiming("loop")
#
#         # Updates the controller behavior parameters.
#         lin_speed, ang_speed = _generate_example_linear_angular_speed(current_time)
#         # lin_speed, ang_speed = (0., 0., 0.), 0.
#         robot.UpdateControllerParams(lin_speed, ang_speed)
#
#         robot.Step(hybrid_action)
#
#         if record_video:
#             p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
#
#         # time.sleep(0.003)
#         current_time = robot.GetTimeSinceReset()
#         p.submitProfileTiming()
# p.stopStateLogging(logId)
# while p.isConnected():
#  time.sleep(0.1)
