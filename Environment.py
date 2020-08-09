import pybullet as p 
import pybullet_data 
import time 
import numpy as np 



class Environment(object):

    '''
    State Space:

    Action Space:

    0 - w - move forward
    1 - s - move backwards
    2 - a - rotate counterclockwise
    3 - d - rotate clockwise
    
    potential actions

    wa - 
    wd - 
    sa -
    sd -  
    

    '''

    '''
    Problem: Agent is not moving continously when taking actions 
    
    Solution0: Destroy agent and spawn again in new position 
    Solution1: Apply force to multiple links to prevent it from tipping over
    '''

    def __init__(self):

        client = p.connect(p.GUI)
        p.setTimeOut(2)
        p.setGravity(0,0,-9.8)

        self.max_timesteps = 10000
        self.prevAction = -1 

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeID = p.loadURDF("plane.urdf")

    def reset(self, agent='r2d2.urdf', startPos=[0,0,1], startOrn=[0,0,0]):
        '''
        Reset world state to given initial state
        '''
        self.timestep = 0
        startOrn = p.getQuaternionFromEuler(startOrn)
        self.r2d2Id = p.loadURDF(agent, startPos, startOrn)

        # Create obstacle course

        #TODO Spawn planes that enclose a rectangular area of dimensions 10x100 (widthxheight) Zarar 

        initial_obs = self.get_observation()

        return initial_obs

    def step(self, action):

        pos_t, orn_t = p.getBasePositionAndOrientation(self.r2d2Id)

        if action != self.prevAction:
            prevAction = action
            self.setAction(action)

        p.stepSimulation()
        time.sleep(1./240)

        next_obs = self.get_observation()
        reward = self.get_reward()
        done = self.is_done()
        debug = []

        self.timestep += 1

        return next_obs, reward, done, debug

    def setAction(self,action):
        '''
        Sets the action the r2d2 should be taking (move forward/backward or rotate CW, CCW).
        There are 15 joints in this robot.

        Joint at index 2 is "right_front_wheel_joint"
        Joint at index 3 is "right_back_wheel_joint"
        Joint at index 7 is "left_front_wheel_joint"
        Joint at index 8 is "left_back_wheel_joint"

        These are the joints that matter for movement. Following code can give information of all joints:

        for i in range(0,15):
            print(p.getJointInfo(self.r2d2Id,i))
        '''

        if action == 0:
                p.setJointMotorControlArray(bodyUniqueId = self.r2d2Id,
                                            jointIndices = [2,3,6,7], 
                                            controlMode = p.VELOCITY_CONTROL,
                                            targetVelocities = [-20,-20,-20,-20],
                                            forces = [100,100,100,100])
        elif action == 1:
                p.setJointMotorControlArray(bodyUniqueId = self.r2d2Id,
                                            jointIndices = [2,3,6,7], 
                                            controlMode = p.VELOCITY_CONTROL,
                                            targetVelocities = [20,20,20,20],
                                            forces = [100,100,100,100])
        elif action == 2:
                p.setJointMotorControlArray(bodyUniqueId = self.r2d2Id,
                                            jointIndices = [3,6], 
                                            controlMode = p.VELOCITY_CONTROL,
                                            targetVelocities = [80,-80],
                                            forces = [100,100])
        elif action == 3:
            p.setJointMotorControlArray(bodyUniqueId = self.r2d2Id,
                                            jointIndices = [2,7], 
                                            controlMode = p.VELOCITY_CONTROL,
                                            targetVelocities = [-80,80],
                                            forces = [100,100])

    def get_observation(self):
        '''
        Returns the observation 
        '''
        #TODO Attach camera to agent
        img_width = 1280 
        img_height = 720

        # state == np array shape (width, height, 3)
        return state 


    def get_reward(self, weight=1):
        '''
        Calculates and returns the reward that the agent maximizes
        '''
        pos, orn = p.getBasePositionAndOrientation(self.r2d2Id)
        reward = pos[0] - self.timestep*(weight)
        return reward 


    def is_done(self):
        '''
        Returns True if agent completes escape (x >= 100) or if episode duration > max_episode_length 
        '''
        #TODO Write is_done function Niranjan
        # pos, orn = p.getBasePositionAndOrientation(self.r2d2)

        return done     