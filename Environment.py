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
        p.setRealTimeSimulation(0)
        self.speed = 1000
        self.rotation_speed = 1
        self.max_timesteps = 10000

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


        for _ in range(500):
            p.stepSimulation()
            time.sleep(1./240)


        initial_obs = self.get_observation()

        return initial_obs




    def step(self, action):

        pos_t, orn_t = p.getBasePositionAndOrientation(self.r2d2Id)

        #TODO Apply solution0 Zarar

        #TODO Try to solve using solution1 Rajat 

        if action == 0 :
            #TODO Implement without using p.LINK_FRAME
            #TODO Excerise: Determine the force that will move the agent relative to its local axis 
            force = np.array([1, 0, 0]) * self.speed
            p.applyExternalForce(self.r2d2Id, -1, [0, 1000, 0], pos_t, p.LINK_FRAME)
        
        elif action == 1:
            force = np.array([-1, 0, 0]) * self.speed
            p.applyExternalForce(self.r2d2Id, -1, force, pos_t, p.LINK_FRAME)
            
        elif action == 2:
            #TODO Determine torque to be applied 
            torque = np.array([0, 0, 1]) * self.rotation_speed
            p.applyExternalTorque(self.r2d2Id, -1, torque, pos_t, p.LINK_FRAME)

        elif action == 3:
            torque =np.array([0, 0, -1]) * self.rotation_speed
            p.applyExternalTorque(self.r2d2Id, -1, torque, pos_t, p.LINK_FRAME)
            
        p.stepSimulation()
        time.sleep(1./240)

        next_obs = self.get_observation()
        reward = self.get_reward()
        done = self.is_done()
        debug = []

        self.timestep += 1

        return next_obs, reward, done, debug


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
        pos, orn = p.getBasePositionAndOrientation(self.r2d2)

        if (self.get_reward >= 100) | (self.timestep>=self.max_timesteps):
            done=True
        else:
            done=False
        
        return done     
