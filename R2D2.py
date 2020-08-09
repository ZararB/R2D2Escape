
import numpy as np 
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv2D, Flatten, MaxPooling2D

class R2D2:

    #TODO
    def __init__(self):


        self.model = Sequential([
        Conv2D(32, 8, 4, input_shape=(self.model_input_x, self.model_input_y, 4,),
            activation='relu'),
        Conv2D(64, 4, 2, activation='relu'),
        Conv2D(64, 3, 1, activation='relu'),
        Flatten(),
        Dense(512, activation='relu'),
        Dense(self.num_actions, activation='linear')
        ]) 

        pass

    

    def get_action(self, st, e=0.0):
        '''
        Return the action to take in state st using 
        an e-greedy policy
        '''
        if np.random.rand() < e:
            action = self.env.action_space.sample()
            
        else:
            model_input  = self.preprocess_state(st)
            q_values = self.model.predict(model_input)
            action = np.argmax(q_values)
            
        return action

    def preprocess_state(self, state):
        return model_input
        
    def update(self, batch):
        pass