
import numpy as np 
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv2D, Flatten, MaxPooling2D

class R2D2(object):

    
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

        self.model.compile(optimizer='adam', loss='mse')


    

    def getAction(self, st, e=0.0):
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


        
        
    def update(self, batch, gamma=0.99):
        
        st = [timestep[0] for timestep in batch]
        at = [timestep[1] for timestep in batch]
        rt = [timestep[2] for timestep in batch]
        st1 = [timestep[3]for timestep in batch]
        done = [timestep[4] for timestep in batch]

        target = self.model.predict([st])

        for t in range(len(batch)):
            if not done[t]:
                st1_q_value = self.target_model.predict([[st1[t]]])
                target[t, at[t]] = rt[t] + gamma*np.max(st1_q_value)
            else:
                target[t, at[t]] = rt[t]
    
        self.model.fit([st], target, verbose=0)
        
        