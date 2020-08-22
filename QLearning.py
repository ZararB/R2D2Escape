

import Environment
import time
#import R2D2 



num_episodes = 10

env = Environment.Environment()
#agent = R2D2.R2D2()


for e in range(num_episodes):

    st = env.reset()
    episode_reward = 0

    while True:
    
        at = 3
        st1, rt, done, debug = env.step(at)
        episode_reward += rt

        
        if done:
            print('Collision Detected!!!')
            print('Episode {} Ended with Reward {}'.format(e, episode_reward))
            break
