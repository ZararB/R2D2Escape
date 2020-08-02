

import Environment
import R2D2 

env = Environment.Environment()
agent = R2D2.R2D2()

s0 = env.reset()

for i in range(1000000):
    at = 3
    st1, rt, done, debug = env.step(at)

    if done:
        break
