import numpy as np
import matplotlib.pyplot as plt

def LIP(init_pos, goal_pos, init_time, goal_time, cur_time):
    _slope = (goal_pos - init_pos) / (goal_time - init_time)
    _desired = _slope*(cur_time - init_time) + init_pos
    
    return _desired

lparray = []
init_pos = 0
goal_pos = 6.5
init_time = 0.0
goal_time = 10
cur_time = 0
for i in range(11):
    lp = LIP(init_pos, goal_pos, init_time, goal_time, cur_time)
    cur_time += 1
    print(lp)
    lparray.append(lp)
    
# print(lp)
print(lparray)

plt.figure(figsize=(20,10))
plt.plot(lparray[:], label = "ref")
plt.show()