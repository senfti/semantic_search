import numpy as np
import matplotlib.pyplot as plt

def u(p,vh,vm):
    return p*vh+(1-p)*vm

def p_next(p,p_old,p0,vh,vm):
    return (u(p,vh,vm)*p_old/p0) / ((u(p,vh,vm)*p_old/p0)+((1-u(p,vh,vm))*(1-p_old)/(1-p0)))

vh = 5
vm = 0.49999999
p0=0.5
p = [0.004]
for i in range(50):
    p.append(p_next(0.1,p[-1],p0,vh,vm))

plt.plot(p)
plt.grid(True)
plt.show()