import tinyik
import numpy as np

arm = tinyik.Actuator(['y',[0,0,0.01], 'z', [1., 0., 0.], 'z', [1., 0., 0.]])
#arm.angles = np.deg2rad([30, 60])
#tinyik.visualize(arm)
arm.ee = [0,0,2]
print(arm.ee)
tinyik.visualize(arm)
arm.ee = [0,2,0]
print(arm.ee)
tinyik.visualize(arm)
arm.ee = [2,0,0]
print(arm.ee)
tinyik.visualize(arm)
