import rclpy, sys, numpy as np
from sensor_msgs.msg import PointCloud2
rclpy.init(); n=rclpy.create_node('hzc'); got=[]
def cb(m):
    a=np.frombuffer(bytes(m.data),dtype=np.float32).reshape(-1,3); got.append(a)
n.create_subscription(PointCloud2,'/a1_1/terrain_hazards',cb,10)
import time; t=time.time()
while time.time()-t<12: rclpy.spin_once(n,timeout_sec=0.2)
a=got[-1] if got else np.zeros((0,3))
print('msgs',len(got),'hazard pts',len(a), 'min y',round(float(a[:,1].min()),3) if len(a) else None)
