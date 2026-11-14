import rclpy, numpy as np, math, sys, time
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from scipy import ndimage
z=np.load('map.npz',allow_pickle=True); m=z['map']; res,ox,oy=[float(v) for v in z['map_info'][:3]]
dist=ndimage.distance_transform_edt(m!=100)*res
rclpy.init(); n=rclpy.create_node('cl'); ac=ActionClient(n,ComputePathToPose,'/a1_1/compute_path_to_pose')
ac.wait_for_server(timeout_sec=30)
def ps(x,y):
    p=PoseStamped(); p.header.frame_id='map'; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.0; return p
time.sleep(3)
for name,(gx,gy) in [('south_beyond_ramp',(0.0,-4.6)),('west_hall',(-3.0,-0.4)),('se_room',(2.8,-4.3)),('sw',(-3.5,-4.5)),('east_hall',(3.0,-0.4))]:
    g=ComputePathToPose.Goal(); g.goal=ps(gx,gy); g.start=ps(0.0,-1.1); g.use_start=True; g.planner_id='GridBased'
    f=ac.send_goal_async(g); rclpy.spin_until_future_complete(n,f,timeout_sec=20); h=f.result()
    r=h.get_result_async(); rclpy.spin_until_future_complete(n,r,timeout_sec=20)
    path=r.result().result.path.poses
    if not path: print(name,'NO PATH'); continue
    xy=np.array([[p.pose.position.x,p.pose.position.y] for p in path])
    L=np.sum(np.hypot(*np.diff(xy,axis=0).T))
    c=[dist[int((y-oy)/res),int((x-ox)/res)] for x,y in xy]
    print(f'{name:18s} len {L:5.2f} m  clearance min {min(c):.2f} median {np.median(c):.2f} m')
