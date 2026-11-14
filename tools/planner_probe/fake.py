import rclpy, numpy as np, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
z=np.load('map.npz',allow_pickle=True); m=z['map']; res,ox,oy=[float(v) for v in z['map_info'][:3]]
class F(Node):
    def __init__(s):
        super().__init__('fake')
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        s.mp=s.create_publisher(OccupancyGrid,'/a1_1/map',q); s.cp=s.create_publisher(Clock,'/clock',10); s.tp=s.create_publisher(TFMessage,'/a1_1/tf',10)
        s.t=0.0; s.create_timer(0.05,s.tick); s.sent=False
    def tick(s):
        s.t+=0.05; c=Clock(); c.clock.sec=int(s.t); c.clock.nanosec=int((s.t%1)*1e9); s.cp.publish(c)
        tf=TransformStamped(); tf.header.stamp=c.clock; tf.header.frame_id='map'; tf.child_frame_id='base_footprint'; tf.transform.translation.y=-1.1; tf.transform.rotation.w=1.0
        s.tp.publish(TFMessage(transforms=[tf]))
        if not s.sent and s.t>1:
            g=OccupancyGrid(); g.header.frame_id='map'; g.header.stamp=c.clock; g.info.resolution=res; g.info.width=m.shape[1]; g.info.height=m.shape[0]
            g.info.origin.position.x=ox; g.info.origin.position.y=oy; g.info.origin.orientation.w=1.0; g.data=[int(v) for v in m.astype(np.int8).ravel()]
            s.mp.publish(g); s.sent=True
rclpy.init(); rclpy.spin(F())
