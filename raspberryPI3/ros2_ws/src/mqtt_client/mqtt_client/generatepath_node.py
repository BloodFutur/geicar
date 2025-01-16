import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Function to generate path (example)
def generate_path():
    path = Path()
    path.header = Header()
    path.header.stamp = rclpy.clock.Clock().now().to_msg()
    path.header.frame_id = "odom"  # or use the frame_id of your robot's frame

    # Example of path points (x, y, theta)
    points = []

    data = """3.750639988	-2.669934191	-0.2811439543	0.9596656068
17.99785186	-13.11623373	-0.2733425426	0.9619167606
34.66604438	-25.24325761	-0.2519154129	0.9677492572
36.22906321	-24.62106289	0.1915498744	0.981482881
36.27611279	-24.59982545	0.1954977402	0.9807041519
36.48717338	-24.50142664	0.1976271115	0.9802772693
36.48335503	-24.50254397	0.1976087201	0.9802809769
36.58527829	-24.45302428	0.1951213411	0.9807791098
36.58525106	-24.45279262	0.1951696016	0.9807695074
36.65538068	-24.18664975	0.2758829184	0.9611912481
36.65502106	-24.18696322	0.2756984556	0.9612441738
36.91886365	-23.98602569	0.3104214262	0.9505990417
36.92312127	-23.98265078	0.3080542017	0.951368808
36.95680125	-23.9531092	0.3122036881	0.9500151879
36.93830269	-23.65695173	0.4206320683	0.9072313173
37.08168004	-23.45187279	0.445471129	0.8952963047
37.10713512	-23.41271161	0.4547714907	0.8906081581
37.16100096	-23.32711556	0.4665343475	0.8845030823
37.16437613	-23.32208727	0.4638087721	0.8859353379
37.00446013	-22.88155675	0.5957180576	0.8031936229
37.01397338	-22.83527861	0.5998828836	0.8000878239
37.02855803	-22.75715828	0.6200754343	0.7845421951
37.03178071	-22.74624755	0.6157779589	0.7879197328
37.03786018	-22.70239902	0.6210852983	0.7837429759
37.06272176	-22.49936178	0.6348331767	0.7726492334
36.76685239	-22.42740386	0.7270661205	0.6865674449
36.76073157	-22.38447022	0.7345373486	0.67856826
36.7281743	-22.18596387	0.7518418388	0.6593434989
36.70823228	-22.08229612	0.7608851523	0.6488865732
36.29907391	-21.86874037	0.8539130051	0.5204157758
36.29500476	-21.859645	0.8516575161	0.5240987267
36.17679579	-21.65565995	0.8503182924	0.5262687541
36.33643855	-21.939819	0.8536493699	0.5208481095
36.29974159	-21.87732957	0.8617876928	0.5072691323
36.29787051	-21.87293415	0.8605115045	0.5094310068
35.84033808	-21.60863034	0.912931647	0.4081124942
35.74956784	-21.51232809	0.9180229468	0.3965272615
35.71570485	-21.47720692	0.918417692	0.3956121117
35.00926617	-20.93114908	0.9347348498	0.3553459732
34.94555596	-20.87346043	0.9332810373	0.3591469135
34.93807017	-20.86711372	0.9333465798	0.3589765479
34.84455309	-20.78280167	0.9323843671	0.3614683831
34.8463514	-20.78495703	0.9322184272	0.3618961232
34.81203783	-20.75498571	0.932542162	0.3610610974
34.74716504	-20.6986546	0.9340539352	0.357131973
34.7368437	-20.68940678	0.9335781255	0.3583739438
34.70106808	-20.65905346	0.9344170299	0.356180873
34.63458356	-20.6029175	0.9376979515	0.3474515098
34.62310263	-20.59243739	0.9370210706	0.3492728349
7.142827974	0.6034816286	0.9432324571	0.3321333043"""

    for line in data.strip().split('\n'):
        if line and not line.isspace():
            x, y, qz, qw = map(float, line.split())
            # Convert quaternion z,w components to yaw angle
            points.append((x, y, qz, qw))

    # Populate the Path message with PoseStamped
    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.header = path.header  # Same header for all poses
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = 0.0  # Assuming 2D path, z=0

        # Convert theta to quaternion for orientation
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = point[2]
        pose_stamped.pose.orientation.w = point[3]

        path.poses.append(pose_stamped)

    return path


class GeneratePath(Node):
    def __init__(self):
        super().__init__('generate_path_node')
        
        # Publisher for the corrected odom topic
        self.publisher = self.create_publisher(
            Path,
            '/path',
            10)
        self.get_logger().info("Generate Path Node has started.")
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        path = generate_path()
        self.publisher.publish(path)
        
def main(args=None):
    rclpy.init(args=args)
    node = GeneratePath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



