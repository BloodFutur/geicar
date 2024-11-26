from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool 

# We check the two sensors each because they don't look at the same place
# Ex : if obstacle only at the left side, the right sensor won't see it
def detection(msg_us : Ultrasonic, detection_distance : int ) :
    if msg_us.front_left <= detection_distance or msg_us.front_right <= detection_distance:
        return True
    else :
        return False