from interfaces.msg import Ultrasonic

# We check the two sensors each because they don't look at the same place
# Ex : if obstacle only at the left side, the right sensor won't see it


def detection(msg_us: Ultrasonic, detection_distance: int):
    """
    Function that detects if the obstacle is closer than the detection distance
    """
    if msg_us.front_left <= detection_distance or msg_us.front_right <= detection_distance:
        return True
    else:
        return False
