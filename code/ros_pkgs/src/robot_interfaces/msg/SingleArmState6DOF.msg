std_msgs/Header header
float64[6] joint_position    # joint angles in rad [j1..j6]
float64[3] eef_position      # end-effector xyz
float64[4] eef_orientation   # end-effector quaternion [x, y, z, w]
bool gripper_open             # True=open, False=closed
