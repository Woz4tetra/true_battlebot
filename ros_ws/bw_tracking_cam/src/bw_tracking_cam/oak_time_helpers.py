import rospy


def ros_time_from_nsec(nsec: int) -> rospy.Time:
    sec = int(nsec // 1e9)
    nsec = int(nsec % 1e9)
    return rospy.Time(sec, nsec)


def get_frame_time(ros_base_time: rospy.Time, steady_base_time: int, curr_time_point: int):
    # Calculate elapsed time in nanoseconds
    elapsed_time_nsec = curr_time_point - steady_base_time

    # Convert ros_base_time to seconds
    ros_base_time_nsec = ros_base_time.to_nsec()

    # Calculate new time in seconds
    new_time_nsec = ros_base_time_nsec + elapsed_time_nsec

    # Convert back to ROS Time
    return ros_time_from_nsec(new_time_nsec)
