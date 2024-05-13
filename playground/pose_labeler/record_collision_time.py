from dataclasses import dataclass

import rospy
from bw_interfaces.msg import CollisionInfo
from std_msgs.msg import Float64


@dataclass
class CollisionData:
    current_time: float = 0.0
    collision_time: float = 0.0
    collision_info: CollisionInfo = CollisionInfo()


def sequence_progress_callback(msg: Float64, collision_data: CollisionData) -> None:
    if msg.data < collision_data.current_time:
        print("-----")
    collision_data.current_time = msg.data


def collision_info_callback(msg: CollisionInfo, collision_data: CollisionData) -> None:
    if not (msg.source_object == "mini_bot" and msg.collision_with == "opponent_1"):
        return
    collision_data.collision_info = msg
    collision_data.collision_time = collision_data.current_time
    print(f"Collision time: {collision_data.collision_time}")


def main() -> None:
    collision_data = CollisionData()
    rospy.init_node("record_collision_time")
    rospy.Subscriber(
        "/mini_bot/sequence_progress",
        Float64,
        lambda m: sequence_progress_callback(m, collision_data),
        queue_size=1,
    )
    rospy.Subscriber(
        "/simulation/collision_info",
        CollisionInfo,
        lambda m: collision_info_callback(m, collision_data),
        queue_size=1,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
