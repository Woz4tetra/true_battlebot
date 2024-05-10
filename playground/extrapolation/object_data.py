from dataclasses import dataclass

from bw_tools.structs.transform3d import Transform3D
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header


@dataclass
class ObjectData:
    ground_truth_data: dict[str, list[PoseStamped]]
    measurements: dict[str, list[PoseStamped]]
    sensor_transforms: dict[str, TransformStamped]

    @property
    def measurements_in_map(self) -> dict[str, list[PoseStamped]]:
        measurements_in_map: dict[str, list[PoseStamped]] = {}
        for label, poses in self.measurements.items():
            for pose in poses:
                sensor_transform = self.sensor_transforms[pose.header.frame_id]
                map_to_sensor = Transform3D.from_msg(sensor_transform.transform)
                pose_in_sensor = Transform3D.from_pose_msg(pose.pose)
                pose_in_map = map_to_sensor.transform_by(pose_in_sensor)
                if label not in measurements_in_map:
                    measurements_in_map[label] = []
                map_header = Header(pose.header.seq, pose.header.stamp, sensor_transform.header.frame_id)
                measurements_in_map[label].append(PoseStamped(map_header, pose_in_map.to_pose_msg()))
        return measurements_in_map
