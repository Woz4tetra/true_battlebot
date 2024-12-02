from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.messages.header import Header


def transform_estimated_object(
    estimated_object: EstimatedObject, transform: Transform3D, header: Header
) -> EstimatedObject:
    transformed_object = Transform3D.from_pose_msg(estimated_object.pose.pose).forward_by(transform)
    new_object = EstimatedObject()
    new_object.header = header.to_msg()
    new_object.child_frame_id = estimated_object.child_frame_id
    new_object.pose.pose = transformed_object.to_pose_msg()
    new_object.size = estimated_object.size
    new_object.label = estimated_object.label
    return new_object
