from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.motor_description import MotorCommand, MotorDescription
from bw_tools.structs.teleop_bridge.ping_info import PingInfo

header = Header(9, 2, 1)
print(header.to_bytes())

ping = PingInfo(header, 123)
print(ping.to_bytes())

desc = MotorDescription(0, [MotorCommand(1, 2), MotorCommand(3, 4)])
print(desc.to_bytes())
