from typing import Union

from bw_shared.radio.crsf.crsf_attitude import CrsfAttitude
from bw_shared.radio.crsf.crsf_battery import CrsfBattery
from bw_shared.radio.crsf.crsf_flight_mode import CrsfFlightMode
from bw_shared.radio.crsf.crsf_link_statistics import CrsfLinkStatistics

CrsfPacket = Union[CrsfBattery, CrsfLinkStatistics, CrsfAttitude, CrsfFlightMode]
