# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/GoalEngineConfig.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy

class GoalEngineConfig(genpy.Message):
  _md5sum: str = "b00b07f915f31ffed0be8d1ff3e56daa"
  _type: str = "bw_interfaces/GoalEngineConfig"
  _has_header: bool = False  # flag to mark the presence of a Header object
  _full_text: str = """float64 max_velocity
float64 max_angular_velocity
float64 max_acceleration
float64 max_centripetal_acceleration
bool is_max_centripetal_acceleration
bool rotate_at_end
float64 start_velocity
bool is_start_velocity
float64 end_velocity
bool is_end_velocity
"""
  __slots__: List[str] = ['max_velocity','max_angular_velocity','max_acceleration','max_centripetal_acceleration','is_max_centripetal_acceleration','rotate_at_end','start_velocity','is_start_velocity','end_velocity','is_end_velocity']
  _slot_types: List[str] = ['float64','float64','float64','float64','bool','bool','float64','bool','float64','bool']

  def __init__(self, max_velocity: float = None,
    max_angular_velocity: float = None,
    max_acceleration: float = None,
    max_centripetal_acceleration: Optional[float] = None,
    is_max_centripetal_acceleration: bool = None,
    rotate_at_end: bool = None,
    start_velocity: Optional[float] = None,
    is_start_velocity: bool = None,
    end_velocity: Optional[float] = None,
    is_end_velocity: bool = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      max_velocity,max_angular_velocity,max_acceleration,max_centripetal_acceleration,is_max_centripetal_acceleration,rotate_at_end,start_velocity,is_start_velocity,end_velocity,is_end_velocity

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(GoalEngineConfig, self).__init__(**{'max_velocity': max_velocity, 'max_angular_velocity': max_angular_velocity, 'max_acceleration': max_acceleration, 'max_centripetal_acceleration': max_centripetal_acceleration, 'is_max_centripetal_acceleration': is_max_centripetal_acceleration, 'rotate_at_end': rotate_at_end, 'start_velocity': start_velocity, 'is_start_velocity': is_start_velocity, 'end_velocity': end_velocity, 'is_end_velocity': is_end_velocity})
    if self.max_velocity is None:
      self.max_velocity: float = 0.
    else:
      self.max_velocity = max_velocity
    if self.max_angular_velocity is None:
      self.max_angular_velocity: float = 0.
    else:
      self.max_angular_velocity = max_angular_velocity
    if self.max_acceleration is None:
      self.max_acceleration: float = 0.
    else:
      self.max_acceleration = max_acceleration
    if self.max_centripetal_acceleration is None:
      self.max_centripetal_acceleration: Optional[float] = 0.
    else:
      self.max_centripetal_acceleration = max_centripetal_acceleration
    if self.is_max_centripetal_acceleration is None:
      self.is_max_centripetal_acceleration: bool = False
    else:
      self.is_max_centripetal_acceleration = is_max_centripetal_acceleration
    if self.rotate_at_end is None:
      self.rotate_at_end: bool = False
    else:
      self.rotate_at_end = rotate_at_end
    if self.start_velocity is None:
      self.start_velocity: Optional[float] = 0.
    else:
      self.start_velocity = start_velocity
    if self.is_start_velocity is None:
      self.is_start_velocity: bool = False
    else:
      self.is_start_velocity = is_start_velocity
    if self.end_velocity is None:
      self.end_velocity: Optional[float] = 0.
    else:
      self.end_velocity = end_velocity
    if self.is_end_velocity is None:
      self.is_end_velocity: bool = False
    else:
      self.is_end_velocity = is_end_velocity

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff: BytesIO) -> None:
    """
    serialize message into buffer
    :param buff: buffer, ``BytesIO``
    """
    try:
      _x = self
      buff.write(_get_struct_4d2BdBdB().pack(_x.max_velocity, _x.max_angular_velocity, _x.max_acceleration, _x.max_centripetal_acceleration, _x.is_max_centripetal_acceleration, _x.rotate_at_end, _x.start_velocity, _x.is_start_velocity, _x.end_velocity, _x.is_end_velocity))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'GoalEngineConfig':
    """
    unpack serialized message in str into this message instance
    :param bytes_: byte array of serialized message, ``bytes``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 52
      (_x.max_velocity, _x.max_angular_velocity, _x.max_acceleration, _x.max_centripetal_acceleration, _x.is_max_centripetal_acceleration, _x.rotate_at_end, _x.start_velocity, _x.is_start_velocity, _x.end_velocity, _x.is_end_velocity,) = _get_struct_4d2BdBdB().unpack(bytes_[start:end])
      self.is_max_centripetal_acceleration = bool(self.is_max_centripetal_acceleration)
      self.rotate_at_end = bool(self.rotate_at_end)
      self.is_start_velocity = bool(self.is_start_velocity)
      self.is_end_velocity = bool(self.is_end_velocity)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``BytesIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_4d2BdBdB().pack(_x.max_velocity, _x.max_angular_velocity, _x.max_acceleration, _x.max_centripetal_acceleration, _x.is_max_centripetal_acceleration, _x.rotate_at_end, _x.start_velocity, _x.is_start_velocity, _x.end_velocity, _x.is_end_velocity))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 52
      (_x.max_velocity, _x.max_angular_velocity, _x.max_acceleration, _x.max_centripetal_acceleration, _x.is_max_centripetal_acceleration, _x.rotate_at_end, _x.start_velocity, _x.is_start_velocity, _x.end_velocity, _x.is_end_velocity,) = _get_struct_4d2BdBdB().unpack(bytes_[start:end])
      self.is_max_centripetal_acceleration = bool(self.is_max_centripetal_acceleration)
      self.rotate_at_end = bool(self.rotate_at_end)
      self.is_start_velocity = bool(self.is_start_velocity)
      self.is_end_velocity = bool(self.is_end_velocity)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4d2BdBdB = None
def _get_struct_4d2BdBdB():
    global _struct_4d2BdBdB
    if _struct_4d2BdBdB is None:
        _struct_4d2BdBdB = struct.Struct("<4d2BdBdB")
    return _struct_4d2BdBdB
