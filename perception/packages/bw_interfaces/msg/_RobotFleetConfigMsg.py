# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/RobotFleetConfigMsg.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy
from bw_interfaces.msg._BundleConfigMsg import BundleConfigMsg as bw_interfaces_msg_BundleConfigMsg
from bw_interfaces.msg._RobotConfigMsg import RobotConfigMsg as bw_interfaces_msg_RobotConfigMsg
from bw_interfaces.msg._TagConfigMsg import TagConfigMsg as bw_interfaces_msg_TagConfigMsg

class RobotFleetConfigMsg(genpy.Message):
  _md5sum: str = "445b76b32261c06fe3411be0c3ef5d28"
  _type: str = "bw_interfaces/RobotFleetConfigMsg"
  _has_header: bool = False  # flag to mark the presence of a Header object
  _full_text: str = """bw_interfaces/RobotConfigMsg[] robots

================================================================================
MSG: bw_interfaces/RobotConfigMsg
string name
string team
bw_interfaces/BundleConfigMsg tags
float64 radius

================================================================================
MSG: bw_interfaces/BundleConfigMsg
string name
bw_interfaces/TagConfigMsg[] tags

================================================================================
MSG: bw_interfaces/TagConfigMsg
int32 tag_id
float64 tag_size
float64 x  # meters
float64 y  # meters
float64 z  # meters
float64 roll  # degrees
float64 pitch  # degrees
float64 yaw  # degrees
"""
  __slots__: List[str] = ['robots']
  _slot_types: List[str] = ['bw_interfaces/RobotConfigMsg[]']

  def __init__(self, robots: List[bw_interfaces_msg_RobotConfigMsg] = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      robots

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(RobotFleetConfigMsg, self).__init__(**{'robots': robots})
    if self.robots is None:
      self.robots: List[bw_interfaces_msg_RobotConfigMsg] = []
    else:
      self.robots = robots

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
      length = len(self.robots)
      buff.write(_struct_I.pack(length))
      for val1 in self.robots:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.team
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v45 = val1.tags
        _x = _v45.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(_v45.tags)
        buff.write(_struct_I.pack(length))
        for val3 in _v45.tags:
          _x = val3
          buff.write(_get_struct_i7d().pack(_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw))
        _x = val1.radius
        buff.write(_get_struct_d().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'RobotFleetConfigMsg':
    """
    unpack serialized message in str into this message instance
    :param bytes_: byte array of serialized message, ``bytes``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.robots is None:
        self.robots = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.robots = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_RobotConfigMsg()
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.name = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = bytes_[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.team = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.team = bytes_[start:end]
        _v46 = val1.tags
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          _v46.name = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          _v46.name = bytes_[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        _v46.tags = []
        for i in range(0, length):
          val3 = bw_interfaces_msg_TagConfigMsg()
          _x = val3
          start = end
          end += 60
          (_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw,) = _get_struct_i7d().unpack(bytes_[start:end])
          _v46.tags.append(val3)
        start = end
        end += 8
        (val1.radius,) = _get_struct_d().unpack(bytes_[start:end])
        self.robots.append(val1)
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
      length = len(self.robots)
      buff.write(_struct_I.pack(length))
      for val1 in self.robots:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.team
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v47 = val1.tags
        _x = _v47.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(_v47.tags)
        buff.write(_struct_I.pack(length))
        for val3 in _v47.tags:
          _x = val3
          buff.write(_get_struct_i7d().pack(_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw))
        _x = val1.radius
        buff.write(_get_struct_d().pack(_x))
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
      if self.robots is None:
        self.robots = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.robots = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_RobotConfigMsg()
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.name = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = bytes_[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.team = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.team = bytes_[start:end]
        _v48 = val1.tags
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          _v48.name = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          _v48.name = bytes_[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        _v48.tags = []
        for i in range(0, length):
          val3 = bw_interfaces_msg_TagConfigMsg()
          _x = val3
          start = end
          end += 60
          (_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw,) = _get_struct_i7d().unpack(bytes_[start:end])
          _v48.tags.append(val3)
        start = end
        end += 8
        (val1.radius,) = _get_struct_d().unpack(bytes_[start:end])
        self.robots.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
_struct_i7d = None
def _get_struct_i7d():
    global _struct_i7d
    if _struct_i7d is None:
        _struct_i7d = struct.Struct("<i7d")
    return _struct_i7d