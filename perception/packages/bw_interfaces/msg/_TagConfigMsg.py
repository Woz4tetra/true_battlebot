# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/TagConfigMsg.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy

class TagConfigMsg(genpy.Message):
  _md5sum: str = "b4dd68b27f1fc126fd8117432c106e7e"
  _type: str = "bw_interfaces/TagConfigMsg"
  _has_header: bool = False  # flag to mark the presence of a Header object
  _full_text: str = """int32 tag_id
float64 tag_size
float64 x  # meters
float64 y  # meters
float64 z  # meters
float64 roll  # degrees
float64 pitch  # degrees
float64 yaw  # degrees
"""
  __slots__: List[str] = ['tag_id','tag_size','x','y','z','roll','pitch','yaw']
  _slot_types: List[str] = ['int32','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, tag_id: int = None,
    tag_size: float = None,
    x: float = None,
    y: float = None,
    z: float = None,
    roll: float = None,
    pitch: float = None,
    yaw: float = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      tag_id,tag_size,x,y,z,roll,pitch,yaw

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(TagConfigMsg, self).__init__(**{'tag_id': tag_id, 'tag_size': tag_size, 'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})
    if self.tag_id is None:
      self.tag_id: int = 0
    else:
      self.tag_id = tag_id
    if self.tag_size is None:
      self.tag_size: float = 0.
    else:
      self.tag_size = tag_size
    if self.x is None:
      self.x: float = 0.
    else:
      self.x = x
    if self.y is None:
      self.y: float = 0.
    else:
      self.y = y
    if self.z is None:
      self.z: float = 0.
    else:
      self.z = z
    if self.roll is None:
      self.roll: float = 0.
    else:
      self.roll = roll
    if self.pitch is None:
      self.pitch: float = 0.
    else:
      self.pitch = pitch
    if self.yaw is None:
      self.yaw: float = 0.
    else:
      self.yaw = yaw

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
      buff.write(_get_struct_i7d().pack(_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'TagConfigMsg':
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
      end += 60
      (_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw,) = _get_struct_i7d().unpack(bytes_[start:end])
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
      buff.write(_get_struct_i7d().pack(_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw))
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
      end += 60
      (_x.tag_id, _x.tag_size, _x.x, _x.y, _x.z, _x.roll, _x.pitch, _x.yaw,) = _get_struct_i7d().unpack(bytes_[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i7d = None
def _get_struct_i7d():
    global _struct_i7d
    if _struct_i7d is None:
        _struct_i7d = struct.Struct("<i7d")
    return _struct_i7d
