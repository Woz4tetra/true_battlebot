# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/KeypointInstanceArray.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy
from bw_interfaces.msg._KeypointInstance import KeypointInstance as bw_interfaces_msg_KeypointInstance
from bw_interfaces.msg._UVKeypoint import UVKeypoint as bw_interfaces_msg_UVKeypoint
from std_msgs.msg._Header import Header as std_msgs_msg_Header

class KeypointInstanceArray(genpy.Message):
  _md5sum: str = "2c627f8b67893f3343b7c0a44a443686"
  _type: str = "bw_interfaces/KeypointInstanceArray"
  _has_header: bool = True  # flag to mark the presence of a Header object
  _full_text: str = """std_msgs/Header header
uint32 height
uint32 width
bw_interfaces/KeypointInstance[] instances

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: bw_interfaces/KeypointInstance
bw_interfaces/UVKeypoint[] keypoints
string[] names
float32 score
string label
uint32 class_index
uint32 object_index

================================================================================
MSG: bw_interfaces/UVKeypoint
float32 x
float32 y
"""
  __slots__: List[str] = ['header','height','width','instances']
  _slot_types: List[str] = ['std_msgs/Header','uint32','uint32','bw_interfaces/KeypointInstance[]']

  def __init__(self, header: std_msgs_msg_Header = None,
    height: int = None,
    width: int = None,
    instances: List[bw_interfaces_msg_KeypointInstance] = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      header,height,width,instances

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(KeypointInstanceArray, self).__init__(**{'header': header, 'height': height, 'width': width, 'instances': instances})
    if self.header is None:
      self.header: std_msgs_msg_Header = std_msgs_msg_Header()
    else:
      self.header = header
    if self.height is None:
      self.height: int = 0
    else:
      self.height = height
    if self.width is None:
      self.width: int = 0
    else:
      self.width = width
    if self.instances is None:
      self.instances: List[bw_interfaces_msg_KeypointInstance] = []
    else:
      self.instances = instances

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.height, _x.width))
      length = len(self.instances)
      buff.write(_struct_I.pack(length))
      for val1 in self.instances:
        length = len(val1.keypoints)
        buff.write(_struct_I.pack(length))
        for val2 in val1.keypoints:
          _x = val2
          buff.write(_get_struct_2f().pack(_x.x, _x.y))
        length = len(val1.names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.Struct('<I%ss'%length).pack(length, val2))
        _x = val1.score
        buff.write(_get_struct_f().pack(_x))
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.class_index, _x.object_index))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'KeypointInstanceArray':
    """
    unpack serialized message in str into this message instance
    :param bytes_: byte array of serialized message, ``bytes``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs_msg_Header()
      if self.instances is None:
        self.instances = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(bytes_[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = bytes_[start:end]
      _x = self
      start = end
      end += 8
      (_x.height, _x.width,) = _get_struct_2I().unpack(bytes_[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.instances = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_KeypointInstance()
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        val1.keypoints = []
        for i in range(0, length):
          val2 = bw_interfaces_msg_UVKeypoint()
          _x = val2
          start = end
          end += 8
          (_x.x, _x.y,) = _get_struct_2f().unpack(bytes_[start:end])
          val1.keypoints.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        val1.names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(bytes_[start:end])
          start = end
          end += length
          if python3:
            val2 = bytes_[start:end].decode('utf-8', 'rosmsg')
          else:
            val2 = bytes_[start:end]
          val1.names.append(val2)
        start = end
        end += 4
        (val1.score,) = _get_struct_f().unpack(bytes_[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.label = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = bytes_[start:end]
        _x = val1
        start = end
        end += 8
        (_x.class_index, _x.object_index,) = _get_struct_2I().unpack(bytes_[start:end])
        self.instances.append(val1)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.height, _x.width))
      length = len(self.instances)
      buff.write(_struct_I.pack(length))
      for val1 in self.instances:
        length = len(val1.keypoints)
        buff.write(_struct_I.pack(length))
        for val2 in val1.keypoints:
          _x = val2
          buff.write(_get_struct_2f().pack(_x.x, _x.y))
        length = len(val1.names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.Struct('<I%ss'%length).pack(length, val2))
        _x = val1.score
        buff.write(_get_struct_f().pack(_x))
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2I().pack(_x.class_index, _x.object_index))
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
      if self.header is None:
        self.header = std_msgs_msg_Header()
      if self.instances is None:
        self.instances = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(bytes_[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = bytes_[start:end]
      _x = self
      start = end
      end += 8
      (_x.height, _x.width,) = _get_struct_2I().unpack(bytes_[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.instances = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_KeypointInstance()
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        val1.keypoints = []
        for i in range(0, length):
          val2 = bw_interfaces_msg_UVKeypoint()
          _x = val2
          start = end
          end += 8
          (_x.x, _x.y,) = _get_struct_2f().unpack(bytes_[start:end])
          val1.keypoints.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        val1.names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(bytes_[start:end])
          start = end
          end += length
          if python3:
            val2 = bytes_[start:end].decode('utf-8', 'rosmsg')
          else:
            val2 = bytes_[start:end]
          val1.names.append(val2)
        start = end
        end += 4
        (val1.score,) = _get_struct_f().unpack(bytes_[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1.label = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = bytes_[start:end]
        _x = val1
        start = end
        end += 8
        (_x.class_index, _x.object_index,) = _get_struct_2I().unpack(bytes_[start:end])
        self.instances.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f