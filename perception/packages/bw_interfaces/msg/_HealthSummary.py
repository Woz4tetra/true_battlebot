# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/HealthSummary.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy
from std_msgs.msg._Header import Header as std_msgs_msg_Header

class HealthSummary(genpy.Message):
  _md5sum: str = "ce57e7b2bf73c946886451233f8977ba"
  _type: str = "bw_interfaces/HealthSummary"
  _has_header: bool = True  # flag to mark the presence of a Header object
  _full_text: str = """std_msgs/Header header
string[] active_nodes
string[] dead_nodes

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
"""
  __slots__: List[str] = ['header','active_nodes','dead_nodes']
  _slot_types: List[str] = ['std_msgs/Header','string[]','string[]']

  def __init__(self, header: std_msgs_msg_Header = None,
    active_nodes: List[str] = None,
    dead_nodes: List[str] = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      header,active_nodes,dead_nodes

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(HealthSummary, self).__init__(**{'header': header, 'active_nodes': active_nodes, 'dead_nodes': dead_nodes})
    if self.header is None:
      self.header: std_msgs_msg_Header = std_msgs_msg_Header()
    else:
      self.header = header
    if self.active_nodes is None:
      self.active_nodes: List[str] = []
    else:
      self.active_nodes = active_nodes
    if self.dead_nodes is None:
      self.dead_nodes: List[str] = []
    else:
      self.dead_nodes = dead_nodes

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
      length = len(self.active_nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.active_nodes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.dead_nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.dead_nodes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'HealthSummary':
    """
    unpack serialized message in str into this message instance
    :param bytes_: byte array of serialized message, ``bytes``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs_msg_Header()
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.active_nodes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1 = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = bytes_[start:end]
        self.active_nodes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.dead_nodes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1 = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = bytes_[start:end]
        self.dead_nodes.append(val1)
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
      length = len(self.active_nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.active_nodes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.dead_nodes)
      buff.write(_struct_I.pack(length))
      for val1 in self.dead_nodes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.active_nodes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1 = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = bytes_[start:end]
        self.active_nodes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.dead_nodes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(bytes_[start:end])
        start = end
        end += length
        if python3:
          val1 = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = bytes_[start:end]
        self.dead_nodes.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I