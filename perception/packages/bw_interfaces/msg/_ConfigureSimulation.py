# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bw_interfaces/ConfigureSimulation.msg. Do not edit."""
import codecs
import sys
from io import BytesIO
from typing import List, Tuple, Optional
python3 = True if sys.hexversion > 0x03000000 else False
import struct
import genpy
from bw_interfaces.msg._SimulationConfig import SimulationConfig as bw_interfaces_msg_SimulationConfig

class ConfigureSimulation(genpy.Message):
  _md5sum: str = "01da5b2d9068c99fdcb16dab6ba1dd31"
  _type: str = "bw_interfaces/ConfigureSimulation"
  _has_header: bool = False  # flag to mark the presence of a Header object
  _full_text: str = """bw_interfaces/SimulationConfig scenario
bw_interfaces/SimulationConfig[] objectives

================================================================================
MSG: bw_interfaces/SimulationConfig
string name
string json_data
"""
  __slots__: List[str] = ['scenario','objectives']
  _slot_types: List[str] = ['bw_interfaces/SimulationConfig','bw_interfaces/SimulationConfig[]']

  def __init__(self, scenario: bw_interfaces_msg_SimulationConfig = None,
    objectives: List[bw_interfaces_msg_SimulationConfig] = None):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
      scenario,objectives

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    super(ConfigureSimulation, self).__init__(**{'scenario': scenario, 'objectives': objectives})
    if self.scenario is None:
      self.scenario: bw_interfaces_msg_SimulationConfig = bw_interfaces_msg_SimulationConfig()
    else:
      self.scenario = scenario
    if self.objectives is None:
      self.objectives: List[bw_interfaces_msg_SimulationConfig] = []
    else:
      self.objectives = objectives

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
      _x = self.scenario.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.scenario.json_data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.objectives)
      buff.write(_struct_I.pack(length))
      for val1 in self.objectives:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.json_data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, bytes_: bytes) -> 'ConfigureSimulation':
    """
    unpack serialized message in str into this message instance
    :param bytes_: byte array of serialized message, ``bytes``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.scenario is None:
        self.scenario = bw_interfaces_msg_SimulationConfig()
      if self.objectives is None:
        self.objectives = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.scenario.name = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.scenario.name = bytes_[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.scenario.json_data = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.scenario.json_data = bytes_[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.objectives = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_SimulationConfig()
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
          val1.json_data = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.json_data = bytes_[start:end]
        self.objectives.append(val1)
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
      _x = self.scenario.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.scenario.json_data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.objectives)
      buff.write(_struct_I.pack(length))
      for val1 in self.objectives:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.json_data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.scenario is None:
        self.scenario = bw_interfaces_msg_SimulationConfig()
      if self.objectives is None:
        self.objectives = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.scenario.name = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.scenario.name = bytes_[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      start = end
      end += length
      if python3:
        self.scenario.json_data = bytes_[start:end].decode('utf-8', 'rosmsg')
      else:
        self.scenario.json_data = bytes_[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(bytes_[start:end])
      self.objectives = []
      for i in range(0, length):
        val1 = bw_interfaces_msg_SimulationConfig()
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
          val1.json_data = bytes_[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.json_data = bytes_[start:end]
        self.objectives.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I