# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_imu_init:msg/States.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'rot_end'
# Member 'pos_end'
# Member 'vel_end'
# Member 'bias_gyr'
# Member 'bias_acc'
# Member 'gravity'
# Member 'cov'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_States(type):
    """Metaclass of message 'States'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('lidar_imu_init')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lidar_imu_init.msg.States')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__states
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__states
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__states
            cls._TYPE_SUPPORT = module.type_support_msg__msg__states
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__states

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class States(metaclass=Metaclass_States):
    """Message class 'States'."""

    __slots__ = [
        '_header',
        '_rot_end',
        '_pos_end',
        '_vel_end',
        '_bias_gyr',
        '_bias_acc',
        '_gravity',
        '_cov',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'rot_end': 'sequence<double>',
        'pos_end': 'sequence<double>',
        'vel_end': 'sequence<double>',
        'bias_gyr': 'sequence<double>',
        'bias_acc': 'sequence<double>',
        'gravity': 'sequence<double>',
        'cov': 'sequence<double>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.rot_end = array.array('d', kwargs.get('rot_end', []))
        self.pos_end = array.array('d', kwargs.get('pos_end', []))
        self.vel_end = array.array('d', kwargs.get('vel_end', []))
        self.bias_gyr = array.array('d', kwargs.get('bias_gyr', []))
        self.bias_acc = array.array('d', kwargs.get('bias_acc', []))
        self.gravity = array.array('d', kwargs.get('gravity', []))
        self.cov = array.array('d', kwargs.get('cov', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.rot_end != other.rot_end:
            return False
        if self.pos_end != other.pos_end:
            return False
        if self.vel_end != other.vel_end:
            return False
        if self.bias_gyr != other.bias_gyr:
            return False
        if self.bias_acc != other.bias_acc:
            return False
        if self.gravity != other.gravity:
            return False
        if self.cov != other.cov:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def rot_end(self):
        """Message field 'rot_end'."""
        return self._rot_end

    @rot_end.setter
    def rot_end(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'rot_end' array.array() must have the type code of 'd'"
            self._rot_end = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'rot_end' field must be a set or sequence and each value of type 'float'"
        self._rot_end = array.array('d', value)

    @property
    def pos_end(self):
        """Message field 'pos_end'."""
        return self._pos_end

    @pos_end.setter
    def pos_end(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'pos_end' array.array() must have the type code of 'd'"
            self._pos_end = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'pos_end' field must be a set or sequence and each value of type 'float'"
        self._pos_end = array.array('d', value)

    @property
    def vel_end(self):
        """Message field 'vel_end'."""
        return self._vel_end

    @vel_end.setter
    def vel_end(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'vel_end' array.array() must have the type code of 'd'"
            self._vel_end = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'vel_end' field must be a set or sequence and each value of type 'float'"
        self._vel_end = array.array('d', value)

    @property
    def bias_gyr(self):
        """Message field 'bias_gyr'."""
        return self._bias_gyr

    @bias_gyr.setter
    def bias_gyr(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'bias_gyr' array.array() must have the type code of 'd'"
            self._bias_gyr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'bias_gyr' field must be a set or sequence and each value of type 'float'"
        self._bias_gyr = array.array('d', value)

    @property
    def bias_acc(self):
        """Message field 'bias_acc'."""
        return self._bias_acc

    @bias_acc.setter
    def bias_acc(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'bias_acc' array.array() must have the type code of 'd'"
            self._bias_acc = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'bias_acc' field must be a set or sequence and each value of type 'float'"
        self._bias_acc = array.array('d', value)

    @property
    def gravity(self):
        """Message field 'gravity'."""
        return self._gravity

    @gravity.setter
    def gravity(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'gravity' array.array() must have the type code of 'd'"
            self._gravity = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'gravity' field must be a set or sequence and each value of type 'float'"
        self._gravity = array.array('d', value)

    @property
    def cov(self):
        """Message field 'cov'."""
        return self._cov

    @cov.setter
    def cov(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'cov' array.array() must have the type code of 'd'"
            self._cov = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'cov' field must be a set or sequence and each value of type 'float'"
        self._cov = array.array('d', value)
