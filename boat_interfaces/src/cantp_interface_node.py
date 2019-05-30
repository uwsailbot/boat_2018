#!/usr/bin/env python
import can
import can.interfaces.serial.serial_can
import importlib
import rospkg
import rospy
import rostopic
import struct
import yaml

from iso_tp import iso_tp_wrapper

# Conversion of ROS message primitives to format characters
FORMAT_CHARACTERS = {
    'bool': '?',  # 1 byte
    'int8': 'b',  # 1 byte
    'uint8': 'B',  # 1 byte
    'int16': 'h',  # 2 bytes
    'uint16': 'H',  # 2 bytes
    'int32': 'i',  # 4 bytes
    'uint32': 'I',  # 4 bytes
    'int64': 'q',  # 8 bytes
    'uint64': 'Q',  # 8 bytes
    'float32': 'f',  # 4 bytes
    'float64': 'd',  # 8 bytes
    'string': 'b{}s',  # Length, data. 1+n bytes
    'time': 'II',  # Sec, Nsec. 4+4 bytes
    'duration': 'ii',  # Sec, Nsec. 4+4 bytes
}

# Dict of input message configurations for CAN Read -> ROS Publish operations
# Key is RX ID
input_msgs = {}

# The iso_tp_wrapper instance
wrapper = ()


class MessageConfig(object):

    def __init__(self):
        self.tx_id = 0
        self.rx_id = 0
        self.topic = ''
        self.topic_type = ''
        self.msg_type = ''
        self.format = ''


def close_bracket(start_idx, text):
    """
    Find the index of the closing bracket of matching depth.
    """
    nest_level = 1
    for i, c in enumerate(text[start_idx + 1:]):
        if c == '[':
            nest_level += 1
        elif c == ']':
            nest_level -= 1
            if nest_level == 0:
                end_idx = start_idx + 2 + i
                return end_idx
    return -1


def parse_msg_format(msg_type):
    """
    Convert the datatypes of a ROS message to the appropriate format string
    Eg. float32, int32, int32, float64 -> "fiid"
    """
    format = ""
    for type in msg_type._slot_types:

        # Handle primitives
        if type in FORMAT_CHARACTERS:
            format += FORMAT_CHARACTERS[type]

        # Handle arrays
        elif type[-1] == ']':

            # Variable-size arrays
            if type[-2] == '[':
                format += 'b['
                base_type = type[:-2]

                if base_type in FORMAT_CHARACTERS:
                    format += FORMAT_CHARACTERS[base_type]
                else:
                    format += parse_msg_format(get_msg_from_type(base_type))

                format += ']'

            # Fixed-size arrays
            else:
                base_type = type.split('[')[0]
                arr_len = int(type.split('[')[1][:-1])

                if base_type in FORMAT_CHARACTERS:
                    format += FORMAT_CHARACTERS[base_type] * arr_len
                elif type != 'std_msgs/Header':
                    format += parse_msg_format(get_msg_from_type(base_type)) * arr_len

        # Handle nested msg objects, excluding Headers
        else:
            if type != 'std_msgs/Header':
                format += parse_msg_format(get_msg_from_type(type))
    return format


def get_msg_from_type(type):
    """
    Get the ROS msg object corresponding to the specified type
    """
    pkg = type.split("/")[0]
    msg = type.split("/")[1]
    return getattr(importlib.import_module(pkg + ".msg"), msg)


def parse_msg_config(yaml):
    """
    Parse the CAN/ROS message configuration from the yaml file
    """
    msg_config = MessageConfig()
    msg_config.topic = yaml['topic']

    # If the topic type wasn't specified in the yaml, fetch it from the topic.
    # This will fail if the topic doesn't exist yet, so to prevent race conditions when using
    # roslaunch, we recommend specifying the topic type in the yaml.
    if 'topic_type' in yaml:
        msg_config.topic_type = yaml['topic_type']
    else:
        msg_config.topic_type = rostopic._get_topic_type(msg_config.topic)[0]

    msg_config.msg_type = get_msg_from_type(msg_config.topic_type)

    # If the message format wasn't specified in the yaml, fetch it from the message type.
    # Note: This will fail for strings!!
    if 'format' in yaml:
        msg_config.topic_type = yaml['format']
    else:
        msg_config.format = parse_msg_format(msg_config.msg_type)

    msg_config.tx_id = yaml['tx_id']
    msg_config.rx_id = yaml['rx_id']

    return msg_config


def unpack(data_format, raw_data):
    """
    Unpack a raw byte array to a list of values based on the specified data format
    """

    data = []
    array_lengths = []
    cur_bit = 0

    i = 0
    while i < len(data_format):
        f = data_format[i]
        i += 1

        # Determine the length of the next value
        if f in ['?', 'b', 'B']:
            data_len = 1
        elif f in ['h', 'H']:
            data_len = 2
        elif f in ['i', 'I', 'f']:
            data_len = 4
        elif f in ['q', 'Q', 'd']:
            data_len = 8

        # Strings
        elif f in ['{', '}']:
            continue
        elif f in ['f']:
            data_len = data[-1]
            data.pop()
            f = str(data_len) + f

        # Variable-length arrays
        elif f in ['[']:
            arr_len = data[-1]
            data.pop()
            array_lengths.append(arr_len)

            start_idx = i - 1
            end_idx = close_bracket(start_idx, data_format)

            array_format = data_format[start_idx + 1:end_idx - 1]
            array_contents = []
            for i in range(arr_len):
                contents, num_bits = unpack(array_format, raw_data[cur_bit:])
                cur_bit += num_bits
                array_contents.append(list(contents))
            data.append(array_contents)

            i += (end_idx - start_idx) + 1
            continue

        else:
            print('Error')

        data.append(struct.unpack('!' + f, bytearray(raw_data[cur_bit:cur_bit + data_len]))[0])
        cur_bit += data_len

    return data, cur_bit


def list_to_msg(data, ros_msg):
    """
    Convert a list of fields to its corresponding ROS message
    """

    for type_name, slot in zip(ros_msg._slot_types, ros_msg.__slots__):

        # Skip headers
        if type_name == 'std_msgs/Header':
            continue

        # Handle arrays
        elif type_name[-1] == ']':

            # Append every element of the list
            for element in data[0]:
                if '/' in type_name:
                    getattr(ros_msg, slot).append(
                        list_to_msg(element,
                                    get_msg_from_type(type_name.split('[')[0])()))
                else:
                    getattr(ros_msg, slot).append(element[0])
            del data[0]

        # Handle special primitives - time and duration
        elif type_name == 'time' or type_name == 'duration':
            getattr(ros_msg, slot).secs = data[0]
            getattr(ros_msg, slot).nsecs = data[1]
            del data[0]
            del data[1]

        # Handle primitives
        elif type_name in FORMAT_CHARACTERS:
            setattr(ros_msg, slot, data[0])
            del data[0]

        # Handle nested messages
        else:
            setattr(ros_msg, slot, list_to_msg(data, get_msg_from_type(type_name)()))
            del data[0]

    return ros_msg


def msg_to_list(msg):
    """
    Convert a ROS message to an array of its fields
    """

    packed_data = []
    string_lengths = []
    array_lengths = []
    for i, slot in enumerate(msg.__slots__):

        # Skip headers:
        if msg._slot_types[i] == 'std_msgs/Header':
            continue

        # Handle arrays
        elif msg._slot_types[i][-1] == ']':

            # For variable-size arrays, append the array length
            if msg._slot_types[i][-2] == '[':
                array_lengths.append(len(getattr(msg, slot)))
                packed_data.append(len(getattr(msg, slot)))

            # Append every element of the list
            for data in getattr(msg, slot):
                if hasattr(data, '__slots__'):
                    new_packed_data, new_string_lengths, new_array_lengths = msg_to_list(data)
                    packed_data += new_packed_data
                    string_lengths += new_string_lengths
                    array_lengths += new_array_lengths
                else:
                    packed_data.append(data)

        # Handle special primitives - time and duration
        elif msg._slot_types[i] == 'time' or msg._slot_types[i] == 'duration':
            packed_data.append(getattr(msg, slot).secs)
            packed_data.append(getattr(msg, slot).nsecs)

        # Handle special primitives - string
        elif msg._slot_types[i] == 'string':
            string_lengths.append(len(getattr(msg, slot)))
            packed_data.append(len(getattr(msg, slot)))
            packed_data.append(getattr(msg, slot))

        # Handle primitives
        elif msg._slot_types[i] in FORMAT_CHARACTERS:
            packed_data.append(getattr(msg, slot))

        # Handle nested messages
        else:
            new_packed_data, new_string_lengths, new_array_lengths = msg_to_list(getattr(msg, slot))

            packed_data += new_packed_data
            string_lengths += new_string_lengths
            array_lengths += new_array_lengths

    return packed_data, string_lengths, array_lengths


def can_to_ros(msg):
    """
    Callback for receiving messages from the CAN bus, to be forwarded to ROS topics
    """

    # If the message is unknown, ignore it
    if msg.rx_id not in input_msgs:
        return

    rospy.logdebug("CAN RX ID: {0}, raw data: '{1}', ts: {2}".format(msg.rx_id, bytearray(msg.data),
                                                                     msg.timestamp))

    config = input_msgs[msg.rx_id]

    # Parse the message from the CAN bus into an array
    data, _ = unpack(config['format'], msg.data)

    # Convert the data array to a ROS msg
    ros_msg = list_to_msg(data, config['msg_type']())

    # If the message has a header, populate the timestamp with the time the message was received on the bus
    if ros_msg._has_header:
        field_name = ros_msg.__slots__[ros_msg._slot_types.index('std_msgs/Header')]
        getattr(ros_msg, field_name).stamp = rospy.Time().from_sec(msg.timestamp)

    # Publish the msg
    config['publisher'].publish(ros_msg)


def ros_to_can(data, msg_config):
    """
    Callback for receiving messages from ROS topics, to be forwarded to the CAN bus
    """
    rospy.logdebug('Got ROS msg on topic {0}\t\t{1}'.format(msg_config.topic, data))

    # Convert the msg to a list of fields
    packed_data, string_lengths, array_lengths = msg_to_list(data)

    # Parse the message format, expanding arrays and inserting string lengths
    data_format = msg_config.format
    for arrlen in array_lengths:
        start_idx = data_format.find('[')
        end_idx = close_bracket(start_idx, data_format)

        data_format = data_format.replace(data_format[start_idx:end_idx],
                                          data_format[start_idx + 1:end_idx - 1] * arrlen, 1)

    data_format = data_format.format(*string_lengths)

    # Convert the data to a bytearray and send it on the bus
    # print(data_format, packed_data, list(bytearray(struct.pack("!" + data_format, *packed_data))))
    wrapper.send_data(msg_config.tx_id, msg_config.rx_id,
                      bytearray(struct.pack("!" + data_format, *packed_data)))


def initialize_node():
    """
    Initialize ROS node, open the CAN bus, and setup the inputs & outputs
    """
    global wrapper
    global input_msgs
    rospy.init_node('can_interface')

    # Fetch the CAN configurations from the yaml file
    bus_config = yaml.load(
        open(rospkg.RosPack().get_path('boat_interfaces') + "/config/can_bus.yaml"))

    # Open the CAN bus
    bus = can.Bus('slcan0', bustype='socketcan')
    wrapper = iso_tp_wrapper.IsoTPWrapper(bus)

    # Setup all the ROS Subscribe -> CAN Write configurations
    for msg_name, config in bus_config['output_msgs'].items():
        msg_config = parse_msg_config(config)
        rospy.loginfo(
            'Creating ROS->CAN config for msg {name}\t- ROS topic={topic},\tCAN TX_ID={tx_id}'.
            format(name=msg_name, topic=msg_config.topic, tx_id=msg_config.tx_id))
        rospy.Subscriber(msg_config.topic, msg_config.msg_type, ros_to_can, msg_config)

    # Setup all the CAN Read -> ROS Publish configurations
    for _, config in bus_config['input_msgs'].items():
        msg_config = parse_msg_config(config)

        rospy.loginfo(
            'Creating CAN->ROS config for msg {name}\t- ROS topic={topic},\tCAN RX_ID={rx_id}'.
            format(name=msg_name, topic=msg_config.topic, rx_id=msg_config.rx_id))

        pub = rospy.Publisher(msg_config.topic, msg_config.msg_type, queue_size=10)
        input_msgs[msg_config.rx_id] = {
            'publisher': pub,
            'msg_type': msg_config.msg_type,
            'format': msg_config.format
        }

        wrapper.config_flow_frame(msg_config.rx_id, msg_config.tx_id, 1, 5)

    wrapper.register_msg_callback(can_to_ros)

    # Spin until shutdown
    while not rospy.is_shutdown():
        wrapper.update()
        rospy.sleep(0.01)  # TODO: Determine rate

    bus.shutdown()


if __name__ == '__main__':
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
