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

# TODO: Currently, only primitive types (including Time and Duration) are supported in messages.
# Consider decomposing non-primitive types into their primitive types and sending/receiving purely primitive data,
# rather than just skipping non-primitive types

# TODO: Check if strings work

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


class MessageConfig(object):

    def __init__(self):
        self.tx_id = 0
        self.rx_id = 0
        self.topic = ''
        self.topic_type = ''
        self.msg_type = ''
        self.format = ''


def parse_msg_format(msg_type):
    """
    Convert the datatypes of a ROS message to the appropriate format string
    Eg. float32, int32, int32, float64 -> "fiid"
    """
    format = ""
    for type in msg_type._slot_types:
        if type in FORMAT_CHARACTERS:
            format += FORMAT_CHARACTERS[type]
        else:
            format += 'x'
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


def CAN_read(msg):
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
    data = []
    cur_bit = 0

    for i, f in enumerate(config['format']):

        # Determine the length of the next value in the message
        if f in ['?', 'b', 'B']:
            data_len = 1
        elif f in ['h', 'H']:
            data_len = 2
        elif f in ['i', 'I', 'f']:
            data_len = 4
        elif f in ['q', 'Q', 'd']:
            data_len = 8

        elif f in ['s']:
            data_len = data[-1]
            data.pop()
            f = str(data_len) + f

        # If we have an x, we have an invalid data type. Insert an empty obj into the data array, and continue
        elif f is 'x':
            field_type = config['msg_type']._slot_types[i]
            data.append(get_msg_from_type(field_type)())
            cur_bit += 1
            continue
        else:
            # Uh oh, not sure how we ended up here
            # TODO: Need to handle this
            continue

        data.append(struct.unpack('!' + f, bytearray(msg.data[cur_bit:cur_bit + data_len]))[0])
        cur_bit += data_len

    for i, type in enumerate(config['msg_type']._slot_types):
        if type is 'time':
            time = rospy.Time(secs=data[i], nsecs=data[i + 1])
            data[i] = time
            del data[i + 1]
        elif type is 'duration':
            duration = rospy.Duration(secs=data[i], nsecs=data[i + 1])
            data[i] = duration
            del data[i + 1]

    # If the message has a header, populate the timestamp with the time the message was received on the bus
    if config['msg_type']._has_header:
        data[config['msg_type']._slot_types.index('std_msgs/Header')].stamp = rospy.Time().from_sec(
            msg.timestamp)

    # Convert the data array to a ROS msg, and publish it
    msg = config['msg_type'](*data)
    config['publisher'].publish(msg)


def subscriber_callback(data, msg_config):
    """
    Callback for receiving messages from ROS topics, to be forwarded to the CAN bus
    """
    rospy.logdebug('Got ROS msg on topic {0}\t\t{1}'.format(msg_config.topic, data))

    packed_data = []
    string_lengths = []
    cur_char = 0
    for i, slot in enumerate(data.__slots__):

        # If the data is a timestamp or duration, format it appropriately
        if data._slot_types[i] is 'time' or data._slot_types[i] is 'duration':
            packed_data.append(getattr(data, slot).secs)
            packed_data.append(getattr(data, slot).nsecs)
            cur_char += 2

        # If the data is a string, format it appropriately
        elif data._slot_types[i] is 'string':
            string_lengths.append(len(getattr(data, slot)))
            packed_data.append(len(getattr(data, slot)))
            packed_data.append(getattr(data, slot))
            cur_char += 4

        # If the data is not a primitive, skip it
        elif msg_config.format[cur_char] is 'x':
            cur_char += 1

        # If the data is any other valid type (Any other primitive), append it
        else:
            packed_data.append(getattr(data, slot))
            cur_char += 1

    format = msg_config.format.format(*string_lengths)

    wrapper.send_data(msg_config.tx_id, msg_config.rx_id,
                      bytearray(struct.pack("!" + format, *packed_data)))


# Initialize ROS, open the CAN bus, and setup the inputs & outputs
def initialize_node():
    global wrapper
    global input_msgs
    rospy.init_node('can_interface')

    # Fetch the CAN configurations from the yaml file
    bus_config = yaml.load(
        open(rospkg.RosPack().get_path('boat_interfaces') + "/config/can_bus.yaml"))

    # Open the CAN bus
    #bus = can.interfaces.serial.serial_can.SerialBus(channel="/dev/ttyUSB0") # For real CAN interface
    bus = can.Bus('slcan0', bustype='socketcan')
    wrapper = iso_tp_wrapper.IsoTPWrapper(bus)

    # Setup all the ROS Subscribe -> CAN Write configurations
    for msg_name, config in bus_config['output_msgs'].items():
        msg_config = parse_msg_config(config)
        rospy.loginfo(
            'Creating ROS->CAN config for msg {name}\t- ROS topic={topic},\tCAN TX_ID={tx_id}'.
            format(name=msg_name, topic=msg_config.topic, tx_id=msg_config.tx_id))
        rospy.Subscriber(msg_config.topic, msg_config.msg_type, subscriber_callback, msg_config)

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

    wrapper.register_msg_callback(CAN_read)

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
