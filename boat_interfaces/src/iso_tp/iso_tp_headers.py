import can


class Message(object):
    """
    A general message structure, containing data of an arbitrary length
    """

    def __init__(self, tx_id=0, rx_id=0, data=[]):
        self.tx_id = tx_id
        self.rx_id = rx_id
        self.data = data
        self.timestamp = None


class SingleFrame(can.Message):
    """
    Single CAN frame

    Each single frame contains up to 7 bytes, plus 1 PCI byte.

    Schema:
     - 0H = ISO-TP Frame Type = 0
     - 0L = Data length, from 1 to 7 bytes
     - 1-7 = Data
    """
    FRAME_TYPE = 0

    def __init__(self, data=[]):
        assert len(data) <= 7, 'Data is too large'

        payload = bytearray()
        payload.append((SingleFrame.FRAME_TYPE << 4) | (len(data) & 0x0F))
        payload += data

        super(SingleFrame, self).__init__(data=payload, extended_id=False)


class FirstFrame(can.Message):
    """
    First CAN frame in a multi-frame message

    Each first frame contains exactly 6 bytes of data, plus 2 PCI bytes,
    resulting in a total payload length of 8 bytes.

    Schema:
     - 0H = ISO-TP Frame Type = 1
     - 0L 1H 1L = Data length, from 8 to 4095 bytes
     - 2-7 = Data
    """
    FRAME_TYPE = 1

    def __init__(self, data=[]):
        assert len(data) <= 4095, 'Data is too large'

        # Payload will always be of length 8
        payload = bytearray()
        payload.append((FirstFrame.FRAME_TYPE << 4) | (len(data) >> 8 & 0x0F))
        payload.append(len(data) & 0xFF)
        payload += data[:6]

        super(FirstFrame, self).__init__(data=payload, extended_id=False)


class ConsecutiveFrame(can.Message):
    """
    Consecutive CAN frame in a multi-frame message

    Each consecutive frame contains 1 to 7 bytes of data, plus 1 PCI byte.

    The sequence number is 1 for the first consecutive frame in a message, and
    increments with each subsequent consecutive frame. It overflows/wraps at 15,
    resulting in a sequence (1, 2, 3, ... , 15, 0, 1, ... )

    Schema:
    - 0H = ISO-TP Frame Type = 2
    - 0L = Sequence number, from 0-15
    - 1-7 = Data
    """
    FRAME_TYPE = 2

    def __init__(self, sequence=0, data=[]):
        assert len(data) <= 4095, 'Data is too large'

        self.sequence = sequence
        offset = (sequence * 7) - 1

        payload = bytearray()
        payload.append((ConsecutiveFrame.FRAME_TYPE << 4)
                       | (self.sequence & 0x0F))
        payload += data[offset:offset + (min(len(data) - offset, 7))]

        super(ConsecutiveFrame, self).__init__(data=payload, extended_id=False)


class FlowFrame(can.Message):
    """
    Flow control CAN frame in a multi-frame message

    Each flow frame contains exactly 0 bytes of data, plus 3 PCI bytes,
    resulting in a total payload length of 3 bytes.

    This frame is a response from the receiver after the first frame,
    acknowledging the beginning of the message and setting the parameters for the
    transmission of the next set of frames in the message.

    Byte 1 of the PCI describes the number of frames that the sender may send
    before it must wait for the next flow control frame to be received. If it is
    0, then all remaining frames may be sent.

    Byte 2 of the PCI describes the minimum required separation time between the
    end of one message and the start of the next that the sender must adhere to.
    Values of 0 to 127 correspond to milliseconds. Values of 0xF1 to 0xF9
    correspond to 0.1-0.9 milliseconds. All other values are illegal.

    Schema:
    - 0H = ISO-TP Frame Type = 3
    - 0L = Flow status. 0=Continue, 1=Wait, 2=Abort
    - 1  = The number of frames that may be sent before the next flow frame
    - 2  = The separation time between the frames
    """
    FRAME_TYPE = 3
    STATUS_CONTINUE = 0
    STATUS_WAIT = 1
    STATUS_ABORT = 2

    def __init__(self, status, num_frames, separation):

        # Payload will always be of length 3
        payload = bytearray()
        payload.append((FlowFrame.FRAME_TYPE << 4) | (status & 0x0F))
        payload.append(num_frames)
        payload.append(separation)

        super(FlowFrame, self).__init__(data=payload, extended_id=False)
