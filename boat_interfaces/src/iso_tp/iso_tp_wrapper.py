import can
import enum
import time

import iso_tp_headers


def get_time_micros():
    return time.time() * 1000000


class IsoTPWrapper(object):

    class ArbitrationMode(enum.Enum):
        PRIORITY_ID = 0
        LIFO = 1
        FIFO = 2

    class TxConfig(object):

        def __init__(self):
            self.msg = iso_tp_headers.Message()

            # Tracking transmission
            # yapf: disable
            self.waiting = False                      # Is waiting for flow frame
            self.sequence = 0                         # Sequence of last sent consecutive frame
            self.remaining_frames = 0                 # Remaining num of frames that may be send
            self.prev_frame_time = get_time_micros()  # Time the last frame was sent, in us
            # yapf: enable

            # Configuration
            self.separation = 0  # Separation time between frames, in 0.1ms

    class RxConfig(object):

        def __init__(self):
            self.msg = iso_tp_headers.Message()

            # Tracking transmission
            # yapf: disable
            self.is_active = False     # Transmission is active
            self.sequence = 0          # Sequence of last received consecutive frame
            self.remaining_frames = 0  # Remaining num of frames before Flow Frame
            # yapf: enable

            # Configuration
            self.block_size = 0  # Num of frames that may be sent
            self.separation = 0  # Separation time between frames, in 0.1ms

    def __init__(self, bus, arbitration_mode=None):

        self._tx_msgs = []
        self._rx_msgs = []

        self._bus = bus

        if arbitration_mode is None:
            self._arbitration_mode = IsoTPWrapper.ArbitrationMode.PRIORITY_ID
        else:
            self._arbitration_mode = arbitration_mode

        class CallbackListener(can.Listener):

            def __init__(self, callback):
                super(CallbackListener, self).__init__()
                self.callback = callback

            def on_message_received(self, msg):
                self.callback(msg)

        can.Notifier(bus, [CallbackListener(self._read_next)])

        self._callbacks = []

    def config_flow_frame(self, rx_id, tx_id, block_size, separation):
        """
        Add the specified handshake details to the wrapper's list of recognized messages
        """
        config = IsoTPWrapper.RxConfig()
        config.msg.rx_id = rx_id
        config.msg.tx_id = tx_id
        config.block_size = block_size
        config.separation = separation
        self._rx_msgs.append(config)

        return True

    def remove_flow_frame(self, rx_id, tx_id):
        """
        Remove the specified message from the wrapper's list of recognized wrappers
        """
        for rx_msg in self._rx_msgs:
            if rx_msg.msg.rx_id == rx_id and rx_msg.msg.tx_id == tx_id:
                self._rx_msgs.remove(rx_msg)
                return True
        return False

    def update(self):
        """
        Update all outgoing messages that are mid-transmission. Must be called regularily
        """

        # Check if outgoing messages are ready to send their next consecutive frame
        for tx_msg in self._tx_msgs[:]:
            if tx_msg.waiting or (get_time_micros() -
                                  tx_msg.prev_frame_time) / 100 < tx_msg.separation:
                continue

            # Update the msg data
            tx_msg.sequence += 1
            tx_msg.prev_frame_time = get_time_micros()
            if tx_msg.remaining_frames == 0:
                pass
            elif tx_msg.remaining_frames == 1:
                tx_msg.waiting = True
            else:
                tx_msg.remaining_frames -= 1

            # Send the next frame
            consec = iso_tp_headers.ConsecutiveFrame(tx_msg.sequence, tx_msg.msg.data)
            consec.arbitration_id = tx_msg.msg.tx_id
            self._bus.send(consec)

            # If all the data has been sent, cleanup
            if (tx_msg.sequence * 7) + 6 >= len(tx_msg.msg.data):
                self._tx_msgs.remove(tx_msg)

    def register_msg_callback(self, callback):
        """
        Register a callback function for when a message is fully received
        """
        self._callbacks.append(callback)

    def send_data(self, tx_id, rx_id, data):
        msg = iso_tp_headers.Message(tx_id=tx_id, rx_id=rx_id, data=data)
        return self.send_msg(msg)

    def send_msg(self, msg):
        """
        Queue a message of arbitrary length for transmission
        """

        # If the message is very short, send a Single Frame
        if len(msg.data) < 8:
            frame = iso_tp_headers.SingleFrame(msg.data)
            frame.arbitration_id = msg.tx_id
            return self._bus.send(frame)

        # Create a new message with the data
        config = IsoTPWrapper.TxConfig()
        config.msg = msg
        config.sequence = 0
        config.waiting = True
        self._tx_msgs.append(config)

        # Send the first frame
        frame = iso_tp_headers.FirstFrame(config.msg.data)
        frame.arbitration_id = config.msg.tx_id
        return self._bus.send(frame)

    def _read_next(self, recv_msg):
        """
        INTERNAL - Read the next frame from the CANbus and handle it
        """

        # Check the type of the received frame
        # We always ignore malformed frames
        msg_type = recv_msg.data[0] >> 4

        # If the received frame is a Single Frame, repackage the msg and notify the callbacks immediately
        if msg_type == iso_tp_headers.SingleFrame.FRAME_TYPE:
            msg = iso_tp_headers.Message()
            msg.timestamp = recv_msg.timestamp
            msg.rx_id = recv_msg.arbitration_id
            msg.data = recv_msg.data[1:]

            for callback in self._callbacks:
                callback(msg)

        # If the received frame is a First Frame, prepare for consecutive frames
        elif msg_type == iso_tp_headers.FirstFrame.FRAME_TYPE:
            for rx_msg in self._rx_msgs:
                if rx_msg.msg.rx_id != recv_msg.arbitration_id:
                    continue

                # Append the new data, prepare for consecutive frames
                rx_msg.msg.maxlen = recv_msg.data[1]
                rx_msg.msg.maxlen |= (recv_msg.data[0] & 0x0F) << 8

                rx_msg.msg.data = recv_msg.data[2:]
                rx_msg.sequence = 0
                rx_msg.is_active = True
                rx_msg.remaining_frames = rx_msg.block_size

                # Send flow frame
                self._send_flow_frame(rx_msg, 0)

                break

        # If the received frame is a Consecutive Frame, append the data
        elif msg_type == iso_tp_headers.ConsecutiveFrame.FRAME_TYPE:
            for rx_msg in self._rx_msgs:
                if rx_msg.msg.rx_id != recv_msg.arbitration_id:
                    continue

                if not rx_msg.is_active:
                    print('Err: Got consec frame, msg inactive')
                    return

                rx_msg.sequence += 1
                if (rx_msg.sequence & 0x0F) != (recv_msg.data[0] & 0x0F):
                    print('Err: Got consec frame, bad sequence')
                    return

                start = (rx_msg.sequence * 7) - 1
                if (start + len(recv_msg.data) - 1 > rx_msg.msg.maxlen):
                    print("Err: Got consec frame, data too long")
                    return


                # Append the new data
                rx_msg.msg.data.extend(recv_msg.data[1:])
                rx_msg.remaining_frames -= 1

                # If this is the last frame in the message, trigger the callback and cleanup
                if start + 7 > len(rx_msg.msg.data):
                    rx_msg.msg.timestamp = recv_msg.timestamp
                    for callback in self._callbacks:
                        callback(rx_msg.msg)
                    rx_msg.sequence = 0
                    rx_msg.is_active = False
                    rx_msg.msg.data = []

                # If this is not the last frame of the message, but it is the last
                # frame of the block, send the next flow frame
                elif rx_msg.remaining_frames == 0:
                    self._send_flow_frame(rx_msg, 0)
                    rx_msg.remaining_frames = rx_msg.block_size

                break

        # If the received frame is a Flow Control Frame, continue transmission
        elif msg_type == iso_tp_headers.FlowFrame.FRAME_TYPE:
            for tx_msg in self._tx_msgs:
                if tx_msg.msg.rx_id != recv_msg.arbitration_id:
                    continue

                status = recv_msg.data[0] & 0x0F

                # Continue transmission
                if status == iso_tp_headers.FlowFrame.STATUS_CONTINUE:
                    val = recv_msg.data[2]
                    if val >> 4 == 1:
                        tx_msg.separation = val & 0x0F
                    else:
                        tx_msg.separation = val * 10
                    tx_msg.waiting = False
                    tx_msg.remaining_frames = recv_msg.data[1]
                    tx_msg.prev_frame_time = get_time_micros()

                # Wait for next flow control frame
                elif status == iso_tp_headers.FlowFrame.STATUS_WAIT:
                    tx_msg.waiting = True

                # Abort transmission
                elif status == iso_tp_headers.FlowFrame.STATUS_ABORT:
                    self._tx_msgs.remove(tx_msg)

                break

        # Malformed frame
        else:
            pass

    def _send_flow_frame(self, config, status):
        frame = iso_tp_headers.FlowFrame(status, config.block_size, config.separation)
        frame.arbitration_id = config.msg.tx_id
        self._bus.send(frame)
