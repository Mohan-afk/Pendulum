import can
import serial
import time

class ATCanBus(can.BusABC):
    def __init__(self, channel='/dev/ttyUSB0', bitrate=1000000, **kwargs):
        self.ser = serial.Serial(channel, baudrate=921600, timeout=0.005)
        self._buffer = bytearray()
        time.sleep(0.3)
        super().__init__(channel, **kwargs)

    def send(self, msg, timeout=None):
        can_id = int(msg.arbitration_id)
        data = bytes(msg.data)
        frame = b'AT' + can_id.to_bytes(4, 'big') + bytes([len(data)]) + data + b'\r\n'
        self.ser.write(frame)

    def recv(self, timeout=1.0):
        end = time.time() + (timeout or 1.0)
        while time.time() < end:
            chunk = self.ser.read(64)
            if chunk:
                self._buffer += chunk
            start = self._buffer.find(b'AT')
            if start == -1:
                self._buffer.clear()
                continue
            end_idx = self._buffer.find(b'\r\n', start)
            if end_idx == -1:
                self._buffer = self._buffer[start:]
                continue
            frame = self._buffer[start:end_idx+2]
            self._buffer = self._buffer[end_idx+2:]
            if len(frame) < 9:
                continue
            can_id = int.from_bytes(frame[2:6], 'big')
            dlc = frame[6]
            data = frame[7:7+dlc]
            return can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True,
                timestamp=time.time()
            )
        return None

    def shutdown(self):
        self.ser.close()

    @staticmethod
    def _detect_available_configs():
        return []
