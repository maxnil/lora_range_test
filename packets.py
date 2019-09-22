# TX Settings for different packets
from tools.crc8 import CRC8
import struct


class Packet:
    """
    Simple LoRa packet
    Only contains Packet Number, Packet Type and Packet Checksum
    """

    def __init__(self):
        self.pkt_nr = None       # 16-bits
        self.pkt_type = None     # 8-bits

    def to_bytes(self):
        """Convert Packet into bytearray and add CRC"""
        raw_packet = bytearray()
        raw_packet.extend(struct.pack('>HB', self.pkt_nr, self.pkt_type))
        crc = CRC8(raw_packet)
        raw_packet.extend(crc.digest())
        return raw_packet

    def from_bytes(self, raw_packet):
        """Convert bytearray into Packet"""
        crc = CRC8(raw_packet[:-1])  # Exclude last byte (the checksum byte)
        if not crc.digest() == raw_packet[-1]:
            print("###WARN: Packet checksum failed")
            return False

        try:
            self.pkt_nr, self.pkt_type = struct.unpack('>HB', raw_packet[-1])
        except struct.error:
            print("###ERR: Could not unpack packet")
            return False


class PingPacket(Packet):
    """Simple 'ping' packet"""

    def __init__(self):
        super.__init__()


# class RssiPacket(Packet):
class RssiPacket:
    """Simple LoRa packe with Rssi+SNR responce"""

    def __init__(self, pkt_nr=0, rx_pkt_rssi_value=b'\x00', rx_pkt_snr_value=b'\x00', payload=bytearray()):
        self.pkt_nr = pkt_nr                        # 16-bits
        self.rx_pkt_rssi_value = rx_pkt_rssi_value  # 8-bits
        self.rx_pkt_snr_value = rx_pkt_snr_value    # 8-bits
        self.payload = payload                      # 8*n-bits
        self.pkt_checksum = 0x00                    # 8-bits

    @staticmethod
    def checksum(raw_packet):
        """Simple XOR checksum"""
        result = 0x00
        for b in raw_packet:
            result ^= b
        return result

    def to_bytes(self):
        """Convert Packet into bytearray"""
        raw_packet = bytearray(0)
        raw_packet.extend(self.pkt_nr.to_bytes(2, 'big'))        # [0:1]
        raw_packet.extend(self.rx_pkt_rssi_value)                # [2]
        raw_packet.extend(self.rx_pkt_snr_value)                 # [3]
        raw_packet.extend(self.payload)                          # [4:]
        self.pkt_checksum = self.checksum(raw_packet)            # Calculate checksum
        raw_packet.extend(self.pkt_checksum.to_bytes(1, 'big'))  # [last]
        return raw_packet

    def from_bytes(self, raw_packet):
        """Convert bytearray into Packet"""
        if not self.checksum(raw_packet) == 0x00:
            print("Raw packet checksum failed")
            return False

        try:
            self.pkt_nr = int.from_bytes(raw_packet[0:2], 'big')  # [0:1]
            self.rx_pkt_rssi_value = raw_packet[2]                # [2]
            self.rx_pkt_rssi_value = raw_packet[3]                # [3]
            self.payload = raw_packet[4:len(raw_packet) - 1]      # [4:n-2]
            self.pkt_checksum = raw_packet[len(raw_packet) - 1]   # [n-1]
            return True
        except IndexError:
            print("Packet length (%d) to short" % len(raw_packet))
            return False
