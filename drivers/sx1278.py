"""
SX1278 Driver class
(C) 2019 Max Nilsson
"""


# Byte to integer (signed or unsigned)
def byte_to_int(value, signed=False):
    """Simple byte to signed/unsigned int (since int.from_bytes does not work in MicroPython"""
    val = int.from_bytes(value, 'big')
    return val if not signed or val <= 127 else val - 256


class SX1278:
    """SX1278 LoRa module class with methods for send and receive response over LoRa"""

    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_PA_CONFIG = 0x09
    REG_PA_RAMP = 0x0A
    REG_OCP = 0x0B
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS_MASK = 0x11
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_RX_HEADER_CNT_VALUE_MSB = 0x14
    REG_RX_HEADER_CNT_VALUE_LSB = 0x15
    REG_RX_PACKET_CNT_VALUE_MSB = 0x16
    REG_RX_PACKET_CNT_VALUE_LSB = 0x17
    REG_MODEM_STAT = 0x18
    REG_PKT_SNR_VALUE = 0x19
    REG_PKT_RSSI_VALUE = 0x1A
    REG_RSSI_VALUE = 0x1B
    REG_HOP_CHANNEL = 0x1C
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_SYMB_TIMEOUT_LSB = 0x1F
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_PAYLOAD_LENGTH = 0x22
    REG_MAX_PAYLOAD_LENGTH = 0x23
    REG_HOP_PERIOD = 0x24
    REG_FIFO_RX_BYTE_ADDR = 0x25
    REG_MODEM_CONFIG_3 = 0x26
    REG_FREQ_ERROR_MSB = 0x28
    REG_FREQ_ERROR_MID = 0x29
    REG_FREQ_ERROR_LSB = 0x2A
    REG_RSSI_WIDEBAND = 0x2C
    REG_IF_FREQ_1 = 0x2F
    REG_IF_FREQ_2 = 0x30
    REG_DETECTION_OPTIMIZE = 0x31
    REG_INVERT_IQ = 0x33
    REG_HIGH_BW_OPTIMIZE_1 = 0x36
    REG_DETECTION_THRESHOLD = 0x37
    REG_SYNC_WORD = 0x39
    REG_HIGH_BW_OPTIMIZE_2 = 0x3A
    REG_INVERT_IQ_2 = 0x3B
    REG_DIO_MAPPING_1 = 0x40
    REG_DIO_MAPPING_2 = 0x41
    REG_VERSION = 0x42
    REG_TCXO = 0x4B
    REG_PA_DAC = 0x4D
    REG_FORMER_TEMP = 0x5B
    REG_AGC_REF = 0x61
    REG_AGC_THRESH_1 = 0x62
    REG_AGC_THRESH_2 = 0x63
    REG_AGC_THRESH_3 = 0x64
    REG_PLL = 0x70

    # Oscillator frequency
    FXOSC = 32e6

    # Modemxc
    MODEM_LORA = 1 << 7
    MODEM_LOW_FREQUENCY = 1 << 3

    # RegOpMode Device Modes
    MODE_LORA_MODE = 0b10000000  # LoRa mode
    MODE_SLEEP = 0b000           # Sleep
    MODE_STDBY = 0b001           # Standby
    MODE_FSTX = 0b010            # Frequency synthesis TX (FSTX)
    MODE_TX = 0b011              # Transmit (TX)
    MODE_FSRX = 0b100            # Frequency synthesis RX (FSRX)
    MODE_RXCONTINOUS = 0b101     # Receive continuous (RXCONTINUOUS)
    MODE_RXSINGLE = 0b110        # Receive single (RXSINGLE)
    MODE_CAD = 0b111             # Channel activity detection (CAD)

    # Interrupts
    IRQ_CAD_DETECTED = 1 << 0
    IRQ_FHSS_CHANGE_CHANNEL = 1 << 1
    IRQ_CAD_DONE = 1 << 2
    IRQ_TX_DONE = 1 << 3
    IRQ_VALID_HEADER = 1 << 4
    IRQ_PAYLOAD_CRC_ERROR = 1 << 5
    IRQ_RX_DONE = 1 << 6
    IRQ_RX_TIMEOUT = 1 << 7

    # Status codes
    STATUS_OK = 0
    STATUS_INVALID_HEADER = 1
    STATUS_PAYLOAD_CRC_ERROR = 2
    STATUS_RX_TIMEOUT = 3
    STATUS_ABORT = 4

    # Bandwidth
    bandwidth = {
        '7.8 kHz':   0b0000,
        '10.4 kHz':  0b0001,
        '15.6 kHz':  0b0010,
        '20.8 kHz':  0b0011,
        '31.25 kHz': 0b0100,
        '41.7 kHz':  0b0101,
        '62.5 kHz':  0b0110,
        '125 kHz':   0b0111,
        '250 kHz':   0b1000,
        '500 kHz':   0b1001
    }

    # Default parameters
    parameters = {
        'frequency': 434.0e6,      # RF Frequency in Hz:
        'bandwidth': '7.8 kHz',    # Bandwidth in Hz: '7.8 kHz' - '500 kHz'
        'spreading_factor': 7,     # Spreading Factor: 6 - 12
        'coding_rate': 1,          # Coding Rate: 1 - 4
        'preamble_length': 6,      # Preamble Length: 6 - 65535
        'implicit_header': False,  # Implicit (no) Header: True, False
        'max_power': 10.8,         # Max RF Power: 10.8 - 15 dBm
        'output_power': 8,         # Output Power: 0 - 15 dBm
        'pa_select': 1,            # PA Select: 0, 1 (PA_BOOST)
        'rx_symb_timeout': 1023,   # RX Symbol Timeout, 4 - 1023
        'payload_crc': True        # Payload CRC enable: True, False
    }

    def __init__(self, spi_ctrl, parameters=None):
        self._spi_ctrl = spi_ctrl
        self._rxcontinous_enabled = False
        self.init(parameters)

    """Initialize the SX1278 module"""
    def init(self, parameters=None):
        # Override default parameters
        if parameters:
            self.parameters.update(parameters)

        # Set Modem in LoRa Mode
        self.set_lora_mode(True)

        # Configure Modem
        self.set_mode(self.MODE_STDBY)
        self.set_rf_frequency(self.parameters['frequency'])
        self.set_max_power(self.parameters['pa_select'], self.parameters['max_power'])
        self.set_output_power(self.parameters['output_power'])
        self.set_bandwidth(self.parameters['bandwidth'])
        self.set_coding_rate(self.parameters['coding_rate'])
        self.set_implicit_header(self.parameters['implicit_header'])
        self.set_spreading_factor(self.parameters['spreading_factor'])
        self.set_payload_crc(self.parameters['payload_crc'])
        self.set_symbol_timeout(self.parameters['rx_symb_timeout'])
        self.set_tx_continous_mode(False)
        self.set_preamble_length(self.parameters['preamble_length'])

    """Read register"""
    def read_reg(self, address, signed=False):
        response = self._spi_ctrl.transfer(address & 0x7F)
        return byte_to_int(response, signed=signed)

    """Write register"""
    def write_reg(self, address, value):
        self._spi_ctrl.transfer(address | 0x80, value)

    """Read FIFO"""
    def read_fifo(self, length=0):
        return self._spi_ctrl.read_bytes(self.REG_FIFO, length)

    """Write FIFO"""
    def write_fifo(self, value):
        self._spi_ctrl.write_bytes(self.REG_FIFO, value)

    """Set LoRa Mode"""
    def set_lora_mode(self, low_frequency):
        lfm = self.MODEM_LOW_FREQUENCY if low_frequency else 0
        self.write_reg(self.REG_OP_MODE, self.MODEM_LORA | lfm | self.MODE_SLEEP)  # LoRa can only be enabled in Sleep

    """Set RF frequency"""
    def set_rf_frequency(self, rf_freq):
        assert 433.05e6 <= rf_freq <= 434.79e6, \
            "RF frequency %.2f MHz is outside the ISM433 band (433.05 MHz - 434.79 MHz" % (rf_freq / 1e6)
        frf = int(rf_freq * (2 ** 19) / self.FXOSC)
        self._spi_ctrl.write_bytes(self.REG_FRF_MSB, frf.to_bytes(3, 'big'))

    """Set Bandwidth"""
    def set_bandwidth(self, bandwidth):
        current = self.read_reg(self.REG_MODEM_CONFIG_1) & 0b00001111  # Mask bandwidth
        bw = self.bandwidth[bandwidth]
        self.write_reg(self.REG_MODEM_CONFIG_1, current | bw << 4)

    """Set Coding Rate"""
    def set_coding_rate(self, coding_rate):
        current = self.read_reg(self.REG_MODEM_CONFIG_1) & 0b11110001  # Mask coding
        self.write_reg(self.REG_MODEM_CONFIG_1, current | coding_rate << 1)

    """Set Implicit Header"""
    def set_implicit_header(self, implicit):
        current = self.read_reg(self.REG_MODEM_CONFIG_1) & 0b11111110  # Mask implicit header
        ih = 1 if implicit else 0
        self.write_reg(self.REG_MODEM_CONFIG_1, current | ih)

    """Set output power (0-15)"""
    def set_output_power(self, output_power):
        assert 0 <= output_power <= 15, "Output power %d dBm is out of range (0 - 15)" % output_power
        current = self.read_reg(self.REG_PA_CONFIG) & 0b11110000  # Mask output power
        self.write_reg(self.REG_PA_CONFIG, current | output_power)

    """Set pa_select (0, 1) and max output power (10.8 - 15)"""
    def set_max_power(self, pa_select, max_power):
        assert 10.8 <= max_power <= 15, "Max RF power %f dBm is out of range (10.8 - 15)" % max_power
        current = self.read_reg(self.REG_PA_CONFIG) & 0b00001111  # Mask pa_select and max power
        max_power_setting = int(round((max_power - 10.8) / 0.6)) & 0b111
        self.write_reg(self.REG_PA_CONFIG, current | pa_select << 7 | max_power_setting << 4)

    """Set Spreading Factor"""
    def set_spreading_factor(self, spreading_factor):
        assert 6 <= spreading_factor <= 12, "Spreading Factor %d is out of range (6 - 12)" % spreading_factor
        current = self.read_reg(self.REG_MODEM_CONFIG_2) & 0b00001111  # Mask spreading factor
        self.write_reg(self.REG_MODEM_CONFIG_2, current | spreading_factor << 4)

    """Set Payload CRC (True/False)"""
    def set_payload_crc(self, payload_crc):
        current = self.read_reg(self.REG_MODEM_CONFIG_2) & 0b11111011  # Mask Payload CRC
        pcrc = 0b100 if payload_crc else 0b000
        self.write_reg(self.REG_MODEM_CONFIG_2, current | pcrc)

    """Set TX Continous Mode (True/False)"""
    def set_tx_continous_mode(self, tx_continous_mode):
        current = self.read_reg(self.REG_MODEM_CONFIG_2) & 0b11110111  # TX Mode
        tx_cont = 0b1000 if tx_continous_mode else 0b0000
        self.write_reg(self.REG_MODEM_CONFIG_2, current | tx_cont)

    """Set Symbol Timeout (4 - 1023)"""
    def set_symbol_timeout(self, symbol_timeout):
        current = self.read_reg(self.REG_MODEM_CONFIG_2) & 0b11111100  # Mask Symbol Timeout MSB
        st_msb = (symbol_timeout >> 8) & 0x03  # Bit[9:8]
        self.write_reg(self.REG_MODEM_CONFIG_2, current | st_msb)
        st_lsb = symbol_timeout & 0xFF  # Bit[7:0]
        self.write_reg(self.REG_SYMB_TIMEOUT_LSB, st_lsb)

    """Set Preamble Length (6 - 65535)"""
    def set_preamble_length(self, preamble_length):
        assert 6 <= preamble_length <= 65535, "Preamble length %d is outside valid range: 6 - 65535" % preamble_length
        self._spi_ctrl.write_bytes(self.REG_PREAMBLE_MSB, preamble_length.to_bytes(2, 'big'))

    """Prints the contents of the SX1278 chip revision register"""
    def print_chip_rev(self):
        response = self.read_reg(self.REG_VERSION)
        print("Silicon revision: 0x%02X" % response)

    """Get interrupts"""
    def get_irq(self):
        return self.read_reg(self.REG_IRQ_FLAGS)

    """Clear interrupts"""
    def clr_irq(self, mask):
        self.write_reg(self.REG_IRQ_FLAGS, mask)

    """Prints interrupt"""
    def print_irq(self):
        response = self.get_irq()
        print("Interrupts (0x%02X):" % response)
        if response == 0x00:
            print("  None")
            return
        if response & (1 << 0):
            print("  CadDetected")
        if response & (1 << 1):
            print("  FhssChangeChannel")
        if response & (1 << 2):
            print("  CadDone")
        if response & (1 << 3):
            print("  TxDone")
        if response & (1 << 4):
            print("  ValidHeader")
        if response & (1 << 5):
            print("  PayloadCrcError")
        if response & (1 << 6):
            print("  RxDone")
        if response & (1 << 7):
            print("  RxTimeout")

    """Get number of payload bytes received"""
    def payload_rx_len(self):
        return self.read_reg(self.REG_RX_NB_BYTES)

    """Get number of valid headers received"""
    def nr_headers_received_xxx(self):
        response = self._spi_ctrl.read_bytes(self.REG_RX_HEADER_CNT_VALUE_MSB, 2)
        return int.from_bytes(response, 'big')

    """Get number of valid packets received"""
    def nr_packets_received_xxx(self):
        response = self._spi_ctrl.read_bytes(self.REG_RX_HEADER_CNT_VALUE_MSB, 2)
        return int.from_bytes(response, 'big')

    """Print status"""
    def print_status(self):
        response = self.read_reg(self.REG_MODEM_STAT)
        print("Modem Status")
        print("  RxCodingRate: %d" % ((response >> 5) & 0x7))
        if response & (1 << 4):
            print("  Modem clear")
        if response & (1 << 3):
            print("  Header info valid")
        if response & (1 << 2):
            print("  RX on-going")
        if response & (1 << 1):
            print("  Signal synchronized")
        if response & (1 << 0):
            print("  Signal detected")

    """Change Mode"""
    def set_mode(self, mode):
        response = self.read_reg(self.REG_OP_MODE) & 0b11111000
        self.write_reg(self.REG_OP_MODE, response | mode)

    """Get current Mode"""
    def get_mode(self):
        response = self.read_reg(self.REG_OP_MODE) & 0b00000111
        return response

    """Get Last Packet SNR (dB)"""
#    def get_last_snr_xxx(self):
#        response = self.read_reg(self.REG_PKT_SNR_VALUE, signed=True)
#        return response / 4.0

    """Get LastPacket RSSI (dBm)"""
#    def get_last_rssi_xxx(self):
#        response = self.read_reg(self.REG_PKT_RSSI_VALUE)
#        return -164 + response

    """Get Current RSSI (dBm)"""
#    def get_current_rssi_xxx(self):
#        response = self.read_reg(self.REG_RSSI_VALUE, signed=True)
#        return -164 + response

    """Print RX Status"""
#    def print_rx_status_xxx(self):
#        print("Headers received: %d" % self.nr_headers_received())
#        print("Packets received: %d" % self.nr_packets_received())

    """Get RX Status"""
    def get_rx_status(self):
        data = self._spi_ctrl.read_bytes(self.REG_RX_HEADER_CNT_VALUE_MSB, 8)
        head_cnt = int.from_bytes(data[0:2], 'big')   # REG_RX_HEADER_CNT_VALUE_xxx
        pkt_cnt = int.from_bytes(data[2:4], 'big')    # REG_RX_PACKET_CNT_VALUE_xxx
        modem_stat = data[4]                          # REG_MODEM_STAT
        pkt_snr = byte_to_int(data[5:6], True) / 4.0  # REG_PKT_SNR_VALUE (signed)
        pkt_rssi = -164 + data[6]                     # REG_PKT_RSSI_VALUE
        rssi = -164 + data[7]                         # REG_RSSI_VALUE
        rx_status = {
            'head_cnt': head_cnt,
            'pkt_cnt': pkt_cnt,
            'modem_stat': modem_stat,
            'pkt_snr': pkt_snr,
            'pkt_rssi': pkt_rssi,
            'rssi': rssi
        }
        return rx_status

    """Received data (bytearray)"""
    def receive_data(self, button):
        if not self._rxcontinous_enabled:
            print("Enable RX Continous mode (was %d)" % self.get_mode())
            self.set_mode(self.MODE_RXCONTINOUS)
            self._rxcontinous_enabled = True

        # print("Waiting for RX_DONE")
        while not self.get_irq() & self.IRQ_RX_DONE:
            if button.value() == 0:
                return bytearray(0), self.STATUS_ABORT
            pass

        # Get fresh copy of the interrupts
        irq = self.get_irq() & (self.IRQ_RX_DONE | self.IRQ_VALID_HEADER | self.IRQ_PAYLOAD_CRC_ERROR)

        # Get payload length
        payload_length = self.payload_rx_len()

        self.clr_irq(irq)  # Clear interrupts

        if not (irq & self.IRQ_VALID_HEADER):
            return bytearray(0), self.STATUS_INVALID_HEADER  # Return empty bytearray and status

        if irq & self.IRQ_PAYLOAD_CRC_ERROR:
            status = self.STATUS_PAYLOAD_CRC_ERROR
        else:
            status = self.STATUS_OK

        # Read data from RX FIFO
        self.write_reg(self.REG_FIFO_ADDR_PTR, self.read_reg(self.REG_FIFO_RX_CURRENT_ADDR))
        data = self.read_fifo(payload_length)

        return data, status

    """Transmit data (bytearray)"""
    def transmit_data(self, data, wait_for_done=False):
        # Wait for Standby Mode
        while not self.get_mode() == self.MODE_STDBY:
            pass

        # Write data to TX FIFO
        self.write_reg(self.REG_FIFO_ADDR_PTR, self.read_reg(self.REG_FIFO_TX_BASE_ADDR))
        self.write_fifo(data)

        # print("Start Transmitting")
        self.write_reg(self.REG_PAYLOAD_LENGTH, len(data))
        self.set_mode(self.MODE_TX)
        if wait_for_done:
            while not self.get_irq() & self.IRQ_TX_DONE:
                pass

            self.clr_irq(self.IRQ_TX_DONE)

        # print("TX_DONE")
