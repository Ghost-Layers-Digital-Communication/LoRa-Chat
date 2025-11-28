# ─────────────────────────────────────────────
# LoRa RFM95 Configuration File (Ghost Layers)
# ─────────────────────────────────────────────
# Black terminal with green text theme
# For use with RPi-based TX/RX LoRa Chat
# ─────────────────────────────────────────────

# ────────────────
# Hardware Pins
# ────────────────
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 5000000

# GPIO mapping (BCM mode)
NSS_PIN = 8 # Chip select (CE0)
RESET_PIN = 17 # Reset pin
DIO0_PIN = 25 # Interrupt pin for RXDONE

# ────────────────
# LoRa Radio Settings
# ────────────────
LORA_FREQ = 915.0e6 # Frequency (MHz)
LORA_BW = 125e3 # Bandwidth
LORA_SF = 7 # Spreading factor
LORA_CR = 5 # Coding rate (4/5)
LORA_SYNC = 0x12 # LoRa sync word
LORA_PWR = 14 # TX power (dBm)
LORA_MAX_LEN = 250 # Max message length
TX_TIMEOUT = 2000 # ms
RX_TIMEOUT = 3000 # ms

# ────────────────
# Register Map (SX1276)
# ────────────────
REG = {
    "FIFO": 0x00,
    "OP_MODE": 0x01,
    "FRF_MSB": 0x06,
    "FRF_MID": 0x07,
    "FRF_LSB": 0x08,
    "FIFO_ADDR_PTR": 0x0D,
    "FIFO_TX_BASE_ADDR": 0x0E,
    "FIFO_RX_BASE_ADDR": 0x0F,
    "FIFO_RX_CURRENT_ADDR": 0x10,
    "IRQ_FLAGS": 0x12,
    "RX_NB_BYTES": 0x13,
    "PKT_SNR_VALUE": 0x19,
    "PKT_RSSI_VALUE": 0x1A,
    "MODEM_CONFIG1": 0x1D,
    "MODEM_CONFIG2": 0x1E,
    "PAYLOAD_LENGTH": 0x22,
    "MODEM_CONFIG3": 0x26,
    "IRQ_FLAGS_MASK": 0x11,
    "DIO_MAPPING_1": 0x40,
    "VERSION": 0x42
}

# ────────────────
# Operation Modes
# ────────────────
MODE = {
    "LONG_RANGE_MODE": 0x80,
    "SLEEP": 0x00,
    "STDBY": 0x01,
    "TX": 0x03,
    "RXCONT": 0x05,
    "RXSINGLE": 0x06
}

# ────────────────
# Device ID & Debug
# ────────────────
DEVICE_ID = "ghost_pi_01" # Change to ghost_pi_02 for other Pi
USER_ID = 1
DEBUG = True