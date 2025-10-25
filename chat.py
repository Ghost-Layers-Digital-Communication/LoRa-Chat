#!/usr/bin/env python3
import tkinter as tk
from tkinter import scrolledtext
import time
import spidev
import RPi.GPIO as GPIO
import config

# ────────────── Packet Counter
packet_counter = 0

# ────────────── SPI + GPIO Init
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(config.RESET_PIN, GPIO.OUT)
GPIO.setup(config.DIO0_PIN, GPIO.IN) # FIXED

spi = spidev.SpiDev()
spi.open(config.SPI_BUS, config.SPI_DEVICE)
spi.max_speed_hz = config.SPI_SPEED_HZ

# Shortcuts from config REG + MODE dicts
REG = config.REG
MODE = config.MODE
IRQ_TX_DONE = 0x08
IRQ_RX_DONE = 0x40
IRQ_PAYLOAD_CRC_ERROR = 0x20

# ────────────── SPI Helpers
def write_reg(addr, val):
    spi.xfer2([addr | 0x80, val & 0xFF])

def read_reg(addr):
    return spi.xfer2([addr & 0x7F, 0x00])[1]

def reset_lora():
    GPIO.output(config.RESET_PIN, 0)
    time.sleep(0.1)
    GPIO.output(config.RESET_PIN, 1)
    time.sleep(0.1)

# ────────────── Radio Init
def init_lora():
    reset_lora()
    write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["SLEEP"])
    time.sleep(0.01)
    write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["STDBY"])
    frf = int(config.LORA_FREQ / (32e6 / (1 << 19)))
    write_reg(REG["FRF_MSB"], (frf >> 16) & 0xFF)
    write_reg(REG["FRF_MID"], (frf >> 8) & 0xFF)
    write_reg(REG["FRF_LSB"], frf & 0xFF)
    # LoRa modem standard config
    write_reg(REG["MODEM_CONFIG1"], 0x72)
    write_reg(REG["MODEM_CONFIG2"], 0x74)
    write_reg(REG["MODEM_CONFIG3"], 0x04)
    # TX power
    write_reg(REG["PAYLOAD_LENGTH"], 0xFF)
    write_reg(0x09, 0x8F if config.LORA_PWR > 20 else 0x80)
    write_reg(REG["FIFO_TX_BASE_ADDR"], 0x00)
    write_reg(REG["FIFO_RX_BASE_ADDR"], 0x00)
    write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["RXCONT"])

# ────────────── TX
def send_packet(data_bytes):
    write_reg(REG["IRQ_FLAGS"], 0xFF)
    write_reg(REG["FIFO_ADDR_PTR"], 0x00)
    for b in data_bytes:
        write_reg(REG["FIFO"], b)
    write_reg(REG["PAYLOAD_LENGTH"], len(data_bytes))
    write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["TX"])
    time.sleep(0.05) # Allow RX module to catch packet
    tx_start = time.time()
    while (read_reg(REG["IRQ_FLAGS"]) & IRQ_TX_DONE) == 0:
        if time.time() - tx_start > (config.TX_TIMEOUT / 1000):
            write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["RXCONT"])
            return False
        time.sleep(0.01)
    write_reg(REG["IRQ_FLAGS"], IRQ_TX_DONE)
    write_reg(REG["OP_MODE"], MODE["LONG_RANGE_MODE"] | MODE["RXCONT"])
    return True

# ────────────── RX
def receive_packet():
    irq = read_reg(REG["IRQ_FLAGS"])
    if irq & IRQ_RX_DONE and not irq & IRQ_PAYLOAD_CRC_ERROR:
        write_reg(REG["IRQ_FLAGS"], IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR)
        addr = read_reg(REG["FIFO_RX_CURRENT_ADDR"])
        write_reg(REG["FIFO_ADDR_PTR"], addr)
        length = read_reg(REG["RX_NB_BYTES"]) # FIXED
        data = bytearray()
        for _ in range(length):
            data.append(read_reg(REG["FIFO"]))
        return data.decode("utf-8", "ignore")
    return None

# ────────────── GUI
class LoRaChatGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("===[Ghost Layers LoRa Chat]=== coded by sacred G")
        self.root.configure(bg="black")
        self.msg_area = scrolledtext.ScrolledText(
            root, wrap=tk.WORD, bg="black", fg="lime", insertbackground="lime"
        )
        self.msg_area.config(state=tk.DISABLED)
        self.msg_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.input_field = tk.Entry(root, bg="black", fg="lime", insertbackground="lime")
        self.input_field.pack(padx=10, pady=(0,5), fill=tk.X)
        self.input_field.bind("<Return>", self.send_message)
        self.send_button = tk.Button(
            root, text="SEND", command=self.send_message,
            bg="green", fg="black", activeforeground="black"
        )
        self.send_button.pack(padx=10, pady=(0,10), fill=tk.X)
        init_lora()
        self.root.after(300, self.check_incoming)

    def send_message(self, event=None):
        global packet_counter
        msg = self.input_field.get().strip().upper()[:config.LORA_MAX_LEN]
        if not msg:
            return
        hexctr = f"{packet_counter & 0xFFFF:04X}"
        timestamp = time.strftime("%y%m%d-%H%M%S")
        payload = f"{config.USER_ID:02d}-{hexctr}|{config.DEVICE_ID}|{timestamp}|{msg}"
        if send_packet(payload.encode("utf-8")):
            self.add_message(f"[{timestamp}] {config.DEVICE_ID}: {msg}")
        packet_counter += 1
        self.input_field.delete(0, tk.END)

    def add_message(self, text):
        self.msg_area.config(state=tk.NORMAL)
        self.msg_area.insert(tk.END, text + "\n")
        self.msg_area.see(tk.END)
        self.msg_area.config(state=tk.DISABLED)

    def check_incoming(self):
        pkt = receive_packet()
        if pkt:
            parts = pkt.split("|", 3)
            if len(parts) == 4:
                _, device, timestamp, msg = parts
                self.add_message(f"[{timestamp}] {device}: {msg}")
            else:
                self.add_message(f"[RX RAW] {pkt}")
        self.root.after(300, self.check_incoming)

    def close(self):
        spi.close()
        GPIO.cleanup()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    gui = LoRaChatGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.close)
    root.mainloop()
