import time
import RPi.GPIO as GPIO
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import config # uses your existing config.py

BOARD.setup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class LoRaChat(LoRa):
    def __init__(self):
        super(LoRaChat, self).__init__(verbose=False)
        self.set_mode(MODE.STDBY)
        self.set_freq(config.FREQUENCY)
        self.set_pa_config(pa_select=1, max_power=21, output_power=config.TX_POWER)
        self.set_spreading_factor(config.SPREADING_FACTOR)
        self.set_bandwidth(config.BANDWIDTH)
        self.set_coding_rate(config.CODING_RATE)
        self.set_rx_crc(True)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        print(f"[READY] LoRa chat started at {config.FREQUENCY} MHz")

    def send_message(self, message):
        self.set_mode(MODE.STDBY)
        self.write_payload(list(bytearray(message, "utf-8")))
        self.set_mode(MODE.TX)
        tx_start = time.time()
        print(f"[TX] Sending: {message}")

        # Wait for TX done or timeout
        while GPIO.input(config.DIO0_PIN) == 0:
            if time.time() - tx_start > 5: # 5-second timeout
                print("[ERROR] TX timed out — check wiring or DIO0 connection")
                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)
                return
            time.sleep(0.01)

        self.clear_irq_flags(TxDone=1)
        print("[OK] TX complete.")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = bytes(self.read_payload(nocheck=True)).decode("utf-8", "ignore")
        print(f"[RX] Received: {payload}")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

try:
    lora = LoRaChat()
    while True:
        msg = input("Enter message: ")[:250]
        if msg.lower() in ("exit", "quit"):
            break
        lora.send_message(msg)
        time.sleep(0.5)

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    BOARD.teardown()
    print("[EXIT] Chat closed.")