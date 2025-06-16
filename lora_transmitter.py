from SX127x.LoRa import LoRa
from SX127x.board_config import BOARD
from SX127x.constants import MODE
import json

BOARD.setup()

class LoRaSender(LoRa):
    def __init__(self):
        super(LoRaSender, self).__init__()
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([1, 0, 0, 0])

    def send(self, message):
        self.write_payload([ord(c) for c in message])
        self.set_mode(MODE.TX)
        while self.get_mode() == MODE.TX:
            pass

lora = LoRaSender()
lora.set_mode(MODE.STDBY)

def send_data(payload):
    msg = json.dumps(payload)
    print("Sending via LoRa:", msg)
    lora.send(msg)