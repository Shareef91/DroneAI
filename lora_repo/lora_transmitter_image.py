import serial
import time
import base64
import os

# LoRa serial setup (adjust port as needed)
lora = serial.Serial(port='/dev/ttyS0', baudrate=9600, parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)

# Read and encode image
with open('images/img_18.jpg', 'rb') as img_file:
    img_data = img_file.read()
b64_data = base64.b64encode(img_data).decode('utf-8')

# Packetize
PACKET_SIZE = 200  # bytes per packet (adjust as needed)
packets = [b64_data[i:i+PACKET_SIZE] for i in range(0, len(b64_data), PACKET_SIZE)]
total_packets = len(packets)

print(f"Sending {total_packets} packets...")

for idx, packet in enumerate(packets):
    header = f"{idx+1}/{total_packets}:"
    to_send = header + packet + "\n"
    lora.write(to_send.encode('utf-8'))
    # Wait for ACK
    ack = lora.readline().decode('utf-8').strip()
    while ack != f"ACK {idx+1}":
        print(f"No ACK for packet {idx+1}, resending...")
        lora.write(to_send.encode('utf-8'))
        ack = lora.readline().decode('utf-8').strip()
    print(f"Packet {idx+1}/{total_packets} sent and acknowledged.")
    time.sleep(0.5)  # Small delay to avoid overwhelming receiver

print("Image transmission complete.")