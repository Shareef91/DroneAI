import serial
import time
import base64

# LoRa serial setup (adjust port as needed)
lora = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=2)

received_packets = {}
expected_total = None

print("Waiting for image packets...")

while True:
    data_read = lora.readline()
    if not data_read:
        continue
    try:
        data = data_read.decode("utf-8").strip()
        if ':' not in data or '/' not in data:
            continue
        header, packet = data.split(':', 1)
        idx, total = header.split('/')
        idx = int(idx)
        total = int(total)
        if expected_total is None:
            expected_total = total
        received_packets[idx] = packet
        # Send ACK
        ack_msg = f"ACK {idx}\n"
        lora.write(ack_msg.encode('utf-8'))
        print(f"Received packet {idx}/{total}")
        # Check if all packets received
        if len(received_packets) == expected_total:
            print("All packets received. Reconstructing image...")
            break
    except Exception as e:
        print(f"Error: {e}")
        continue

# Reassemble and decode
b64_data = ''.join([received_packets[i] for i in range(1, expected_total+1)])
img_data = base64.b64decode(b64_data)

# Save image
with open('received_img_18.jpg', 'wb') as img_file:
    img_file.write(img_data)

print("Image saved as received_img_18.jpg")
