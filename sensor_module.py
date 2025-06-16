import board
import busio
import adafruit_bme280
from datetime import datetime

i2c = busio.I2C(board.SCL, board.SDA)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

def get_sensor_data():
    return {
        "sensor_id": "PiSensor001",
        "datetime": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "temperature_C": round(bme280.temperature, 2),
        "humidity_percent": round(bme280.humidity, 2),
        "pressure_hPa": round(bme280.pressure, 2)
    }