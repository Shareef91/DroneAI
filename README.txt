Setup Instructions:
1. Install dependencies:
   pip install opencv-python adafruit-circuitpython-bme280

2. Download MobileNet SSD model files:
   mkdir -p ~/models/mobilenet_ssd
   cd ~/models/mobilenet_ssd
   wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.prototxt
   wget https://github.com/chuanqi305/MobileNet-SSD/releases/download/v1.0/MobileNetSSD_deploy.caffemodel

3. Run the project:
   python main.py