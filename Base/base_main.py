import threading

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath("weather_data.py")))
from weather_data import WeatherData

from station_GUI import main, update_plot, test_update

if __name__ == "__main__":
    threading.Thread(target=test_update).start()
    threading.Thread(target=update_plot, args=(WeatherData(6, 3, 5, 22),)).start()
    main()