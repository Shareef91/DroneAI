import threading
import queue
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath("weather_data.py")))
from weather_data import WeatherData

from station_GUI import main, update_plot, test_update

def plot_data(data_queue):
    while data_queue.empty() is False:
        update_plot(data_queue.get())

if __name__ == "__main__":
    threading.Thread(target=test_update).start() # use plot_data() rather than test_update()
    threading.Thread(target=update_plot, args=(WeatherData(6, 3, 5, 22),)).start()
    main()

    # weather packet -  W:(int here)
    # fraction of total image packet
    # image name packet - just the name; like object type  = should come first once Jack does that