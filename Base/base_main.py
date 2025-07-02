import threading
import queue

from station_GUI import main, update_plot, test_update, update_img
from Lora_Rec import LoRaReceiver, WeatherData

def plot_data(data_queue):
    while data_queue.empty() is False:
        update_plot(data_queue.get())

def display_img(img_queue):
    while img_queue.empty() is False:
        img_data = img_queue.get()
        if isinstance(img_data, str):
            update_img(img_data)

if __name__ == "__main__":
    # test threads
    # threading.Thread(target=test_update).start() # use plot_data() rather than test_update()
    # threading.Thread(target=update_plot, args=(WeatherData(6, 3, 5, 22),)).start()
    
    base_receiver = LoRaReceiver()
    threading.Thread(target=base_receiver.receiver).start()
    threading.Thread(target=plot_data, args=(base_receiver.wQueue,)).start()
    threading.Thread(target=display_img, args=(base_receiver.imgQueue,)).start()

    main()

    # weather packet -  W:(int here)
    # fraction of total image packet
    # image name packet - just the name; like object type  = should come first once Jack does that