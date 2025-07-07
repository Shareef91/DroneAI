import threading
from queue import Queue

from station_GUI import main, update_plot, test_update, update_img
from Lora_Rec import LoRaReceiver

def plot_data(data_queue, done):
    while not done:
        while data_queue.empty() is False:
            w_data = data_queue.get()  # Clear the queue
            print("Test data: " + str(w_data))  # probably right
            print("Data type: ", type(w_data))  # it was correct
            print("Queue size: " + str(data_queue.qsize()))  # it was 0 each time
            update_plot(w_data)

def display_img(img_queue, done):
    while not done:
        while img_queue.empty() is False:
            img_data = img_queue.get()
            if isinstance(img_data, str):
                update_img(img_data)

if __name__ == "__main__":
    # test threads
    # threading.Thread(target=test_update).start() # use plot_data() rather than test_update()
    # threading.Thread(target=update_plot, args=(WeatherData(6, 3, 5, 22),)).start()
    done = False
    base_receiver = LoRaReceiver()
    threading.Thread(target=base_receiver.receiver).start()
    #threading.Thread(target=plot_data, args=(base_receiver.wQueue, done)).start()
    threading.Thread(target=display_img, args=(base_receiver.imgQueue, done)).start()

    main(base_receiver.wQueue)
    # weather packet -  W:(int here)
    # fraction of total image packet
    # image name packet - just the name; like object type  = should come first once Jack does that