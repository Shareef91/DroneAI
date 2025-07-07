import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from PIL import ImageTk, Image

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath("../weather_data.py")))
from weather_data import WeatherData

import threading

dict1 = {}
dict2 = {}
dict3 = {}
img_list = ["lora_repo/images/placeholder.png"] # ../lora_repo/images/placeholder.png

def update_plot(data_val):
    if data_val is not None:
        print("Weather data about to be plotted.")
        dict1[data_val.time] = data_val.temp
        dict2[data_val.time] = data_val.humidity
        dict3[data_val.time] = data_val.pressure
    weather_data = [dict1, dict2, dict3]
    
    return weather_data

def update_img(img_addr):
    global img_list
    img_list.insert(0, img_addr)

def display_prev_image(label, img):
    previous_img = Image.open(img)
    previous_img = previous_img.resize((150, int(150 * previous_img.height / previous_img.width)))
    ready_img = ImageTk.PhotoImage(previous_img)
    label.config(image=ready_img)
    label.image = ready_img  # Keep a reference to avoid garbage collection
    return label


def main(wQueue=None):
    window = tk.Tk()
    window.title("Sensor Data and Image Display")
    window.geometry("700x500")

    # Create a frame to hold both the plot and the image side by side
    frame = tk.Frame(window)
    frame.config(background='lightblue')
    frame.pack(fill=tk.BOTH, expand=1)

    # this frame holds all the images   
    right_frame = tk.Frame(frame, width=600, background='lightblue')
    right_frame.pack(side=tk.RIGHT, fill=tk.Y)
    right_frame.pack_propagate(False)

    # display the plot
    fig = Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111)
    data = update_plot(None)  # Initial call to set up the plot
    print("Plotting data: ", data)
    temp_plot, = ax.plot(data[0].keys(), data[0].values(), label="Fake Temperature")
    hum_plot, = ax.plot(data[1].keys(), data[1].values(), label="Fake Humidity")
    pres_plot, = ax.plot(data[2].keys(), data[2].values(), label="Fake Pressure")
    ax.legend()
    fig.patch.set_facecolor('lightblue')
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack(in_=frame, side=tk.LEFT, fill=tk.BOTH, expand=1, pady=60)

    # Timer event to update the plot
    def refresh_plot():
        try:
            # Safely get new data from the queue if available
            data_val = None
            if wQueue and not wQueue.empty():
                data_val = wQueue.get_nowait()
            data = update_plot(data_val)
            print("Refreshing plot with data: ", data)
            # Ensure data is a list of dicts and not empty
            if data and all(isinstance(d, dict) for d in data):
                temp_plot.set_data(list(data[0].keys()), list(data[0].values()))
                hum_plot.set_data(list(data[1].keys()), list(data[1].values()))
                pres_plot.set_data(list(data[2].keys()), list(data[2].values()))
                # Rescale axes
                ax.relim()
                ax.autoscale_view()
                canvas.draw()
        except Exception as e:
            print("Error in refresh_plot:", e)
        window.after(2000, refresh_plot)  # Schedule next update in 2000 ms

    refresh_plot()  # Start the timer

    # if button makes sense
    stop_rec_btn = tk.Button(frame, text="Stop Receiving Data", background='red')
    stop_rec_btn.pack(side=tk.LEFT, anchor='s', pady=20)
    # Display the image on the right side of the frame
    display_img = Image.open(img_list[0])
    expandedImg = display_img.resize((500, int(500 * display_img.height / display_img.width)))
    img = ImageTk.PhotoImage(expandedImg)
    image_label = tk.Label(right_frame, image=img, background='lightblue')
    object_label = tk.Label(right_frame, text="Object Type: Unknown", background='lightblue')
    object_label.pack(side=tk.TOP, pady=(70,0))
    image_label.pack(side=tk.TOP, fill=tk.BOTH)#, expand=1
    
    #previous images
    for i in range(1, len(img_list)):
        new_img_label = display_prev_image(tk.Label(right_frame, background='lightblue'), img_list[-i])
        new_img_label.pack(side=tk.RIGHT, pady=15, fill=tk.BOTH, expand=1)
    window.mainloop()

def test_update():
    w_data1 = WeatherData(1, 10, 7, 15)
    w_data2 = WeatherData(2, 5, 12, 9)
    w_data3 = WeatherData(3, 20, 8, 11)
    w_data4 = WeatherData(4, 15, 18, 6)
    w_data5 = WeatherData(5, 2, 24, 13)
    for data in [w_data1, w_data2, w_data3, w_data4, w_data5]:
        update_plot(data)
    img_l = ["lora_repo/images/img.png", "lora_repo/images/img2.png", "lora_repo/images/img3.png", "lora_repo/images/compressed_img_18.jpg"]
    for i in img_l:
        update_img(i)
    print("Test update complete")

if __name__ == "__main__":
    threading.Thread(target=print, args=("Main thread started", )).start()
    threading.Thread(target=test_update).start()
    main()