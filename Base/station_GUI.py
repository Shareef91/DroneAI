import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MaxNLocator
import matplotlib.pyplot as plt
from PIL import ImageTk, Image
from datetime import datetime

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath("../weather_data.py")))
from weather_data import WeatherData

import threading

dict1 = {}
dict2 = {}
dict3 = {}
img_list = ["../lora_repo/images/placeholder.png"] # ../lora_repo/images/placeholder.png

def update_plot(data_val):
    if data_val is not None:
        #print("Weather data about to be plotted.")
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


def main(wQueue=None, objQueue=None, imgQueue=None):
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

    # --- Add a frame and Listbox for object types ---
    object_list_frame = tk.Frame(right_frame, background='lightblue')
    object_list_frame.pack(side=tk.TOP, fill=tk.X, pady=(10, 0))
    tk.Label(object_list_frame, text="Detected Object Types:", background='lightblue', font=('Arial', 12, 'bold')).pack(anchor='w')
    object_listbox = tk.Listbox(object_list_frame, height=6, width=30)
    object_listbox.pack(anchor='w', padx=10, pady=(0, 10))
    detected_objects = set()

    # display the plot
    fig = Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111)
    ax2 = ax.twinx()  # Second y-axis
    ax3 = ax.twinx()  # Third y-axis

    # Offset the third axis to the right
    ax3.spines["right"].set_position(("axes", 1.15))
    ax3.spines["right"].set_visible(True)

    data = update_plot(None)  # Initial call to set up the plot

    temp_times = [datetime.fromisoformat(str(k)) for k in data[0].keys()]
    hum_times = [datetime.fromisoformat(str(k)) for k in data[1].keys()]
    pres_times = [datetime.fromisoformat(str(k)) for k in data[2].keys()]

    temp_plot, = ax.plot(temp_times, list(data[0].values()), label="Temperature", color='tab:red')
    hum_plot, = ax2.plot(hum_times, list(data[1].values()), label="Humidity", color='tab:blue')
    pres_plot, = ax3.plot(pres_times, list(data[2].values()), label="Pressure", color='tab:green')

    ax.set_ylabel("Temperature")
    ax2.set_ylabel("Humidity")
    ax3.set_ylabel("Pressure")

    # Set axis colors for clarity
    ax.yaxis.label.set_color('tab:red')
    ax2.yaxis.label.set_color('tab:blue')
    ax3.yaxis.label.set_color('tab:green')
    ax.tick_params(axis='y', colors='tab:red')
    ax2.tick_params(axis='y', colors='tab:blue')
    ax3.tick_params(axis='y', colors='tab:green')

    # Set x-axis label
    ax.set_xlabel("Time")

    # Combine legends
    lines = [temp_plot, hum_plot, pres_plot]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='upper left')

    fig.patch.set_facecolor('lightblue')
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack(in_=frame, side=tk.LEFT, fill=tk.BOTH, expand=1, pady=60)

    # Timer event to update the plot
    def refresh_plot():
        try:
            data_val = None
            if wQueue and not wQueue.empty():
                data_val = wQueue.get_nowait()
            data = update_plot(data_val)
            # --- Update object type list if new object detected ---
            if objQueue and not objQueue.empty():
                obj_data = objQueue.get_nowait() # string representing the object type
                if obj_data and obj_data not in detected_objects:
                    detected_objects.add(obj_data)
                    object_listbox.insert(tk.END, obj_data)
            print("Refreshing plot with data: ", data)
            if data and all(isinstance(d, dict) for d in data):
                temp_times = [datetime.fromisoformat(str(k)) for k in data[0].keys()]
                hum_times = [datetime.fromisoformat(str(k)) for k in data[1].keys()]
                pres_times = [datetime.fromisoformat(str(k)) for k in data[2].keys()]
                temp_plot.set_data(temp_times, list(data[0].values()))
                hum_plot.set_data(hum_times, list(data[1].values()))
                pres_plot.set_data(pres_times, list(data[2].values()))
                # Autoscale each axis independently
                ax.relim()
                ax.autoscale_view()
                ax2.relim()
                ax2.autoscale_view()
                ax3.relim()
                ax3.autoscale_view()
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