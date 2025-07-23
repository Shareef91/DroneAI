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
dict4 = {}
img_list = [] # ../lora_repo/images/placeholder.png
rec_done = False

def update_plot(data_val):
    if data_val is not None:
        #print("Weather data about to be plotted.")
        dict1[data_val.time] = data_val.temp
        dict2[data_val.time] = data_val.humidity
        dict3[data_val.time] = data_val.pressure
        dict4[data_val.time] = data_val.altitude
    weather_data = [dict1, dict2, dict3, dict4]
    
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

def stop_receiving():
        global rec_done
        rec_done = True

def main(wQueue=None, objQueue=None, imgQueue=None):
    window = tk.Tk()
    window.attributes('-fullscreen', True)
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

    # Add a vertical scrollbar to the Listbox
    object_scrollbar = tk.Scrollbar(object_list_frame, orient=tk.VERTICAL)
    object_listbox = tk.Listbox(
        object_list_frame,
        height=6,
        width=30,
        yscrollcommand=object_scrollbar.set
    )
    object_scrollbar.config(command=object_listbox.yview)
    object_listbox.pack(side=tk.LEFT, anchor='w', padx=10, pady=(0, 10), fill=tk.BOTH, expand=True)
    object_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    detected_objects = {}
    # Exit button in the top right corner
    exit_btn = tk.Button(window, text="Exit", command=window.destroy, background='orange', font=('Arial', 12, 'bold'))
    exit_btn.place(relx=1.0, rely=0.0, anchor='ne', x=-10, y=10)
    # display the plot
    fig = Figure(figsize=(5, 4), dpi=100)
    top_ax = fig.add_subplot(2, 1, 1)
    bot_ax = fig.add_subplot(2, 1, 2)
    top_ax2 = top_ax.twinx()
    bot_ax2 = bot_ax.twinx()

    data = update_plot(None)  # Initial call to set up the plot

    temp_times = [datetime.fromisoformat(str(k)) for k in data[0].keys()]
    hum_times = [datetime.fromisoformat(str(k)) for k in data[1].keys()]
    pres_times = [datetime.fromisoformat(str(k)) for k in data[2].keys()]
    alt_times = [datetime.fromisoformat(str(k)) for k in data[3].keys()]

    temp_plot, = top_ax.plot(temp_times, list(data[0].values()), label="Temperature", color='tab:red')
    hum_plot, = top_ax2.plot(hum_times, list(data[1].values()), label="Humidity", color='tab:blue')
    pres_plot, = bot_ax.plot(pres_times, list(data[2].values()), label="Pressure", color='tab:green')
    alt_plot, = bot_ax2.plot(alt_times, list(data[3].values()), label="Altitude", color='tab:purple')  # Placeholder for altitude

    top_ax.set_ylabel("Temperature (C)")
    top_ax2.set_ylabel("Humidity (%)")
    bot_ax.set_ylabel("Pressure (hPa)")
    bot_ax2.set_ylabel("Altitude (m)")

    # Set axis colors for clarity
    top_ax.yaxis.label.set_color('tab:red')
    top_ax2.yaxis.label.set_color('tab:blue')
    bot_ax.yaxis.label.set_color('tab:green')
    bot_ax2.yaxis.label.set_color('tab:purple')
    top_ax.tick_params(axis='y', colors='tab:red')
    top_ax2.tick_params(axis='y', colors='tab:blue')
    bot_ax.tick_params(axis='y', colors='tab:green')
    bot_ax2.tick_params(axis='y', colors='tab:purple')

    # Set x-axis label
    bot_ax.set_xlabel("Time")

    # Combine legends
    top_lines = [temp_plot, hum_plot]
    bot_lines = [pres_plot, alt_plot]
    top_labels = [l.get_label() for l in top_lines]
    bot_labels = [l.get_label() for l in bot_lines]
    top_ax.legend(top_lines, top_labels, loc='upper left')
    bot_ax.legend(bot_lines, bot_labels, loc='upper left')

    fig.patch.set_facecolor('lightblue')
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.draw()
    canvas.get_tk_widget().pack(in_=frame, side=tk.LEFT, fill=tk.BOTH, expand=1, pady=60)

    # Timer event to update the plot
    def refresh_plot():
        try:
            if not rec_done:
                data_val = None
                if wQueue and not wQueue.empty():
                    data_val = wQueue.get_nowait()
                data = update_plot(data_val)
                # --- Update object type list if new object detected ---
                if objQueue and not objQueue.empty():
                    obj_data = objQueue.get_nowait() # string representing the object type
                    if obj_data and obj_data in detected_objects:
                        detected_objects[obj_data] += 1
                        obj_index = list(detected_objects.keys()).index(obj_data)
                        object_listbox.delete(obj_index)
                        object_listbox.insert(obj_index, f" {obj_data}: ({detected_objects[obj_data]})")
                    if obj_data and obj_data not in detected_objects:
                        detected_objects[obj_data] = 1
                        object_listbox.insert(tk.END, f" {obj_data}: ({detected_objects[obj_data]})")
                        # Autoscroll to the bottom when a new item is added
                        object_listbox.yview_moveto(1.0)
                    
                print("Refreshing plot with data: ", data)
                if data and all(isinstance(d, dict) for d in data):
                    temp_times = [datetime.fromisoformat(str(k)) for k in data[0].keys()]
                    hum_times = [datetime.fromisoformat(str(k)) for k in data[1].keys()]
                    pres_times = [datetime.fromisoformat(str(k)) for k in data[2].keys()]
                    alt_times = [datetime.fromisoformat(str(k)) for k in data[3].keys()]
                    temp_plot.set_data(temp_times, list(data[0].values()))
                    hum_plot.set_data(hum_times, list(data[1].values()))
                    pres_plot.set_data(pres_times, list(data[2].values()))
                    alt_plot.set_data(alt_times, list(data[3].values()))
                    # Autoscale each axis independently
                    top_ax.relim()
                    top_ax.autoscale_view()
                    top_ax2.relim()
                    top_ax2.autoscale_view()
                    bot_ax.relim()
                    bot_ax.autoscale_view()
                    bot_ax2.relim()
                    bot_ax2.autoscale_view()
                    canvas.draw()

                # Image update
                if imgQueue and not imgQueue.empty():
                    img_name = imgQueue.get_nowait()
                    update_img(img_name)
                    display_img = Image.open(img_name)
                    display_img.save("display_image.jpg")  # Save the resized image
                    expandedImg = display_img.resize((500, int(500 * display_img.height / display_img.width)))
                    expandedImg.save("expanded_image.jpg")  # Save the resized image
                    img = ImageTk.PhotoImage(expandedImg)
                    image_label.config(image=img)
                    image_label.image = img
                    object_label.config(text=f"Object Type: {img_name.split('.')[0]}")
                    #previous images
                    for i in range(1, len(img_list)):
                        new_img_label = display_prev_image(tk.Label(right_frame, background='lightblue'), img_list[-i])
                        if i == 1:
                            new_img_label.pack(side=tk.RIGHT, pady=15, fill=tk.BOTH, expand=1)
        except Exception as e:
            print("Error in refresh_plot:", e)
        window.after(2000, refresh_plot)  # Schedule next update in 2000 ms
    # Initial image
    display_img = Image.open("../lora_repo/images/placeholder.png")
    expandedImg = display_img.resize((500, int(500 * display_img.height / display_img.width)))
    img = ImageTk.PhotoImage(expandedImg)
    image_label = tk.Label(right_frame, image=img, background='lightblue')
    object_label = tk.Label(right_frame, text=f"Object Type: Child Drawing", background='lightblue')
    object_label.pack(side=tk.TOP, pady=(70,0))
    image_label.pack(side=tk.TOP, fill=tk.BOTH)#, expand=1
        
    refresh_plot()  # Start the timer

    # if button makes sense
    stop_rec_btn = tk.Button(frame, text="Stop Receiving Data", background='red', command=stop_receiving)
    stop_rec_btn.pack(side=tk.LEFT, anchor='s', pady=20)
    # Display the image on the right side of the frame
    
    
    
    window.mainloop()

def test_update():
    w_data1 = WeatherData(datetime(2025, 4, 20, 15, 30, 0, 123456), 10, 7, 15, 3)
    w_data2 = WeatherData(datetime(2025, 4, 20, 15, 30, 2, 123456), 5, 12, 9, 5)
    w_data3 = WeatherData(datetime(2025, 4, 20, 15, 30, 4, 123456), 20, 8, 11, 7)
    w_data4 = WeatherData(datetime(2025, 4, 20, 15, 30, 6, 123456), 15, 18, 6, 9)
    w_data5 = WeatherData(datetime(2025, 4, 20, 15, 30, 8, 123456), 2, 24, 13, 15)
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