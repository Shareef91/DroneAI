import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from PIL import ImageTk, Image


import sys
import os
sys.path.append(os.path.dirname(os.path.abspath("weather_data.py")))
from weather_data import WeatherData

import threading

dict1 = {}
dict2 = {}
dict3 = {}

def update_plot(data_val):
    if data_val is not None:
        dict1[data_val.time] = data_val.temp
        dict2[data_val.time] = data_val.humidity
        dict3[data_val.time] = data_val.pressure
    weather_data = [dict1, dict2, dict3]
    
    return weather_data


def main():
    window = tk.Tk()
    window.title("Sensor Data and Image Display")
    window.geometry("700x500")

    # Create a frame to hold both the plot and the image side by side
    frame = tk.Frame(window)
    frame.config(background='lightblue')
    frame.pack(fill=tk.BOTH, expand=1)

    #display the plot
    fig = Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111)
    data = update_plot(None)  # Initial call to set up the plot
    ax.plot(data[0].keys(), data[0].values(), label="Fake Temperature")
    ax.plot(data[1].keys(), data[1].values(), label="Fake Humidity")
    ax.plot(data[2].keys(), data[2].values(), label="Fake Pressure")
    fig.patch.set_facecolor('lightblue')
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    #canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.Y, expand=1)
   
    # Move the plot canvas into the left side of the frame
    canvas.get_tk_widget().pack_forget()  # Remove from previous packing
    canvas.get_tk_widget().pack(in_=frame, side=tk.LEFT, fill=tk.BOTH, expand=1)

    # if button makes sense
    stop_rec_btn = tk.Button(frame, text="Stop Receiving Data", background='red')
    stop_rec_btn.pack(side=tk.LEFT, anchor='s', pady=20)
    # Display the image on the right side of the frame
    compressedImg = Image.open("lora_repo/images/compressed_img_18.jpg")
    expandedImg = compressedImg.resize((compressedImg.width * 3, compressedImg.height * 3))
    img = ImageTk.PhotoImage(expandedImg)
    image_label = tk.Label(frame, image=img, background='lightblue')
    image_label.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    #previous images
    img2 = ImageTk.PhotoImage(compressedImg)
    image_label2 = tk.Label(frame, image=img2, background='lightblue')
    image_label2.pack(side=tk.RIGHT, pady=30, fill=tk.BOTH, expand=1)
    image_label3 = tk.Label(frame, image=img2, background='lightblue')
    image_label3.pack(side=tk.RIGHT, pady=30, fill=tk.BOTH, expand=1)
    window.mainloop()

def test_update():
    w_data1 = WeatherData(1, 10, 7, 15)
    w_data2 = WeatherData(2, 5, 12, 9)
    w_data3 = WeatherData(3, 20, 8, 11)
    w_data4 = WeatherData(4, 15, 18, 6)
    w_data5 = WeatherData(5, 2, 24, 13)
    for data in [w_data1, w_data2, w_data3, w_data4, w_data5]:
        update_plot(data)
    print("Test update complete")

if __name__ == "__main__":
    
    threading.Thread(target=print, args=("Main thread started", )).start()
    threading.Thread(target=test_update).start()
    # threading.Thread(target=update_plot, args=(WeatherData(5, 19, 23, 5), )).start()
    main()
    print("plot updated")