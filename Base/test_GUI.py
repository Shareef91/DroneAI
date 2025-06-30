import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from PIL import ImageTk, Image

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
ax.plot([1, 2, 3, 4], [10, 5, 20, 15], label="Fake Temperature")
ax.plot([1, 2, 3, 4], [7, 12, 8, 18], label="Fake Humidity")
ax.plot([1, 2, 3, 4], [15, 9, 11, 6], label="Fake Pressure")
fig.patch.set_facecolor('lightblue')
canvas = FigureCanvasTkAgg(fig, master=window)
canvas.draw()
#canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.Y, expand=1)
# if button makes sense
stop_rec_btn = tk.Button(frame, text="Stop Receiving Data", background='red')
stop_rec_btn.pack(side=tk.LEFT, anchor='s', pady=20)

# Move the plot canvas into the left side of the frame
canvas.get_tk_widget().pack_forget()  # Remove from previous packing
canvas.get_tk_widget().pack(in_=frame, side=tk.LEFT, fill=tk.BOTH, expand=1)

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