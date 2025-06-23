from PIL import Image

# Needs Pillow installed: pip install Pillow

img = Image.open('images/img_18.jpg')
img = img.resize((img.width // 4, img.height // 4))
img = img.crop((0, 0, img.width // 2, img.height // 2))
img.save('images/compressed_img_18.jpg', format='JPEG', quality=72)

print("Done")