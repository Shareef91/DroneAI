from PIL import Image

# Needs Pillow installed: pip install Pillow

img = Image.open('lora_repo/images/img_18.jpg')

img = img.crop((0, 0, img.width // 2, img.height // 2))
img = img.resize((img.width // 4, img.height // 4))
# 112 quality for farther objects or resize it less
img.save('lora_repo/images/compressed_img_18.jpg', format='JPEG', quality=72) 


print("Done")