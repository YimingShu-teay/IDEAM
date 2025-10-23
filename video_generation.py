import cv2
import os

# 定义图片文件夹路径
image_folder = r'C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\figsave' 
video_name = r'C:\Users\sym02\Desktop\Research\Extension\codes\decision_change_rear\figsave\output_video_jiansu2.mp4'

# 获取文件夹中所有图片文件名并按顺序排序
images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
images.sort(key=lambda x: int(os.path.splitext(x)[0]))

# 读取第一张图片以获取帧的宽度和高度
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

# 定义视频编解码器和输出视频对象
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter(video_name, fourcc, 20, (width, height))

# 遍历所有图片并将其写入视频
for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

# 释放视频对象
video.release()

print("Video has been successfully created as", video_name)


