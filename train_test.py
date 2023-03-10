import numpy as np

import shutil

total_num = 4000
ratio = 0.8

train_num = int(total_num * ratio)
test_num = int(total_num - train_num)
print(test_num)

for i in range(train_num):

    cur_path = 'Dataset/lego_yolo/IMG_test%s.png' % (i+1000)
    tar_path = 'Dataset/train_img/img%s.png' % i
    shutil.copy(cur_path, tar_path)

    cur_path = 'Dataset/yolo_label/img%s.txt' % (i + 1000)
    tar_path = 'Dataset/train_label/img%s.txt' % i
    shutil.copy(cur_path, tar_path)

for i in range(train_num, total_num):
    cur_path = 'Dataset/lego_yolo/IMG_test%s.png' % (i+1000)
    tar_path = 'Dataset/test_img/img%s.png' % i
    shutil.copy(cur_path, tar_path)
    cur_path = 'Dataset/yolo_label/img%s.txt' % (i + 1000)
    tar_path = 'Dataset/test_label/img%s.txt' % i
    shutil.copy(cur_path, tar_path)