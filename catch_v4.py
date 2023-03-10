import time
import pybullet as p
import pybullet_data as pd
import os
import gym
import cv2
import numpy as np
import random
import math
from PIL import Image
from knolling_env3 import *

def sort_cube(obj_list, x_sorted):
    s = np.array(x_sorted)
    sort_index = np.argsort(s)
    # print(sort_index)
    sorted_sas_list = [obj_list[j] for j in sort_index]
    return sorted_sas_list


def my_plot_one_box(x, img, my_yaw, my_length, my_width, color=None, label1='1', line_thickness=3):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    # cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label1:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label1, 0, fontScale=tl / 3, thickness=tf)[0]
        # c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3

        # cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        # my boundary
        gamma = my_yaw

        rot_z = [[np.cos(gamma), -np.sin(gamma)],
                 [np.sin(gamma), np.cos(gamma)]]

        yy = (x[2]+x[0])/2
        xx = (x[3]+x[1])/2

        rot_z = np.asarray(rot_z)

        c11 = [my_length / 2, my_width / 2]
        c22 = [my_length / 2, -my_width / 2]
        c33 = [-my_length / 2, my_width / 2]
        c44 = [-my_length / 2, -my_width / 2]

        mm2px = 1 / 0.000625
        c11, c22, c33, c44 = np.asarray(c11), np.asarray(c22), np.asarray(c33), np.asarray(c44)
        c11 = c11 * mm2px
        c22 = c22 * mm2px
        c33 = c33 * mm2px
        c44 = c44 * mm2px

        corn1 = np.dot(rot_z, c11)
        corn2 = np.dot(rot_z, c22)
        corn3 = np.dot(rot_z, c33)
        corn4 = np.dot(rot_z, c44)

        corn1 = [corn1[0] + xx, corn1[1] + yy]
        corn2 = [corn2[0] + xx, corn2[1] + yy]
        corn3 = [corn3[0] + xx, corn3[1] + yy]
        corn4 = [corn4[0] + xx, corn4[1] + yy]

        cv2.line(img, (int(corn1[1]), int(corn1[0])), (int(corn2[1]), int(corn2[0])), (0, 0, 0), 2)
        cv2.line(img, (int(corn2[1]), int(corn2[0])), (int(corn4[1]), int(corn4[0])), (0, 0, 0), 2)
        cv2.line(img, (int(corn4[1]), int(corn4[0])), (int(corn3[1]), int(corn3[0])), (0, 0, 0), 2)
        cv2.line(img, (int(corn3[1]), int(corn3[0])), (int(corn1[1]), int(corn1[0])), (0, 0, 0), 2)

        # cv2.putText(img, label1, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        # cv2.putText(img, label2, (c1[0], c1[1] - 20), 0, tl / 3, color, thickness=tf, lineType=cv2.LINE_AA)
        # cv2.putText(img, label3, (c1[0], c1[1] - 40), 0, tl / 3, color, thickness=tf,
        #             lineType=cv2.LINE_AA)


def yolo_box(img, label):
    # label = [0,x,y,l,w],[0,x,y,l,w],...
    # label = label[:,1:]
    for i in range(len(label)):
        # label = label[i]
        # print('1',label)
        x_lt = int(label[i][1] * 640 - label[i][3] * 640/2)
        y_lt = int(label[i][2] * 640 - label[i][4] * 640/2)

        x_rb = int(label[i][1] * 640 + label[i][3] * 640/2)
        y_rb = int(label[i][2] * 640 + label[i][4] * 640/2)

        # img = img/255
        img = cv2.rectangle(img,(x_lt,y_lt),(x_rb,y_rb), color = (0,0,0), thickness = 1)
    # cv2.imshow('x',img)
    # cv2.waitKey(0)

    return img

if __name__ == '__main__':


    # p.connect(1)
    p.connect(p.DIRECT)

    startnum = 1000
    endnum = 5000
    lebal_list = []
    reset_flag = True
    num_reset = True

    count_item = 0
    mm2px = 1 / 0.000625
    for epoch in range(startnum,endnum):
        # num_item = random.randint(1, 5)
        num_item = 15
        # path = "urdf/box/"
        # num_item = 4

        if epoch % 1 == 0:
            # count_item = 0
            reset_flag = True

        if reset_flag == True:
            # num_item = random.randint(1, 5)
            # num_item = 3
            env = Arm_env(max_step=1, is_render=False, num_objects=num_item)
            if random.random() < 0.5:
                state, rdm_pos_x, rdm_pos_y, rdm_pos_z, rdm_ori_yaw, lucky_list = env.reset_close()
            else:
                state, rdm_pos_x, rdm_pos_y, rdm_pos_z, rdm_ori_yaw, lucky_list = env.reset()

        # time.sleep(10)
        # if reset_flag != True:
        #
        #     while True:
        #         rdm_pos_x = np.random.uniform(env.x_low_obs * 2.2, env.x_high_obs, size=3)
        #         rdm_pos_y = np.random.uniform(env.y_low_obs, env.y_high_obs, size=3)
        #
        #         dis1_2 = math.dist([rdm_pos_x[0], rdm_pos_y[0]], [rdm_pos_x[1], rdm_pos_y[1]])
        #         dis1_3 = math.dist([rdm_pos_x[0], rdm_pos_y[0]], [rdm_pos_x[2], rdm_pos_y[2]])
        #         dis2_3 = math.dist([rdm_pos_x[1], rdm_pos_y[1]], [rdm_pos_x[2], rdm_pos_y[2]])
        #
        #         if dis1_2 > 0.03 and dis1_3 > 0.03 and dis2_3 > 0.03:
        #             break
        #
        #     rdm_pos_z = 0.01
        #     rdm_ori_yaw = np.random.uniform(0, math.pi / 2, size=3)
        #
        #
        #
        #     for i in range(num_item):
        #         # print("num",num_item)
        #         # print(len(env.obj_idx))
        #         # p.removeBody(env.obj_idx[i])
        #
        #         p.resetBasePositionAndOrientation(env.obj_idx[i], posObj=[rdm_pos_x[i],rdm_pos_y[i],0.01],
        #                                    ornObj=p.getQuaternionFromEuler([0,0,rdm_ori_yaw[i]]))
        #
        #     state = env.get_obs()

        # self.obs = np.concatenate([self.ee_pos, self.ee_ori, self.box_pos, self.box_ori, self.joints_angle])
        #
        #                                  3             3          3*N           3*N            7
        # reset_flag = False
        label = []
        # print(state)
        all_pos = state[6:6+3*num_item]
        all_ori = state[6+3*num_item: 6+6*num_item]
        # print(all_pos)
        # print(all_ori)


        corner_list = []
        for j in range(num_item):
            if j >= num_item:
                element = np.zeros(7)
                # element = np.append(element, 0)
                label.append(element)
            else:
                xpos1 = all_pos[0+3*j]
                ypos1 = all_pos[1+3*j]
                yawori = all_ori[2+3*j]

                xpos, ypos = xyz_resolve(xpos1,ypos1)

                corn1, corn2, corn3, corn4 = find_corner(xpos, ypos, lucky_list[j], yawori)

                corn1, corn2, corn3, corn4 = resolve_img(corn1, corn2, corn3, corn4)

                corner_list.append([corn1, corn2, corn3, corn4])

                if lucky_list[j] == 2:
                    l = 16/1000
                    w = 16/1000
                if lucky_list[j] == 3:
                    l = 24/1000
                    w = 16/1000
                if lucky_list[j] == 4:
                    l = 32/1000
                    w = 16/1000

                corns = corner_list[j]

                col_offset = 320
                row_offset = (0.16 - (0.3112 - 0.16)) * mm2px - 14

                col_list = [int(mm2px * corns[0][1] + col_offset), int(mm2px * corns[3][1] + col_offset),
                            int(mm2px * corns[1][1] + col_offset), int(mm2px * corns[2][1] + col_offset)]
                row_list = [int(mm2px * corns[0][0] - row_offset), int(mm2px * corns[3][0] - row_offset),
                            int(mm2px * corns[1][0] - row_offset), int(mm2px * corns[2][0] - row_offset)]

                col_list = np.sort(col_list)
                row_list = np.sort(row_list)
                col_list[3] = col_list[3] + 3
                col_list[0] = col_list[0] - 3

                row_list[3] = row_list[3] + 3
                row_list[0] = row_list[0] - 3

                label_x = ((col_list[0] + col_list[3]) / 2)/640
                label_y = (((row_list[0] + row_list[3]) / 2)+80)/640

                length = (col_list[3] - col_list[0])/640
                width = (row_list[3] - row_list[0])/640

                # if lucky_list[j] == 2 and rdm_ori_yaw[j] < 0:
                #     rdm_ori_yaw[j] = rdm_ori_yaw[j] + np.pi/2


                element = []
                # element.append(xpos1)
                # element.append(ypos1)
                # # element.append(rdm_pos_z)
                # element.append(yawori)
                # element.append(l)
                # element.append(w)

                ########################################
                element.append(0)
                element.append(label_x)
                element.append(label_y)
                element.append(length)
                element.append(width)
                element = np.asarray(element)
                label.append(element)
                ###############################
                # # element.append(0)
                # element.append(rdm_pos_x[j])
                # element.append(rdm_pos_y[j])
                # # element.append(length)
                # # element.append(width)
                # element = np.asarray(element)
                # label.append(element)

        # x_compare = [label[0][0], label[1][0], label[2][0]]
        # idx_cmp = np.argsort(x_compare)
        # print(idx_cmp)
        # print(x_compare)
        # print(x_compare)
        # label2 = sort_cube(label, x_compare)
        # print(label2)

        # obj_list = np.concatenate([label2[0], label2[1], label2[2]])

        # print(label)
        np.savetxt("Dataset/yolo_label/img%s.txt" %epoch, label,fmt='%.8s')


        # np.savetxt("Dataset/obj_16_label/img%s.txt" % epoch, label, fmt='%.10s')

        # for nnn in range(1):
        lebal_list.append(element)
        my_im2 = env.get_image()
        # print(my_im2.shape)
        add = int((640 - 480) / 2)
        img = cv2.copyMakeBorder(my_im2, add, add, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0, 255))
        # print(img.shape)
        # img = yolo_box(img,label)
        cv2.imwrite("Dataset/lego_yolo/" + "IMG_test%s.png" % epoch, img)
        '''
        # time.sleep(1)
        # for _ in range(20000):
        #     p.stepSimulation()
        #     time.sleep(1/240)
        # num = startnum // 10000
        # my_im2.save("Dataset/lego_yolo/" + "IMG%s.png" % epoch)


        cv2.imwrite("Dataset/lego_yolo/" + "IMG%s.png" % epoch, img)
        
            px_resy1 = row_list[0]
            px_resy2 = row_list[3]
            px_resx1 = col_list[0]
            px_resx2 = col_list[3]

            px_padtop = px_resy1 - 13
            px_padbot = px_resy2 + 13
            px_padleft = px_resx1 - 13
            px_padright = px_resx2 + 13

            if px_padtop < 0:
                px_padtop = 0

            if px_padbot > 640:
                px_padbot = 640

            if px_padleft < 0:
                px_padleft = 0

            if px_padright > 640:
                px_padright = 640

            my_im2 = my_im2[px_padtop:px_padbot, px_padleft:px_padright, :]
            im2_y = my_im2.shape[0]
            im2_x = my_im2.shape[1]
            # print(my_im2.shape[0])

            pad_top = int((96 - im2_y) / 2)
            pad_bot = (96 - im2_y) - pad_top

            pad_left = int((96 - im2_x) / 2)
            pad_right = (96 - im2_x) - pad_left

            img = cv2.copyMakeBorder(my_im2, pad_top, pad_bot, pad_left, pad_right, cv2.BORDER_CONSTANT,
                                     value=(0, 0, 0))


            # my_plot_one_box([0,0,96,96], img, yawori , l ,w)


            # cv2.imwrite('Dataset/yolo_test3/img%s.png'%count_item, img)
            count_item += 1

        '''
    #     for k in range(num_item):
    #
    #         corns = corner_list[k]
    #         corns = np.asarray(corns)
    #         # print(corns.shape)
    #         col_offset = 320
    #         # row_offset = (0.16 - (0.3112 - 0.16)) * mm2px + 2 - 80
    #         row_offset = -80
    #         print(row_offset)
    #
    #         col_list = [int(mm2px*corns[0][1]+col_offset), int(mm2px*corns[3][1]+col_offset), int(mm2px*corns[1][1]+col_offset), int(mm2px*corns[2][1]+col_offset)]
    #         row_list = [int(mm2px*corns[0][0]-row_offset), int(mm2px*corns[3][0]-row_offset), int(mm2px*corns[1][0]-row_offset), int(mm2px*corns[2][0]-row_offset)]
    #         col_list = np.sort(col_list)
    #         row_list = np.sort(row_list)
    #         col_list[3] = col_list[3] + 3
    #         col_list[0] = col_list[0] - 3
    #
    #         row_list[3] = row_list[3] + 3
    #         row_list[0] = row_list[0] - 3
    #
    #         # print('this is col_list', col_list)
    #         # print('this is row_list', row_list)
    #         img = cv2.line(img, (col_list[0], row_list[0]), (col_list[0], row_list[3]), (255, 0, 0), 1)
    #         img = cv2.line(img, (col_list[0], row_list[0]), (col_list[3], row_list[0]), (255, 0, 0), 1)
    #         img = cv2.line(img, (col_list[3], row_list[3]), (col_list[3], row_list[0]), (255, 0, 0), 1)
    #         img = cv2.line(img, (col_list[3], row_list[3]), (col_list[0], row_list[3]), (255, 0, 0), 1)
    #     # 11/13 comment
    #     #     img = cv2.line(img, (25, 455), (615, 455), (255, 0, 0), 26)
    #     #     img = cv2.line(img, (25, 455), (25, 0), (255, 0, 0), 26)
    #     #     img = cv2.line(img, (615, 0), (615, 455), (255, 0, 0), 26)
    #     #     img = cv2.line(img, (20, 8), (620, 8), (255, 0, 0), 16)
    #
    #     add = int((640 - 480) / 2)
    #     # img = cv2.copyMakeBorder(img, add, add, 0, 0, cv2.BORDER_CONSTANT, None, value=(0, 0, 0, 255))
    #
    #     # img.save("Dataset/yolo_test/" + "img%s.png" % epoch)
    #     cv2.imwrite("Dataset/pipe_test/" + "img%s.png" % epoch, img)
    #     # p.disconnect()
    #     # p.resetSimulation()
    #     # time.sleep(0.5)
    # # np.savetxt("Dataset/label/label_27.csv", lebal_list)
