#!/usr/bin/env python

import numpy as np
import cv2

import time
import rospy


class CollectTrainingData(object):

    def __init__(self):


        self.video_width = 480  
        self.video_height = 320  
        self.NUM = 2  
        self.range = 1000 
        self.data_path = "dataset"
        self.saved_file_name = 'labeled_img_data_' + str(int(time.time()))
        self.count = 0
        self.isLeft = False
        self.isRight = False
        # create labels
        self.k = np.zeros((self.NUM, self.NUM), 'float')
        for i in range(self.NUM):
            self.k[i, i] = 1

        self.collect_image()

    def collect_image(self):

        
        total_images_collected = 0
        num_list = [0, 0, 0, 0, 0, 0, 0]
        cap = cv2.VideoCapture(0)
        images = np.zeros((1, self.video_height * self.video_width), dtype=float)
        labels = np.zeros((1, self.NUM), dtype=float)


        while cap.isOpened():
            _, frame = cap.read()
            resized_height = int(self.video_width * 0.75)
            
            frame = cv2.resize(frame, (self.video_width, resized_height))
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # slice the lower part of a frame
            res = frame[resized_height - self.video_height:, :]
            cv2.imshow("review", res)

            command = cv2.waitKey(1) & 0xFF
            if command == ord('q'):
                break
            # forward -- 0
            elif command == ord('a'):
                self.isLeft = True
            elif command == ord('d'):
                self.isRight = True

            if self.isLeft:
                if self.count <= self.range:
                    self.count += 1
                    cv2.imwrite('/home/bcsh/dataset/' + str(self.count) + ".png", res)
                    print("left image collect: ", num_list[0])
                else:
                    print("left success")
                    self.isLeft = False
                    continue


            if self.isRight:
                if self.count <= self.range:
                    self.count += 1
                    cv2.imwrite('dataset/car' + str(self.count) + ".png", res)
                    print("right image collect: ", num_list[0])
                else:
                    print("right success")
                    self.isRight = False
                    continue


    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("image_collect_node")
    CollectTrainingData()
    rospy.spin()
