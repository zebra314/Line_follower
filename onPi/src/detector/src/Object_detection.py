import torch
import numpy as np
import cv2
import os

class ObjectDetection:
    """
    Class implements Yolo5 model to detect the rubik's cube in the image.
    """

    def __init__(self):
        self.model = self.load_model()
        self.classes = self.model.names

        self.device = 'cpu' # 'cuda' if torch.cuda.is_available() else 'cpu'
        print('\nDevice used: ',self.device,'\n')


    def load_model(self):
        """
        Loads Yolo5 model from the local weight file.
        :return: Trained Pytorch model.
        """
        username = os.getlogin()
        yolo_path = '/home/' + str(username) + '/Desktop/yolov5'
        weight_path = '/home/' + str(username) + '/Desktop/weights/endgame.pt'
        model = torch.hub.load(yolo_path, 'custom', source = 'local', path = weight_path) # fill location of the .pt file in path
        # beside the .pt file at he path
        # there also need to be a yolov5 dir which is clone from the offical github
        # at ultralytics/yolov5 in order to run without internet
        return model


    def score_frame(self, frame):
        """
        Takes a single frame as input, and scores the frame using yolo5 model
        :param frame: input frame in numpy/list/tuple format.
        :return: Labels and Coordinates of objects detected by mdoel in the frame.
        """
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)

        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord

    def class_to_label(self, x):
        """
        For a given label value, return corresponding string label.
        :param x: numeric label
        :return: corresponding string label 
        """
        return self.classes[int(x)]

    def plot_boxes(self, results, frame):
        """
        Takes a frame and its results as input, and plots the bounding boxes on the frame.
        Also stores the position info of the target area 
        :param results: contains labels and coordinates predicted by model on the given frame.
        :return: Frame with bounding boxes and labels ploted on it.
        """
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        self.target_cord =[0, 0, 0, 0]

        for i in range(n):
            row = cord[i]
            if row[4] < 0.2:
                continue

            x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
            w = abs(x1-x2)
            h = abs(y1-y2)
            bgr = (0, 255, 0)

            # square
            # if w/h >0.7 and w/h <1.3:
            cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
            self.target_cord = [min(y1, y2), min(y1, y2)+h, min(x1, x2), min(x1, x2)+w] # store the position of the target area

        return frame

    def __call__(self, frame):
        """
        This function is called when class is executed
        :return: the frame with the cube boxed and the coordinates of the cube
        """
        results = self.score_frame(frame)
        frame = self.plot_boxes(results, frame)

        if str(results[0]) == 'tensor([])':
            status = 'keep'
        else :
            status = 'stop'
        return status, frame