import datetime
import os
import cv2



class Recorder:
    """
    Class to record the video and store to the video folder
    """
    
    def __init__(self, type):
        """
        Setup the video format and where to store
        """
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.get_file_name(type)
        self.check_folder()
        self.writer = cv2.VideoWriter(self.path + self.file_name + '.mp4', fourcc, 30.0, (640,480))
    
    def get_file_name(self, type):
        """
        Generate the file name using the date and current time
        """
        now = datetime.datetime.now()
        self.file_name = str(now.year) + "-" + str(now.month) + "-" + str(now.day) + " " + str(now.hour) + ":" + str(now.minute) + ":" +str(now.second) + '-' + str(type)
        
    def check_folder(self):
        """        
        Detect if the video folder exist
        if not, create one
        """
        # cwd = os.getcwd() # current working directory
        # self.path = cwd + '/video/'
        name = os.getlogin()
        self.path = '/home/' + name + '/Line_follower/asset/videos/'
        
        isExist = os.path.exists(self.path) 
        if not isExist:
            os.mkdir(self.path)
        print('Video save in' + self.path)

    def __call__(self, frame):
        self.writer.write(frame)