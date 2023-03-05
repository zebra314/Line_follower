       # PID setting
        self.P = 0
        self.I = 0
        self.D = 0
        self.Kp = 0.5
        self.Ki = 0.0008
        self.Kd = 0.6
        self.ideal = 0
        self.lastError = 0

        # motor speed
        self.motorSpeed = 55

 def PID(self):

        # right offset error > 0
        # left offeset error < 0 
        error = (self.ideal - self.position)*100/(self.ideal)
        # print(error)
        self.P = error
        # self.I = self.I + error
        self.D = error - self.lastError
        self.lastError = error

        # calculate the correction
        motorspeed = self.P * self.Kp # + self.D * self.Kd # + self.I * self.Ki

        leftspeed = int(self.motorSpeed - motorspeed)
        rightspeed = int(self.motorSpeed + motorspeed)

        stringspeed = str(leftspeed) + ' ' + str(rightspeed)
        # print(stringspeed)
        return stringspeed