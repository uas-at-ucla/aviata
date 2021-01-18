import datetime
class Log:
    def __init__(self):
        self.filename='logs/'+str(datetime.datetime.now().time())+'txt'
        self.file=open(self.filename,'a')

    def writeLiteral(self, literal):
        file=open(self.filename,'a')
        file.write(literal+"\n")
        file.close()

    def write(self,droneX,droneY,droneAlt,droneYaw,tagsDetected,successful_frames,x_err,y_err,alt_err,yaw_err):
        file=open(self.filename,'a')
        currLog="Drone X: "+str(format(droneX,'0.2f'))+", Drone Y: "+str(format(droneY,'0.2f'))+", Drone Alt: "+str(format(droneAlt,'0.2f'))+", Drone Yaw: "+str(format(droneYaw,'0.2f'))+", x_err: "+str(format(x_err,'0.2f'))+ ", y_err: "+str(format(y_err,'0.2f'))+ ", alt_err: "+str(format(alt_err,'0.2f'))+ ", x_err: "+str(format(yaw_err,'0.2f'))+", Tags Detected: "+str(tagsDetected)+", Successful Frames: "+str(successful_frames)+"\n"
        file.write(currLog)
        file.close()

    def close(self):
        self.file.close()