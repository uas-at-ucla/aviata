from tkinter import *

class DebugWindow:
    def __init__(self,stage,target):
        self.window=Tk()
        self.stage=stage
        self.output_str=StringVar()
        self.output_str.set("Waiting for takeoff...")
        self.window.geometry("450x200")
        self.label=Label(self.window,textvariable=self.output_str)
        self.label.pack()
        self.target_lat=target.getLat()
        self.target_lon=target.getLon()
        self.target_alt=target.getAlt()
        self.target_yaw=target.getYaw() 

    def updateTargetLocation(self,target):
        self.target_lat=target.getLat()
        self.target_lon=target.getLon()
        self.target_alt=target.getAlt()
        self.target_yaw=target.getYaw() 

    def updateWindow(self,droneX,droneY,droneAlt,droneYaw,tagsDetected):
        self.output_str.set("Docking stage "+str(self.stage)+":\nDrone X: "+str(format(droneX,'0.2f'))+", Drone Y: "+str(format(droneY,'0.2f'))+", Drone Alt: "+
        str(format(droneAlt,'0.2f'))+", Drone Yaw: "+str(format(droneYaw,'0.2f'))+"\nTarget X: "+str(format(self.target_lat,'0.2f'))+", Target Y: "+str(format(self.target_lon,'0.2f'))+
        ", Target Alt: "+str(format(self.target_alt,'0.2f'))+", Target Yaw: "+str(format(self.target_yaw,'0.2f'))+"\nTags Detected: "+str(tagsDetected))
        self.window.update()

    def destroyWindow(self):
        self.window.destroy()
