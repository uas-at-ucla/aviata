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
        self.output_str.set("Docking stage "+str(self.stage)+":\nDrone X: "+str(droneX)+", Drone Y: "+str(droneY)+", Drone Alt: "+
        str(droneAlt)+", Drone Yaw: "+str(droneYaw)+"\nTarget X: "+str(self.target_lat)+", Target Y: "+str(self.target_lon)+
        ", Target Alt: "+str(self.target_alt)+", Target Yaw: "+str(self.target_yaw)+"\nTags Detected: "+str(tagsDetected))
        self.window.update()

    def destroyWindow(self):
        self.window.destroy()
