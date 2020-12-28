#Helper class for target
class Target:
    def __init__(self, lat, lon, alt, yaw):
        self.lat=lat
        self.lon=lon
        self.alt=alt
        self.yaw=yaw
    
    def getLat(self):
        return self.lat
    
    def getLon(self):
        return self.lon

    def getAlt(self):
        return self.alt
    
    def getYaw(self):
        return self.yaw
    
    def setLat(self,new):
        self.lat=new

    def setLon(self,new):
        self.lon=new

    def setAlt(self,new):
        self.alt=new
    
    def setYaw(self,new):
        self.yaw=new