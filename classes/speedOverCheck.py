from classes.speedMonitor import SpeedMonitor

class SpeedOverCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current >= limit):
            return 0
        else:
            return -1