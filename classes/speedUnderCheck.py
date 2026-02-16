from classes.speedMonitor import SpeedMonitor

class SpeedUnderCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current < (limit - self.get_threshold())):
            return 0.8
        return -1