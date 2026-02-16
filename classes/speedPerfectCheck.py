from classes.speedMonitor import SpeedMonitor

class SpeedPerfectCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current == (limit - self.get_threshold())):
            return 0.3
        return -1