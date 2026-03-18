from unittest.mock import Mock
import unittest

# test maintain_speed
class MaintainSpeedTesting(unittest.TestCase):
    def setUp(self):
        #checks = [SpeedOverCheck(speed_threshold), SpeedPerfectCheck(speed_threshold), SpeedUnderCheck(speed_threshold)]
        self.checks = [Mock(), Mock(), Mock()]
        self.limit = 30
        self.new = 0
        self.threshold = 3

    
    def maintain_speed(self):


        new = 0
        for check in self.checks:
            if(check.speed_check(self.current, self.limit) != -1):
                new = check.speed_check(self.current, self.limit)
                break
        
        return new
    
    def test_maintain_speed1(self):
        # none of the checks return -1
        self.current = 28

        # setting up checks
        if(self.current >= self.limit):
            self.checks[0].speed_check.return_value =  0
        else:
            self.checks[0].speed_check.return_value =  -1

        if(self.current == (self.limit - self.threshold)):
            self.checks[1].speed_check.return_value  = 0.3
        else:
            self.checks[1].speed_check.return_value  = -1

        if(self.current < (self.limit - self.threshold)):
            self.checks[2].speed_check.return_value =  0.8
        else:
            self.checks[2].speed_check.return_value =  -1

        self.assertEqual(self.maintain_speed(), 0)

    def test_maintain_speed2(self):
        # need to change speed brach
        self.current = 26

        # setting up checks
        if(self.current >= self.limit):
            self.checks[0].speed_check.return_value =  0
        else:
            self.checks[0].speed_check.return_value =  -1

        if(self.current == (self.limit - self.threshold)):
            self.checks[1].speed_check.return_value  = 0.3
        else:
            self.checks[1].speed_check.return_value  = -1

        if(self.current < (self.limit - self.threshold)):
            self.checks[2].speed_check.return_value =  0.8
        else:
            self.checks[2].speed_check.return_value =  -1

        self.assertEqual(self.maintain_speed(), 0.8)
    
# test plan

# test notify


if __name__ == "__main__":
    unittest.main()