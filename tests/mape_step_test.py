import unittest

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from classes.MAPEStep import MAPEStep

def setUpModule():
    global mape_step
    mape_step = MAPEStep()

class NotifyTest(unittest.TestCase):
    def test_notify(self):
        self.assertTrue(mape_step.notify())

if __name__ == "__main__":
    unittest.main()
