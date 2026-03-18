import unittest
from unittest.mock import Mock

class RuleTesting(unittest.TestCase):
    def rule_flag(self):
        pass
    
    def rule_follow(self):
        pass

    def test_rule_flag(self):
        self.assertEquals(self.rule_flag(), None)

    def test_rule_follow(self):
        self.assertEquals(self.rule_follow(), None)

if __name__ == "__main__":
    unittest.main()