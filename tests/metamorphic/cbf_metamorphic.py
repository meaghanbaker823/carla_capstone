import unittest

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from classes.CBFRule import CBFRule

'''
This function calculates a metamorphic relation for the function final_logic() in the CBFRule

This relation is between the
input: (standard_distance, extra, alpha, dt, u_nom, maxx_acc, max_brake, cbf)
output: (result)

Because final logic relies of the output of calculate_min_distance(),
calculate_safety_function(), and calculate_allowable_distance(). Therefore the relation function
replicates this calculations to give a realistic relation to the function.

final_logic():
(u_nom, u_cbf max_acc, max_brake)                                             ->        (result)

calculate_relation():
(standard_distance, extra, alpha, dt, u_nom, maxx_acc, max_brake, cbf)        ->        (result)
'''
def calculate_relation(standard_distance, extra, alpha, dt, u_nom, max_acc, max_brake, cbf: CBFRule):
        min_distance = standard_distance + (cbf._CBFRule__r * extra)
        h = cbf._CBFRule__distance - min_distance
        allowable_distance = (((alpha * h) - cbf._CBFRule__s) / dt/1000)
        result = max(min(u_nom, allowable_distance, max_acc), -1 * max_brake)

        return result

class MetamorphicFinalLogic(unittest.TestCase):
    def setUp(self):
        self.rules = [CBFRule(0, 10, 5),
            CBFRule(0, 20, 10),
            CBFRule(0, 30, 15),
            CBFRule(1, 10, 5),
            CBFRule(1, 20, 10),
            CBFRule(1, 30, 15)]

        # these are static in the program
        self.dt = 0.005
        self.extra = 6
        self.alpha = 1.5
        self.standard_distance = 8
        self.u_nom = 1
        self.max_acc = 3
        self.max_brake = 6

        '''
        Each test case; labeled test_meta_0 - test_meta_3, test different scenarios by comparing the output
        of final_logic() to the calculate_relation() function by verifying that the output is equal to the
        output of the real final_logic() function.
        '''
    def test_meta_0(self):
        cbf = self.rules[0]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

    def test_meta_1(self):
        cbf = self.rules[1]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

    def test_meta_2(self):
        cbf = self.rules[2]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

    def test_meta_3(self):
        cbf = self.rules[3]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

    def test_meta_4(self):
        cbf = self.rules[4]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

    def test_meta_5(self):
        cbf = self.rules[5]

        min_distance = cbf.calculate_min_distance(self.extra, self.standard_distance) 
        h = cbf.calculate_safety_function(min_distance)
        u_cbf = cbf.calculate_allowable_distance(self.alpha, h, self.dt)
        output = cbf.final_logic(self.u_nom, u_cbf, self.max_acc, self.max_brake)

        result = calculate_relation(self.standard_distance, self.extra, self.alpha, self.dt, self.u_nom, u_cbf, self.max_acc, self.max_brake, cbf)
        
        self.assertEqual(output, result)

if __name__ == "__main__":
    unittest.main()
