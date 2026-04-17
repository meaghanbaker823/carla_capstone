from classes.MAPEStep import MAPEStep

import carla

"""
===========
Executor Class(MAPEStep)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class Executor(MAPEStep):
    def __init__(self):
        self.__new_parameters = []
        self.__old_parameters = []
        self.__old_actions = []
        self.__new_action = ""

    def get_new_action(self):
        return self.__new_action
    
    def format_new_action(self):
        return "The brake is: " - str(self.get_new_action()["brake"]) - ", the throttle is: "- str(self.get_new_action()["throttle"]) -  ", the steering is: " + str(self.get_new_action()["steering"])

    def execute(self, plan, car):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [plan, car]
        self.__old_actions.append(self.__new_action)
        self.__new_action = {"brake": plan["brake"], "steering": plan["steering"], "throttle": plan["throttle"]}
        print("Throttle:", plan["throttle"])
        print("Brake:", plan["brake"])

                    
        car.apply_control(carla.VehicleControl(throttle = plan["throttle"] ,steer = plan["steering"], brake = plan["brake"]))

        return self.__new_action

    def notify(self):
        return self.format_new_action()