from classes.MAPEStep import MAPEStep
import carla

class Executor(MAPEStep):
    def __init__(self):
        self.__new_parameters = []
        self.__old_parameters = []
        self.__old_actions = []
        self.__new_action = ""

    def get_new_action(self):
        return self.__new_action
    
    def format_new_action(self):
        return "The brake is: " + self.get_new_action()["brake"] + ", the throttle is: " + self.get_new_action()["throttle"] +  ", the steering is: " + self.get_new_action()["steering"]

    def execute(self, plan, car):
        self.__old_actions.append(self.__new_action)
        self.__new_action = {"brake": plan["brake"], "steering": plan["steering"], "throttle": plan["throttle"]}
                    
        car().apply_control(carla.VehicleControl(throttle = plan["throttle"] ,steer = plan["steering"], brake = plan["brake"]))

        return self.__new_action

    def notify(self):
        return "The action in this iteration is " + self.format_new_action()