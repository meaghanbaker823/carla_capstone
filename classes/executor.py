from classes.MAPEStep import MAPEStep

import carla

class Executor(MAPEStep):
    """
    The class for the executor step in the MAPE structure
    """
    def __init__(self):
        """
        Initializes the variables needed for the Executor class
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """
        self.__new_parameters = []
        self.__old_parameters = []
        self.__old_actions = []
        self.__new_action = ""

    def get_new_action(self):
        """
        Getter for self.__new_action
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the steering, throttle, and brake controls
        """
        return self.__new_action

    def execute(self, plan, car):
        """
        Directly applies the controls from the planner onto the Carla car
        \n\tINPUT(S): plan: the steering angle, throttle, and brake controls to be applied,
                      car: the carla vehicle object
        \n\tOUTPUT(S):
        """
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [plan, car]
        self.__old_actions.append(self.__new_action)
        self.__new_action = {"brake": plan["brake"], "steering": plan["steering"], "throttle": plan["throttle"]}
                    
        car.apply_control(carla.VehicleControl(throttle = plan["throttle"] ,steer = plan["steering"], brake = plan["brake"]))

        return self.__new_action

    def notify(self):
        """
        Formats the actions completed for the MAPE Step
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): a string saying what the current vehicle controls are
        """
        return "The brake is: " + str(self.get_new_action()["brake"]) + ", the throttle is: " + str(self.get_new_action()["throttle"]) +  ", the steering is: " + str(self.get_new_action()["steering"])