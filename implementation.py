from duckietown_world import GenericKinematicsSE2
from duckietown_world.world_duckietown.car_dynamics import  CarCommands, CarParameters
import geometry as geo

class CarDynamics(GenericKinematicsSE2):

    def __init__(self, parameters, c0, t0):
        """
        :param parameters:  instance of CarParameters
        :param c0: initial configuration
        :param t0: initial time
        """
        check_isinstance(parameters, CarParameters)
        self.parameters = parameters
        GenericKinematicsSE2.__init__(self, c0, t0)

    def integrate(self, dt, commands):
        """

        :param dt:
        :param commands: an instance of WheelVelocityCommands
        :return:
        """
        check_isinstance(commands, CarCommands)


        # fill in here
        linear = [0,0]
        angular=  0
        # represent this as se(2)
        commands_se2 = geo.se2_from_linear_angular(linear, angular)

        # Call the "integrate" function of GenericKinematicsSE2
        s1 = GenericKinematicsSE2.integrate(self, dt, commands_se2)

        # new state
        c1 = s1.q0, s1.v0
        t1 = s1.t0
        return CarDynamics(self.parameters, c1, t1)
