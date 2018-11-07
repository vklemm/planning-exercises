

def test_my_implementation():
    """ This will be run by nose """
    from implementation import CarDynamics, CarCommands, CarParameters
    from check import check_car_dynamics_correct
    klass = CarDynamics
    commands = CarCommands
    params = CarParameters
    check_car_dynamics_correct(klass,commands,params)


if __name__ == '__main__':
    test_my_implementation()
