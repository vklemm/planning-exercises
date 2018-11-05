

def test_my_implementation():
    """ This will be run by nose """
    from implementation import CarDynamics
    from check import check_car_dynamics_correct
    klass = CarDynamics
    check_car_dynamics_correct(klass)
