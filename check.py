from duckietown_world.world_duckietown.map_loading import load_map
from duckietown_world.seqs import SampledSequence
from duckietown_world_tests import get_robot_trajectory
from duckietown_world.rules import evaluate_rules
from duckietown_world.rules.rule import make_timeseries
from duckietown_world import SE2Transform, RectangularArea, list_maps
from duckietown_world.svg_drawing import draw_static

from duckietown_world.world_duckietown.duckiebot import DB18

from comptests import get_comptests_output_dir

import geometry as geo
import numpy as np

def check_car_dynamics_correct(klass,CarCommands,CarParams):
    """

    :param klass: the implementation
    :return:
    """


    # load one of the maps (see the list using dt-world-draw-maps)
    outdir = get_comptests_output_dir()
    dw = load_map('udem1')

    v = 5

    # define a SampledSequence with timestamp, command
    commands_sequence = SampledSequence.from_iterator([
        # we set these velocities at 1.0
        (1.0, CarCommands(0.1 * v, 0.1 * v)),
        # at 2.0 we switch and so on
        (2.0, CarCommands(0.1 * v, 0.4 * v)),
        (4.0, CarCommands(0.1 * v, 0.4 * v)),
        (5.0, CarCommands(0.1 * v, 0.2 * v)),
        (6.0, CarCommands(0.1 * v, 0.1 * v)),
    ])

    # we upsample the sequence by 5
    commands_sequence = commands_sequence.upsample(5)

    ## Simulate the dynamics of the vehicle
    # start from q0 and v0
    q0 = geo.SE2_from_translation_angle([1.8, 0.7], 0)
    v0 = geo.se2.zero()
    c0 = q0,v0
    # instantiate the class that represents the dynamics
    dynamics = CarParams(10.0)
    # this function integrates the dynamics
    poses_sequence = get_robot_trajectory(dynamics, q0, commands_sequence)

    #################
    # Visualization and rule evaluation

    # Creates an object 'duckiebot'
    ego_name = 'duckiebot'
    db = DB18()  # class that gives the appearance

    # convert from SE2 to SE2Transform representation
    transforms_sequence = poses_sequence.transform_values(SE2Transform.from_SE2)
    # puts the object in the world with a certain "ground_truth" constraint
    dw.set_object(ego_name, db, ground_truth=transforms_sequence)

    # Rule evaluation (do not touch)
    interval = SampledSequence.from_iterator(enumerate(commands_sequence.timestamps))
    evaluated = evaluate_rules(poses_sequence=transforms_sequence,
                               interval=interval, world=dw, ego_name=ego_name)
    timeseries = make_timeseries(evaluated)
    # Drawing
    area = RectangularArea((0, 0), (3, 3))
    draw_static(dw, outdir, area=area, timeseries=timeseries)

    expected = {
        1.0: geo.SE2_from_translation_angle([1.8, 0.7], 0.0),
        1.6: geo.SE2_from_translation_angle([2.09998657, 0.70245831], 0.016389074695313716),
        2.0: geo.SE2_from_translation_angle([2.29993783, 0.70682836], 0.027315124492189525),
        2.4: geo.SE2_from_translation_angle([2.49991893, 0.70792121], -0.016385672773040847),
        2.8: geo.SE2_from_translation_angle([2.69975684, 0.70027647], -0.060086470038271216),
        3.2: geo.SE2_from_translation_angle([2.89906999, 0.68390873], -0.10378726730350164),
        3.6: geo.SE2_from_translation_angle([3.09747779, 0.65884924], -0.14748806456873198),
        4.0: geo.SE2_from_translation_angle([3.2946014 , 0.62514586], -0.19118886183396236),
        4.6: geo.SE2_from_translation_angle([3.58705642, 0.55852875], -0.25674005773180786),
        5.0: geo.SE2_from_translation_angle([3.77932992, 0.50353298], -0.3004408549970382),
        6.0: geo.SE2_from_translation_angle([4.2622088 , 0.37429798], -0.22257046876429318),
    }
    for t, expected_pose in expected.items():
        assert np.allclose(poses_sequence.at(t=t), expected_pose), t

    print('All tests passed successfully!')
