

def check_car_dynamics_correct(klass):
    """

    :param klass: the implementation
    :return:
    """


    # load one of the maps (see the list using dt-world-draw-maps)
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
    # start from q0
    q0 = geo.SE2_from_translation_angle([1.8, 0.7], 0)

    # instantiate the class that represents the dynamics
    dynamics = klass()
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


    # TODO: compare poses_sequence to known result
    expected = {
        6.0: geo.SE2_from_translation_angle([1,2],23),
        5.0: geo.SE2_from_translation_angle([1, 2], 23),
    }
    for t, expected_pose in expected.items():
        assert np.allclose(poses_sequence.at(t=t), expected_pose), t
