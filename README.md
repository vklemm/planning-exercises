# planning-exercises

[![CircleCI](https://circleci.com/gh/GITHUB-USERNAME/planning-exercises.svg?style=svg)](https://circleci.com/gh/GITHUB-USERNAME/planning-exercises)

Please login to [circleci.com](circleci.com) with your github account, go to `Add projects` find `planning-exercises`, click `Set Up Project` and click `Start Building`. Change `GITHUB-USERNAME` in the link above with your github username, in order to automatically test your solution.

## Homework Assignment 4

Fork and clone this repository: https://github.com/duckietown/planning-exercises

In the implementation.py implement the equivalent CarDynamics according to the description of Lavalle’s book (see 13.1.2.1 A simple car).

Create a class CarDynamics that integrates the dynamics of a car in the pattern of DifferentialDriveDynamics. We provided you with the classes CarCommmands and CarParameters.

CarCommands has two variables:

 - linear_velocity
 - steering_angle

CarParameters has one variable:

 - wheel_distance

To test your solution run `python test.py` in the repository. If your solution is correct you should see the message: `All tests passed successfully!`

To automatically test your solution commit your changes to your forked repository and note the CircleCI badge (make sure you followed instructions at the beginning of this document).

Copy the `implementation.py` file to `ethz-fall2018-subs/Homework Assignment 4/AMOD18-ETH-last_name-duckiebot_name/`. Push your solution and make a pull request.

Deadline is 23:59:59 on Tuesday 13.11.2018.
