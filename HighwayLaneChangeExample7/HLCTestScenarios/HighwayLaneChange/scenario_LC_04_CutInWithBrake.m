function [scenario, egoVehicle, roadCenters] = scenario_LC_04_CutInWithBrake()
%scenario_04_CutInWithBrake creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There are two other vehicles in the scenario. One vehicle travels
% in the right lane. Other vehicle cuts into the ego lane from the right
% lane and slows down. Ego car initiates lane change to avoid these
% vehicles.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 20, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0;
    600 0 0];
laneSpecification = lanespec(3);
straightRoad = road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10 0 0]);
waypoints = [10 0 0;
    200 0 0;
    201 0 0;
    250 3.6 0;
    251 3.6 0;
    500 3.6 0];
speed = 20;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFrontSlowCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [180 -3.6 0]);
waypoints = [180 -3.6 0;
    430 -3.6 0];
speed = 10;
trajectory(egoFrontSlowCar, waypoints, speed);

passingCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [70 -3.6 0]);
waypoints = [70 -3.6 0;
    140 -3.6 0;
    141 -3.6 0;
    190 0 0;
    191 0 0;
    210 0 0;
    211 0 0;
    400 0 0];
speed = [18;18;18;18;18;10;10;10];
trajectory(passingCar, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

