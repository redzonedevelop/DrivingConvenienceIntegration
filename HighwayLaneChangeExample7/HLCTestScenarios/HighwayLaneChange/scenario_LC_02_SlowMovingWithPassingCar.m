function [scenario, egoVehicle, roadCenters] = scenario_LC_02_SlowMovingWithPassingCar()
%scenario_02_SlowMovingWithPassingCar creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There are two other vehicles in the scenario. One vehicle is the
% lead vehicle and other is in the adjacent left lane. Ego car initiates
% lane change to avoid lead vehicle and the passing vehicle in the left
% lane.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 23, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0;
    600 0 0];
laneSpecification = lanespec(3);
straightRoad = road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [50 0 0]);
waypoints = [50 0 0;
    200 0 0;
    201 0 0;
    250 3.6 0;
    251 3.6 0;
    600 3.6 0];
speed = 20;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFrontSlowCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [180 0 0]);
waypoints = [180 0 0;
    600 0 0];
speed = 10;
trajectory(egoFrontSlowCar, waypoints, speed);

passingCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10 3.6 0]);
waypoints = [10 3.6 0;
    136 3.6 0;
    600 3.6 0];
speed = 33;
trajectory(passingCar, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

