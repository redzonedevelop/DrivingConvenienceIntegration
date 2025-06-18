function [scenario, egoVehicle, roadCenters] = scenario_LC_05_SingleLaneChange()
%scenario_05_SingleLaneChange creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There are eight other vehicles in the scenario. Seven vehicles
% follow their respective lanes and one vehicle(leftRear1) does a lane
% change with high velocity. Ego vehicle initiates lane change to avoid
% collision with these vehicles.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 14, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0;
    700 0 0];
laneSpecification = lanespec(4);
straightRoad = road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [200 1.8 0]);
waypoints = [200 1.8 0;
    240.5 1.8 0;
    267 1.8 0;
    268 1.8 0;
    287 5.4 0;
    288 5.4 0;
    343.3 5.4 0;
    417.5 5.4 0;
    700 5.4 0];
speed = 15;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFront1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [290 1.8 0]);
waypoints = [290 1.8 0;
    300 1.8 0;
    340 1.8 0;
    400 1.8 0;
    500 1.8 0;
    550 1.8 0;
    600 1.8 0;
    700 1.8 0];
speed = [10;10;7;5;5;5;10;10];
trajectory(egoFront1, waypoints, speed);

leftRear1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [140 5.4 0]);
waypoints = [140 5.4 0;
    360 5.4 0;
    361 5.4 0;
    410 1.8 0;
    411 1.8 0;
    700 5.4 0];
speed = 34;
trajectory(leftRear1, waypoints, speed);

leftFront1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [304.7 5.4 0]);
waypoints = [304.7 5.4 0;
    370.2 5.4 0;
    453.2 5.4 0;
    700 5.4 0];
speed = 17;
trajectory(leftFront1, waypoints, speed);

right1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [186 -1.8 0]);
waypoints = [186 -1.8 0;
    250.4 -1.8 0;
    700 -1.8 0];
speed = 12;
trajectory(right1, waypoints, speed);

right2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [222 -1.8 0]);
waypoints = [222 -1.8 0;
    286 -1.8 0;
    700 -1.8 0];
speed = 12;
trajectory(right2, waypoints, speed);

right3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [258 -1.8 0]);
waypoints = [258 -1.8 0;
    332.3 -1.8 0;
    700 -1.8 0];
speed = 12;
trajectory(right3, waypoints, speed);

right4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [294 -1.8 0]);
waypoints = [294 -1.8 0;
    340.6 -1.8 0;
    700 -1.9 0];
speed = 12;
trajectory(right4, waypoints, speed);

leftRear2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [50 5.4 0]);
waypoints = [100 5.4 0;
    700 5.4 0];
speed = 20;
trajectory(leftRear2, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

