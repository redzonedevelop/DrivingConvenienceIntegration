function [scenario, egoVehicle, roadCenters] = scenario_LC_07_RightLaneChange()
%scenario_07_RightLaneChange creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There are three other vehicles in the scenario. There is a slow
% moving lead vehicle in the ego lane and other two vehicles are moving in
% the left lane. Ego car initiates a right lane change to avoid these
% vehicles.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 25, ...
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
    250 -3.6 0;
    251 -3.6 0;
    600 -3.6 0];
speed = 20;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFrontSlowCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [170 0 0]);
waypoints = [170 0 0;
    600 0 0];
speed = 10;
trajectory(egoFrontSlowCar, waypoints, speed);

passingCarRear = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10 3.6 0]);
waypoints = [10 3.6 0;
    136 3.6 0;
    600 3.6 0];
speed = [24, 20, 16];
trajectory(passingCarRear, waypoints, speed);

passingCarFront = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [100 3.6 0]);
waypoints = [100 3.6 0;
    600 3.6 0];
speed = 25;
trajectory(passingCarFront, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

