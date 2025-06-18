function [scenario, egoVehicle, roadCenters] = scenario_LC_01_SlowMoving()
%scenario_01_SlowMoving creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There is a slow moving lead vehicle in the scenario. Ego car
% initiates lane change to avoid collision with this vehicle.
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
    400 3.6 0];
speed = 20;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
egoFrontSlowCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [180 0 0]);
waypoints = [180 0 0;
    400 0 0];
speed = 10;
trajectory(egoFrontSlowCar, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end
