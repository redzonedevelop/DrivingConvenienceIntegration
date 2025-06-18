function [scenario, egoVehicle, roadCenters] = scenario_LC_03_DisabledCar()
%scenario_03_DisabledCar creates a scenario that is compatible
% with HighwayLaneChangeTestBench.slx. This test scenario is on a Straight
% road. There is one break down vehicle in the same lane as ego. Ego car
% initiates lane change to avoid this vehicle.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2019-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 16, ...
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
    'Position', [290 0 0]);
waypoints = [290 0 0;
    290.02 0 0];
speed = 0.001;
trajectory(egoFrontSlowCar, waypoints, speed);

% Output road centers
roadCenters = straightRoad.RoadCenters;
end

