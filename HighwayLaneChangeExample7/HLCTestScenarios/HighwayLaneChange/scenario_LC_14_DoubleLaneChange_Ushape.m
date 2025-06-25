function [scenario, egoVehicle, roadCenters] = scenario_LC_14_DoubleLaneChange_Ushape()
% scenario_LC_14_DoubleLaneChange_Ushape creates a scenario that is
% compatible with HighwayLaneChangeTestBench.slx. This test scenario is on
% a "U" shaped road. This is same as the one used in the shipped as part of
% Highway Trajectory Planning Using Frenet Reference Path example. Ego
% vehicle initiates lane change to avoid collision with these vehicles.
% In this scenario ego vehicle performs lane change twice to avoid collision
% with disabled vehicle in the middle lane.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

%   Copyright 2020-2021 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 39, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 50 0;
    150 50 0;
    300 75 0;
    310 75 0;
    400 0 0;
    300 -50 0;
    290 -50 0];
laneSpecification = lanespec(4);
ushapedRoad = road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [11.8982627066058 50.9922295445627 0.01], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'egoCar');
waypoints = [11.8982627066058 50.9922295445627 0.01;
    47 48.7 0];
speed = [11;11];
waittime = [0;0];
trajectory(egoVehicle, waypoints, speed, waittime);

% Add the non-ego actors
leadorangecar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [48.5991851586063 48.8050182840696 0], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'leadOrangeCar');
waypoints = [48.5991851586063 48.8050182840696 0;
    60.1 48.2 0;
    84.2 47.9 0;
    119 49.3 0;
    148.1 51.4 0;
    189.6 58.7 0;
    230.6 68 0;
    272.6 74.7 0;
    301.3 76.8 0;
    317 76.5 0;
    332.4 75.2 0;
    351.3 70.6 0;
    365.7 64.4 0;
    379.6 55.6 0;
    391.9 42.1 0;
    400.3 25.5 0;
    402.6 11.3 0;
    400.4 -5 0];
speed = [10;10;10;10;10;10;10;10;10;10;10;10;10;10;10;10;10;10];
trajectory(leadorangecar, waypoints, speed);

thirdlaneyellowcar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [17.6 46.7 0], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'thirdLaneYellowCar');
waypoints = [17.6 46.7 0;
    43.4 45.5 0;
    71.3 43.8 0;
    102.3 43.5 0;
    123.5 45.5 0;
    143.6 47.4 0;
    162.4 50 0;
    198.5 61 0;
    241.1 70.1 0;
    272.3 74.1 0;
    292 76.6 0;
    312.8 77.2 0;
    350.3 75.2 0;
    362.5 70.4 0;
    375.9 63.3 0;
    390.7 49.9 0;
    401.3 33 0];
speed = [9;9;9;9;9;9;9;9;9;9;9;9;9;9;9;9;9];
trajectory(thirdlaneyellowcar, waypoints, speed);

firstlanevioletcar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [62.6 51.9 0], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'firstLaneVioletCar');
waypoints = [62.6 51.9 0;
    87.4 51.3 0;
    117.7 52.2 0;
    147.6 55 0;
    174.9 59.7 0;
    203.3 65.8 0;
    243.9 70.21 0.01;
    256.55 72.25 0.01;
    288.3 73.1 0;
    314.5 73.1 0;
    334.9 70.8 0;
    360 59.9 0;
    375.6 49.6 0;
    385.9 37.9 0;
    392.3 26.2 0;
    395.3 15.4 0];
speed = [6;6;6;6;6;6;6;6;6;6;6;6;6;6;6;6];
trajectory(firstlanevioletcar, waypoints, speed);

forthlanegreencar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [101.7 41.1 0], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'forthLaneGreenCar');
waypoints = [101.7 41.1 0;
    124.6 42 0;
    148.5 43.9 0;
    171.9 48.2 0;
    197.1 52.8 0;
    222.3 58.5 0;
    252.4 64.4 0;
    281.4 68.5 0;
    307.7 69.5 0;
    329.9 68.2 0;
    352.7 62.8 0;
    365 57.6 0;
    379.7 45.9 0;
    388.4 34.7 0;
    392.8 26.2 0;
    395.5 15 0];
speed = [7;7;7;7;7;7;7;7;7;7;7;7;7;7;7;7];
trajectory(forthlanegreencar, waypoints, speed);

disabledcyancar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [252.93 68.09 0.01], ...
    'FrontOverhang', 0.9, ...
    'Wheelbase', 2.8, ...
    'Name', 'disabledCyanCar');
waypoints = [252.93 68.09 0.01;
    257.15 68.93 0.01];
speed = [0.01;0.01];
trajectory(disabledcyancar, waypoints, speed);

% Output road centers
roadCenters = ushapedRoad.RoadCenters;
end