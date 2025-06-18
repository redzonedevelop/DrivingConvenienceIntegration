function [scenario, egoVehicle] = scenario_SVSF_02_CrossTargetActors()
%scenario_SVSF_02_CrossTargetActors creates a scenario that is compatible
% with SurroundVehicleSensorFusionTestBench.slx.  This test scenario is on
% a Straight road. There are two target vehicles in the scenario in
% adjacent lanes to the ego car. Ego car tarvels with more velocity
% compared to target vehicles and crosses them in the middle lane.
%
% This scenario is created in Driving Scenario Designer and exported as a
% MATLAB function.

% Copyright 2020 The MathWorks, Inc.

% Construct a drivingScenario object.
scenario = drivingScenario('StopTime', 23, ...
    'SampleTime', 0.1);

% Add all road segments
roadCenters = [0 0 0;
    600 0 0];
laneSpecification = lanespec(3);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [25 0 0]);
waypoints = [25 0 0;
    400 0 0];
speed = 16;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
Car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [50 3.6 0]);
waypoints = [75 3.6 0;
    400 3.6 0];
speed = 10;
trajectory(Car1, waypoints, speed);

% Add the non-ego actors
Car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [50 -3.6 0]);
waypoints = [75 -3.6 0;
    400 -3.6 0];
speed = 10;
trajectory(Car2, waypoints, speed);

end