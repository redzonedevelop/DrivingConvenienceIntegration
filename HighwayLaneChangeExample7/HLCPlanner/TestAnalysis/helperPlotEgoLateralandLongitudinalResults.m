function hFigure = helperPlotEgoLateralandLongitudinalResults(logsout)
% helperPlotEgoLateralandLongitudinalResults function plots the profiles of
% ego vehicle after the simulation.
%
%   helperPlotEgoLateralandLongitudinalResults(logsout) plots the
%   orientation and velocity profiles of ego vehicle.
%
%   The function assumes that the example outputs the Simulink log,
%   logsout, containing the following elements to be plotted.
%
%   This is a helper function for example purposes and may be removed or
%   modified in the future.
%
%   Copyright 2020 The MathWorks, Inc.
%

%% Plot the spacing control results

% Create a figure handle
hFigure = figure('Name','Ego Performance Metrics','position',[100 100 720 600]);
     
%% Get the data from simulation
velocity = logsout.getElement('<RefVelocity>');
egoYaw = logsout.getElement('yaw_angle');

% Get Ego velocity and ego Yaw angle
velocityData = reshape(velocity.Values.Data, size(velocity.Values.Data,3), []);
egoYawData = reshape(egoYaw.Values.Data, size(egoYaw.Values.Data,3), [])';
tmax = velocity.Values.Time(end); % simulation time

subplot(2,1,1)
hold on

% Plot orientation of the first target vehicle 
plot(velocity.Values.Time,velocityData, 'Color', 'b')

% Set plot limits in x and y axises
xlim([0,tmax])
ylim([min(velocityData)-2, max(velocityData)+2])
grid on

% Set the plot title and axis labels
title('Ego Velocity')
xlabel('time (sec)')
ylabel('$m/s$','Interpreter','latex')

subplot(2,1,2)
hold on

% Plot orientation of the first target vehicle 
plot(egoYaw.Values.Time,egoYawData, 'Color', 'b')

% Set plot limits in x and y axises
xlim([0,tmax])
ylim([min(egoYawData)-2, max(egoYawData)+2])
grid on

% Set the plot title and axis labels
title('Ego Orientation')
xlabel('time (sec)')
ylabel('$degrees$','Interpreter','latex')
end