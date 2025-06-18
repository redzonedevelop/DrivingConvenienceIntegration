function helperLaneChangeReport(logsout)
% helperLaneChangeReport function plots the profiles of
% ego vehicle after the simulation.
%
%   helperLaneChangeReport(logsout) plots the
%   orientation, velocity, longitudinal jerk and lateral jerk of ego
%   vehicle.
%
%   The function assumes that the example outputs the Simulink log,
%   logsout, containing the following elements to be plotted.
%
%   This is a helper function for example purposes and may be removed or
%   modified in the future.
%

%   Copyright 2021 The MathWorks, Inc.
%

%% Plot the spacing control results

% Create a figure handle
screenSize = double(get(groot,'ScreenSize'));
hFigureController = figure('Name','Lane Change Controller Performance',...
          'Position',[screenSize(3)*0.17 screenSize(4)*0.15 screenSize(3)*0.6 screenSize(4)*0.6]);
     
%% Get the data from simulation
velocity = logsout.getElement('ego_velocity');
egoSteering = logsout.getElement('steering_angle');
egoLongJerk = logsout.getElement('longitudinal_jerk');
egoLateralJerk = logsout.getElement('lateral_jerk');
previewedCurvature = logsout.getElement('previewed_curvature');
lateralDeviation = logsout.getElement('lateral_deviation');
headingAngle = logsout.getElement('relative_yaw_angle');
egoAcceleration = logsout.getElement('acceleration');

% Get the data for plotting
velocityData = reshape(velocity.Values.Data, size(velocity.Values.Data,3), []);
egoSteeringData   = reshape(egoSteering.Values.Data, size(egoSteering.Values.Data,3), [])';
longJerkData = reshape(egoLongJerk.Values.Data, size(egoLongJerk.Values.Data,3), []);
latJerkData  = reshape(egoLateralJerk.Values.Data, size(egoLateralJerk.Values.Data,3), []);
previewedCurvatureData = reshape(previewedCurvature.Values.Data(:,1), size(previewedCurvature.Values.Data(:,1),3), []);
lateralDeviationData = reshape(lateralDeviation.Values.Data, size(lateralDeviation.Values.Data,3), []);
headingAngleData = reshape(headingAngle.Values.Data, size(headingAngle.Values.Data,3), []);
egoAccData = reshape(egoAcceleration.Values.Data, size(egoAcceleration.Values.Data,3), []);

% Get simulation time
tmax = velocity.Values.Time(end);

subplot(5,1,1,'Parent',hFigureController)
hold on

% Plot previewed curvature.
plot(previewedCurvature.Values.Time,previewedCurvatureData, 'Color', 'b')

% Set plot limits in x and y axises
xlim([0,tmax])
ylim([min(previewedCurvatureData)-2, max(previewedCurvatureData)+2])
grid on

% Set the plot title and axis labels
title('Curvature')
xlabel('time (sec)')
ylabel('$1/m$','Interpreter','latex')

subplot(5,1,2,'Parent',hFigureController)
hold on

% Plot lateral deviation.
plot(lateralDeviation.Values.Time,lateralDeviationData, 'Color', 'b')

% Set plot limits in x and y axises
xlim([0,tmax])
ylim([min(lateralDeviationData)-2, max(lateralDeviationData)+2])
grid on

% Set the plot title and axis labels
title('Lateral Deviation')
xlabel('time (sec)')
ylabel('$m$','Interpreter','latex')

subplot(5,1,3,'Parent',hFigureController)
hold on

% Plot heading angle.
plot(headingAngle.Values.Time,headingAngleData, 'Color', 'b')

% Set plot limits in x and y axises
xlim([0,tmax])
ylim([min(headingAngleData)-2, max(headingAngleData)+2])
grid on

% Set the plot title and axis labels
title('Heading Angle')
xlabel('time (sec)')
ylabel('$degrees$','Interpreter','latex')

subplot(5,1,4,'Parent',hFigureController);
hold on;

% Plot velocity
plot(velocity.Values.Time,velocityData, 'Color', 'b');

% Set plot limits in x and y axises
xlim([0,tmax]);
ylim([min(velocityData)-2, max(velocityData)+2]);
grid on;

% Set the plot title and axis labels
title('Ego Velocity');
xlabel('time (sec)');
ylabel('$m/s$','Interpreter','latex');

subplot(5,1,5,'Parent',hFigureController);
hold on;

% Plot steering angle
plot(egoSteering.Values.Time,egoSteeringData, 'Color', 'b');

% Set plot limits in x and y axises
xlim([0,tmax]);
ylim([min(egoSteeringData)-2, max(egoSteeringData)+2]);
grid on;

% Set the plot title and axis labels
title('Steering Angle');
xlabel('time (sec)');
ylabel('$degrees$','Interpreter','latex');


%% Plot lane change metrics
% Create a different figure handle to plot lane change metrics.
hFigureMetrics = figure('Name','Lane Change Metrics',...
          'Position',[screenSize(3)*0.17 screenSize(4)*0.15 screenSize(3)*0.6 screenSize(4)*0.6]);
subplot(3,1,1,'Parent',hFigureMetrics);
hold on;

% Plot acceleration
plot(egoAcceleration.Values.Time,egoAccData, 'Color', 'b');

% Set plot limits in x and y axises
xlim([0,tmax]);
ylim([min(egoAccData)-2, max(egoAccData)+2]);
grid on;

% Set the plot title and axis labels
title('Acceleration');
xlabel('time (sec)');
ylabel('$m/s^2$','Interpreter','latex');

subplot(3,1,2,'Parent',hFigureMetrics);
hold on;

% Plot longitudinal jerk
plot(egoLongJerk.Values.Time,longJerkData, 'Color', 'b');

% Set plot limits in x and y axises
xlim([0,tmax]);
ylim([min(longJerkData)-2, max(longJerkData)+2]);
grid on;

% Set the plot title and axis labels
title('Longitudinal Jerk');
xlabel('time (sec)');
ylabel('$m/s^3$','Interpreter','latex');

subplot(3,1,3,'Parent',hFigureMetrics);
hold on;

% Plot lateral jerk 
plot(egoLateralJerk.Values.Time,latJerkData, 'Color', 'b');

% Set plot limits in x and y axises
xlim([0,tmax]);
ylim([min(latJerkData)-2, max(latJerkData)+2]);
grid on;

% Set the plot title and axis labels
title('Lateral Jerk');
xlabel('time (sec)');
ylabel('$m/s^3$','Interpreter','latex');

end