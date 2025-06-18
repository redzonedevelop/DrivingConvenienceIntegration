function hFigure = helperPlotLCPlannerDiffSignals(diffResult)
%helperPlotLCPlannerDiffSignals Plots the signal differences that are
%computed from two simulation runs of
%HighwayLaneChangePlannerTestbench.slx.
% Required Input
% diffResult: Simulink.sdi.DiffRunResult object obtained from
% Simulink.sdi.compareRuns 
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2020 The MathWorks, Inc.

%%
% Get velocity difference
velocityDiffResult = getResultsByName(diffResult,'ego_velocity');

% Get yaw angle difference
yawAngleDiffResult = getResultsByName(diffResult,'yaw_angle');

% Get curvature difference
curvatureDiffResult = getResultsByName(diffResult,'ref_point_on_path.RefCurvature');

%% Plot the difference results
hFigure =figure('Name','Planner Output Parameters');
subplot(3,1,1)
plot(velocityDiffResult.Diff.Time,velocityDiffResult.Diff.Data(:))
ylim([-0.0001 0.0001]);
title('Velocity')
xlabel('time (sec)')
ylabel('diff signal')

subplot(3,1,2)
plot(yawAngleDiffResult.Diff.Time,yawAngleDiffResult.Diff.Data(:))
ylim([-0.001 0.001]);
title('Yaw Angle')
xlabel('time (sec)')
ylabel('diff signal')

subplot(3,1,3)
plot(curvatureDiffResult.Diff.Time,curvatureDiffResult.Diff.Data(:))
ylim([-0.001 0.001]);
title('Curvature')
xlabel('time (sec)')
ylabel('diff signal')

end