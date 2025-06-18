function hFigure = helperPlotLCPlannerExecutionProfile(executionProfile)
%helperPlotLCPlannerExecutionProfile Plots the simulation time taken by
%the step function that is generated from HighwayLaneChangePlanner.slx.
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2020 The MathWorks, Inc.
%%  Extract Simulation time and sample time taken for step function
for i=1:size(executionProfile.Sections,2)
    if strfind(executionProfile.Sections(i).Name, 'step')
        if ~isempty(executionProfile.TimerTicksPerSecond)
            step0_executionTime = executionProfile.Sections(i).ExecutionTimeInSeconds;
            step0_sampleTime = executionProfile.Sections(i).Time;
        else
			platform = profile('info');			
			executionProfile.TimerTicksPerSecond = platform.ClockSpeed;
            step0_executionTime = executionProfile.Sections(i).ExecutionTimeInTicks/executionProfile.TimerTicksPerSecond;
            step0_sampleTime = executionProfile.Sections(i).Time;
        end
    end
end
    endTime = step0_sampleTime(end);
    hFigure = figure('Name','Execution profiles');
    h = subplot(1,1,1);
    
    % Plot the execution profile from 2nd step by removing the
    % initialization peak
    line(h,step0_sampleTime(2:end,:),step0_executionTime(:,2:end),'Color','Red');
    grid on;
    title('SIL Task Execution Time')
    xlabel('Simulation Time (sec)');
    ylabel('Time (sec)');
    xlim([0 endTime]);
    ylim([0 0.005]);
end