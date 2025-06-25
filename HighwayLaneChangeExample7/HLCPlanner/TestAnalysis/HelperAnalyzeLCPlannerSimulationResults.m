classdef HelperAnalyzeLCPlannerSimulationResults < matlab.System
%%HelperAnalyzeLCPlannerSimulationResults helps in visualizing the 
% simulation results and its corresponding planner information on a MATLAB 
% figure for post simulation analysis.
%
%   NOTE: This is a helper function for example purposes and
%   may be removed or modified in the future.

%   Copyright 2021-2022 The MathWorks, Inc.

    properties(SetAccess=protected,Hidden=true)
        % Figure holds the instance of MATLAB figure and their properties.
        HFigure;

        % HScenViewAxes holds the axes that shows lane change
        % visualization.
        HScenViewAxes;

        % Line handles for plotting trajesctories
        LineHandles;

        % Slider handle
        HSlider;

        % Log of output of HLC planner.
        VisualizationInfo;

        % Main grid layout of the figure.
        GridLayout;

        % Logged signals.
        LogSignals;
        
        % Capsule list.
        CapList;

        % Handle for uitable for MIO information.
        MIOTable;

        % Handle for uitable for trajectory information.
        TrajectoryInfoTable;

        % Handle for uilabel for simulation step.
        CurrentStepEditField;
        % Handle for uilabel for simulation time.
        CurrentTimeEditField;
        % Handle for uilabel for mode.
        EgoModeEditField;
        % Handle for uilabel for ego velocity.
        EgoSpeedEditField;

        % Struct for handles of uilabel for planner parameters.
        PlannerParamsStruct;

        % Struct for handles of uilabel for ego state paramters.
        EgoStateParamsStruct;
    end

    methods
        function obj = HelperAnalyzeLCPlannerSimulationResults(logsout)
            %HelperAnalyzeLCPlannerSimulationResults Construct an instance 
            % of this class.

            % Create and set Figure Properties.
            figureName = 'Lane Change Post Simulation Analysis';
            obj.HFigure = findobj('Type','Figure','Name',figureName);
            if isempty(obj.HFigure)
                screenSize = double(get(groot,'ScreenSize'));
                obj.HFigure = uifigure('Name',figureName);
                obj.HFigure.Position = [screenSize(3)*0.17 screenSize(4)*0.15 screenSize(3)*0.6 screenSize(4)*0.6];
                obj.HFigure.NumberTitle = 'off';
                obj.HFigure.MenuBar = 'none';
                obj.HFigure.ToolBar = 'none';
                obj.HFigure.Visible = 'off';
            end
            
            % Clear figure.
            clf(obj.HFigure);

            % Create GridLayout
            obj.GridLayout = uigridlayout(obj.HFigure);
            obj.GridLayout.ColumnWidth = {'2x', '0.9x', '0.9x'};
            obj.GridLayout.RowHeight = {'2x', '1x', '3x'};
            
            % Create axes handle for the figure.
            hPanelChase = uipanel(obj.GridLayout,'Title','Chase View');
            hPanelChase.Layout.Row = [1 2];
            hPanelChase.Layout.Column = 1;
            obj.HScenViewAxes = axes('Parent',hPanelChase);

            % Add resize callback
            obj.HFigure.AutoResizeChildren = 'off';
            obj.HFigure.SizeChangedFcn = @(src,event)figureResizeCallbak(obj,src,event);

            % Read log data.
            readLogData(obj, logsout);
        end

        function visualizeSimulationData(obj)
            %visualizeSimulationData visualizes simulation data. It
            % initialzes MATLAB figure and plots the first time step
            % simulation data. Callback for the slider is initialized which
            % is used to visualize the simulation data.

            % Read required variables from base workspace.
            egoFrontExt = evalin('base','egoFrontExt');
            targetFrontExt = evalin('base','targetFrontExt;');
            scenarioFcnName = evalin('base','scenarioFcnName');

            initializeMATLABFigure(obj);

            % Define default start index of the slider.
            sliderStartIdx = 1;
            obj.HSlider.Value = sliderStartIdx;

            % Read the scenario from the scenario function name.
            scenarioFcnHandle = str2func(scenarioFcnName);
            [scenarioObj, egoCar] = scenarioFcnHandle();

            % Get number of actors.
            numTargetActors = size(scenarioObj.Actors,2)-1;

            % Get obstacle profiles from scenario.
            obstacleProfiles = actorProfiles(scenarioObj);

            maxTrajectories = size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory,1);
            maxTrajectoryPoints = size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(1).Trajectory.Data,1);

            % Length of the car.
            carLength = 4.7;
            
            % Width of the car.
            carWidth = 1.8;

            % Define capsule list for plotting.
            obj.CapList = dynamicCapsuleList;
            
            % Updating the EgoGeometry in Capsule List.
            [~, egoGeom] = egoGeometry(obj.CapList, egoCar.ActorID);
            egoGeom.Geometry.Length = carLength + egoFrontExt; % in meters
            egoGeom.Geometry.Radius = carWidth/2; % in meters
            egoGeom.Geometry.FixedTransform(1,end) = 1;
            updateEgoGeometry(obj.CapList, egoCar.ActorID, egoGeom);
            
            % Initialize Capsule List with default actor ids and default
            % actor geometry.
            actorID=(1:12)';
            targetGeom = egoGeom;
            targetGeom.Geometry.Length = carLength + targetFrontExt;
            actorGeom = repelem(targetGeom,12,1);
            updateObstacleGeometry(obj.CapList, actorID, actorGeom);

            % Initialize chase view.
            chasePlot(egoCar, 'Parent', obj.HScenViewAxes, 'ViewLocation',-[carLength*4, 0],'ViewHeight',11,'ViewPitch',20);
            
            % Get simulation data and update UI elements.
            [egoActor, targetActorsInWorld, futureTrajectory] = updateSimulationData(obj,obstacleProfiles,maxTrajectories, egoCar,maxTrajectoryPoints,numTargetActors,egoGeom,sliderStartIdx);

            % Update actors information from logged data to scenario object.
            plotColor = updateActorPosesToScenarioObj(obj, scenarioObj, egoActor, targetActorsInWorld);

            % Update MIO information in MIO table.
            updateMIOTable(obj, obj.MIOTable, obj.LogSignals.mioInfoData, futureTrajectory, plotColor, sliderStartIdx);

            % Update plots.
            updatePlots(scenarioObj)
            
            % Define call back function for slider.
            obj.HSlider.ValueChangedFcn = @(src, event) stepSimulation(obj, obstacleProfiles, maxTrajectories, scenarioObj, egoCar, maxTrajectoryPoints, numTargetActors, egoGeom);
        end

    end
    methods(Access=protected)
        function readLogData(obj, logsout)
            %readLogData reads required data from the simulation log. It
            %take logsout as input which contains all the logged signals.

            % Get logged elements.
            obj.LogSignals.egoPoseData     = logsout.getElement('ego_actor');
            obj.LogSignals.targetActors    = logsout.getElement('target_actors_world');
            obj.LogSignals.preferredLane   = logsout.getElement('current_preferred_lane');
            obj.LogSignals.terminalStates  = logsout.getElement('sorted_states');
            obj.LogSignals.mioInfoData     = logsout.getElement('mio_info');
            obj.LogSignals.plannerParams   = logsout.getElement('planner_parameters');
            obj.LogSignals.refPointOnPath  = logsout.getElement('ref_point_on_path');
            obj.LogSignals.egoCurrentLane  = logsout.getElement('current_ego_lane');
            obj.LogSignals.leadCarSpeed    = logsout.getElement('mio_relative_long_velocity_truth');
            obj.LogSignals.headway         = logsout.getElement('headway');
            obj.LogSignals.egoVelocityData = logsout.getElement('ego_velocity');
            vizData         = logsout.getElement('viz_data');
            obj.VisualizationInfo = vizData.Values;
        end

        function initializeMATLABFigure(obj)
            %initializeMATLABFigure initializes MATLAB figure and its UI
            %elements.

            % Create legend for chase view.
            hChase(1) = line(obj.HScenViewAxes,0,0,...
                'LineStyle','-','Color','g','LineWidth',3);
            set(hChase(1),'XData',[],'YData',[]);
            
            hChase(2) = line(obj.HScenViewAxes,0,0,...
                'LineStyle','-','Color','r','LineWidth',2);
            set(hChase(2),'XData',[],'YData',[]);
            
            hChase(3) = line(obj.HScenViewAxes,0,0,...
                'LineStyle','--','Color','c','LineWidth',2);
            set(hChase(3),'XData',[],'YData',[]);
            
            hChase(4) = line(obj.HScenViewAxes,0,0,...
                'LineStyle','-','Color','k','LineWidth',2);
            set(hChase(4),'XData',[],'YData',[]);
            
            legend(obj.HScenViewAxes,hChase,...
                {'optimal','colliding','infeasible','not evaluated'},...
                'Color',[0.94,0.94,0.94],...
                'AutoUpdate','off',...
                'Location','bestoutside');

            maxTrajectories = size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory,1);
            numSimSteps = size(obj.VisualizationInfo.TrajectoryInfo.NumTrajectories.Data,1);
            
            % Initialize line handles to plot candidate trajectories in top
            % view and chase view.
            obj.LineHandles = [];
            if isempty(obj.LineHandles)
                for i = 1:maxTrajectories
                    obj.LineHandles(end+1) = line('Parent',obj.HScenViewAxes);
                end
            end
            
            % Initialize uicontrol labels, text fields, tables required to update the
            % required information.
            initializeUIConrolForPostAnalysis(obj, numSimSteps)

            % Define default start index of the slider.
            sliderStartIdx = 1;
            obj.HSlider.Value = sliderStartIdx;

            obj.HFigure.Visible = 'on';
        end

        function stepSimulation(obj, obstacleProfiles, maxTrajectories, scenarioObj, egoCar, maxTrajectoryPoints, numTargetActors, egoGeom)
            %stepSimulation is the slider call back function. It is used to
            % update the information on MATLAB figure from the logged data
            % of lane change system.
            %
            % It takes below parameters as intputs:
            %   obstacleProfiles    - Actor profiles of the vehicles.
            %   maxTrajectories     - Maximum number of trajectories.
            %   scenarioObj         - Driving scenario object.
            %   egoCar              - Ego vehicle runtime information.
            %   maxTrajectoryPoints - Maximum number of points in trajectory.
            %   numTargetActors     - Number of target actors.
            %   egoGeom             - Ego vehicle geometry.
            
            % Get the index value from slider.
            sliderValue = round(obj.HSlider.Value);
            idx = sliderValue;
            
            % update UI elements in the figure
            [egoActor, targetActorsInWorld, futureTrajectory] = updateSimulationData(obj,obstacleProfiles,maxTrajectories, egoCar,maxTrajectoryPoints,numTargetActors,egoGeom,idx);
            
            % Update actors information from logged data to scenario object.
            plotColor = updateActorPosesToScenarioObj(obj, scenarioObj, egoActor, targetActorsInWorld);
            
            % Update MIO information in MIO table.
            updateMIOTable(obj, obj.MIOTable, obj.LogSignals.mioInfoData, futureTrajectory, plotColor, idx);
            
            % Update plots.
            updatePlots(scenarioObj);
        end

        function plotColor = updateActorPosesToScenarioObj(~, scenarioObj, egoActor, targetActorsInWorld)
            % updateActorPosesToScenarioObj updates actor poses from logged
            % data to scenario object and returns the color of vehicles
            % that are used for visualization.
            %
            % It takes scenario object, ego actor pose and traget actors
            % pose information.
            
            % Update actor info to scenario object for plotting.
            scenarioObj.Actors(egoActor.ActorID).Position = egoActor.Position;
            scenarioObj.Actors(egoActor.ActorID).Velocity = egoActor.Velocity;
            scenarioObj.Actors(egoActor.ActorID).Roll = egoActor.Roll;
            scenarioObj.Actors(egoActor.ActorID).Pitch = egoActor.Pitch;
            scenarioObj.Actors(egoActor.ActorID).Yaw = egoActor.Yaw;
            scenarioObj.Actors(egoActor.ActorID).AngularVelocity = egoActor.AngularVelocity;
            
            % Get plot color of actors.
            plotColor = zeros(targetActorsInWorld.NumActors, 3);
            
            numActors = size(scenarioObj.Actors,2);
            targetActorIdx = 1;
            
            % Update actors information in Scenario object.
            for n=1:numActors
                if egoActor.ActorID ~= n
                    scenarioObj.Actors(n).Position = targetActorsInWorld.Actors(targetActorIdx).Position;
                    scenarioObj.Actors(n).Velocity = targetActorsInWorld.Actors(targetActorIdx).Velocity;
                    scenarioObj.Actors(n).Roll = targetActorsInWorld.Actors(targetActorIdx).Roll;
                    scenarioObj.Actors(n).Pitch = targetActorsInWorld.Actors(targetActorIdx).Pitch;
                    scenarioObj.Actors(n).Yaw = targetActorsInWorld.Actors(targetActorIdx).Yaw;
                    scenarioObj.Actors(n).AngularVelocity = targetActorsInWorld.Actors(targetActorIdx).AngularVelocity;
                    plotColor(targetActorIdx,:) = scenarioObj.Actors(n).PlotColor;
                    targetActorIdx = targetActorIdx + 1;
                end
            end
        end

        function [egoActor, targetActorsInWorld, futureTrajectory] = updateSimulationData(obj, obstacleProfiles, maxTrajectories, egoCar, maxTrajectoryPoints, numTargetActors, egoGeom, sliderValue)
            % updateSimulationData gets simulation data and update UI
            % elements of the MATLAB figure at the provided slider values.
            %
            % It takes below parameters as intputs:
            %   obstacleProfiles    - Actor profiles of the vehicles.
            %   maxTrajectories     - Maximum number of trajectories.
            %   egoCar              - Ego vehicle runtime information.
            %   maxTrajectoryPoints - Maximum number of points in trajectory.
            %   numTargetActors     - Number of target actors.
            %   egoGeom             - Ego vehicle geometry.
            %   sliderValue         - Positon of the thumb on the slider.

            % Update current simulation step.
            obj.CurrentStepEditField.Text = num2str(sliderValue);
            
            % Update current simulation time.
            obj.CurrentTimeEditField.Text = num2str(obj.LogSignals.plannerParams.Values.TimeResolution.Time(sliderValue));
            
            % Get preferred.
            currentPreferredLane = reshape(obj.LogSignals.preferredLane.Values.Data, size(obj.LogSignals.preferredLane.Values.Data,1), []);
            
            % Retrieve information for plotting and updating from the elements of logged data.
            [globalTraj, futureTrajectory, egoActor, targetActorsInWorld]= recreateDataForPlotting(obj, maxTrajectories, maxTrajectoryPoints, obj.LogSignals.egoPoseData, obj.LogSignals.targetActors, numTargetActors, sliderValue);

            % Get current mode of the optimal trajectory.
            currentMode = globalTraj.GlobalTrajectory(globalTraj.OptimalTrajectoryIndex).BehaviorType;
            
            % Update trajectory information to trajectory information table.
            updateTrajectoryInfoTable(obj, obj.TrajectoryInfoTable, obj.LogSignals.terminalStates, globalTraj, sliderValue);
            
            % Update ego and lead vehicle information.
            updateEgoStateInformation(obj, obj.LogSignals.egoCurrentLane, obj.LogSignals.refPointOnPath, obj.LogSignals.egoVelocityData, obj.LogSignals.leadCarSpeed, obj.LogSignals.headway, currentPreferredLane, currentMode, sliderValue);
            
            % Update Planner Parameters.
            updatePlannerParameters(obj, obj.LogSignals.plannerParams, sliderValue);
            
            % Plot trajectories on chase view.
            plotTrajectories(obj, globalTraj, maxTrajectories);

            % Plot capsule list on the chase view.
            plotCapsuleList(obj, futureTrajectory, globalTraj, obj.CapList, obstacleProfiles, egoGeom, egoCar);
        end

        function initializeUIConrolForPostAnalysis(obj, numSimSteps)
            % initializeUIConrolForPostAnalysis initializes the uiLabels, 
            % text fields, and tables required for the displaying the on
            % MATLAB figure.
            
            %% Define grid for slider and trajectory information table
            panel = uipanel(obj.GridLayout);
            panel.Layout.Row = 3;
            panel.Layout.Column = [1 2];
            panel.HighlightColor = obj.HFigure.Color;

            % Create GridLayout6
            trajSliderGrid = uigridlayout(panel);
            trajSliderGrid.ColumnWidth = {'1x', '1x', '1x'};
            trajSliderGrid.RowHeight = {'1x', '0.5x', '5x'};
            trajSliderGrid.Padding = [0 0 0 0];
            trajSliderGrid.RowSpacing = 2;

            % Define column names for tables.
            trajectoryInfoColumnNames = {'Mode','Time','Long Dist','Long Velocity','Lat Dist','Max Accel','Max Kapa', 'Max YawRate','Cost','Valid','Eval Status', 'Collision'};
            mioTablecolumNames = {'MIO lane','TTC','Relative Velocity','Relative Dist','Front','Safe'};
            
            % Initialize trajectory information panel.
            trajInfoPanel = uipanel(trajSliderGrid,'Title','Trajectory Information');
            trajInfoPanel.Layout.Row = 3;
            trajInfoPanel.Layout.Column = [1 3];
            
            % Initialize trajectory information table.
            obj.TrajectoryInfoTable = uitable('Parent',trajInfoPanel,'Units','normalized','Position',[0 0 1 1]);
            obj.TrajectoryInfoTable.ColumnName = trajectoryInfoColumnNames;

            % Create Slider
            obj.HSlider = uislider(trajSliderGrid,'value',1, 'Limits',[1,numSimSteps], 'MajorTicks',1:10:numSimSteps);
            obj.HSlider.Layout.Row = 1;
            obj.HSlider.Layout.Column = [1 2];
            if numSimSteps>200
                obj.HSlider.MajorTicks = round(linspace(1,numSimSteps,20));
            end
            
            % Create label for slider
            sliderLabel = uilabel(trajSliderGrid,'Text','Samples','HorizontalAlignment', 'center', 'FontWeight','bold');
            sliderLabel.Layout.Row = 2;
            sliderLabel.Layout.Column = [1 2];
            
            %% Initialize tables for MIO information
            mioInfoPanel = uipanel(obj.GridLayout,'Title','MIO Information');
            mioInfoPanel.Layout.Row = 1;
            mioInfoPanel.Layout.Column = 2;

            % Create UITable
            obj.MIOTable = uitable(mioInfoPanel,'ColumnWidth',{'1x','1x','1x','1x','1x','1x'},'Units','normalized','Position', [0 0 1 1]);

            % Set the column width and column names for MIO table.
            obj.MIOTable.ColumnName = mioTablecolumNames;

            %% Initialize Mode, Ego Velocity, Simulation Step and Simulation Time.
            % Create gridLayout
            simParamsGrid = uigridlayout(obj.GridLayout);
            simParamsGrid.ColumnWidth = {'1x', '1x', '1x'};
            simParamsGrid.ColumnSpacing = 2;
            simParamsGrid.RowSpacing = 2;
            simParamsGrid.Padding = [0 0 0 0];
            simParamsGrid.Layout.Row = 2;
            simParamsGrid.Layout.Column = 2;

            % Create panel for ego velocity
            egoSpeedPanel = uipanel(simParamsGrid,'BackgroundColor','#FFCC00');
            egoSpeedPanel.Layout.Row = 1;
            egoSpeedPanel.Layout.Column = 2;

            % Create egoSpeedGrid
            egoSpeedGrid = uigridlayout(egoSpeedPanel);
            egoSpeedGrid.ColumnWidth = {'1x'};
            egoSpeedGrid.RowHeight = {'1x', '2x'};
            egoSpeedGrid.RowSpacing = 5;
            egoSpeedGrid.Padding = [0 0 0 0];
            egoSpeedGrid.BackgroundColor = [0.9294 0.6941 0.1255];

            % Create egoSpeedLabel
            egoSpeedLabel = uilabel(egoSpeedGrid,'Text','Ego Velocity','FontWeight','bold','FontSize',10,'HorizontalAlignment', 'center');
            % Create egoSpeedEditField
            obj.EgoSpeedEditField = uilabel(egoSpeedGrid,'Text',strcat(num2str(0,'%0.1f'), ' m/s'),'FontWeight','bold','FontSize',14,'HorizontalAlignment', 'center');

            % Create currentStepPanel
            currentStepPanel = uipanel(simParamsGrid);
            currentStepPanel.Layout.Row = 1;
            currentStepPanel.Layout.Column = 3;

            % Create currentStepGrid
            currentStepGrid = uigridlayout(currentStepPanel);
            currentStepGrid.ColumnWidth = {'1x'};
            currentStepGrid.RowHeight = {'1x', '2x'};
            currentStepGrid.RowSpacing = 5;
            currentStepGrid.Padding = [0 0 0 0];
            currentStepGrid.BackgroundColor = '#FFCC00';

            % Create SimulationStepLabel
            currentStepLabel = uilabel(currentStepGrid,'Text','Simulation Step','FontWeight','bold','FontSize',10,'HorizontalAlignment', 'center');
            % Create currentStepEditField
            obj.CurrentStepEditField = uilabel(currentStepGrid,'Text','0','FontWeight','bold','FontSize',14,'HorizontalAlignment', 'center');

            % Create currentTimePanel
            currentTimePanel = uipanel(simParamsGrid);
            currentTimePanel.Layout.Row = 2;
            currentTimePanel.Layout.Column = 2;

            % Create currentTimeGrid
            currentTimeGrid = uigridlayout(currentTimePanel);
            currentTimeGrid.ColumnWidth = {'1x'};
            currentTimeGrid.RowHeight = {'1x', '2x'};
            currentTimeGrid.RowSpacing = 5;
            currentTimeGrid.Padding = [0 0 0 0];
            currentTimeGrid.BackgroundColor = '#ABC8D1';

            % Create SimulationTimeLabel
            currentTimeLabel = uilabel(currentTimeGrid,'Text','Simulation Time','FontWeight','bold','FontSize',10,'HorizontalAlignment', 'center');
            % Create currentTimeEditField
            obj.CurrentTimeEditField = uilabel(currentTimeGrid,'Text',strcat(num2str(0.1),'s'),'FontWeight','bold','FontSize',14,'HorizontalAlignment', 'center');

            % Create Panel_6
            egoModePanel = uipanel(simParamsGrid);
            egoModePanel.BackgroundColor = [0.9294 0.6941 0.1255];
            egoModePanel.Layout.Row = 1;
            egoModePanel.Layout.Column = 1;

            % Create GridLayout5
            egoModeGrid = uigridlayout(egoModePanel);
            egoModeGrid.ColumnWidth = {'1x'};
            egoModeGrid.RowHeight = {'1x', '2x'};
            egoModeGrid.RowSpacing = 5;
            egoModeGrid.Padding = [0 0 0 0];
            egoModeGrid.BackgroundColor = '#FFCC00';

            % Create egoModeLabel
            egoModeLabel = uilabel(egoModeGrid,'Text','Mode','FontWeight','bold','FontSize',10,'HorizontalAlignment', 'center');
            % Create CCLabel
            obj.EgoModeEditField = uilabel(egoModeGrid,'Text','None','FontWeight','bold','FontSize',14,'HorizontalAlignment', 'center');

            %% Initialize labels and text fields for planner parameter
            plannerParamsPanel = uipanel(obj.GridLayout,'Title','Planner  Parameters');
            plannerParamsPanel.Layout.Row = [2 3];
            plannerParamsPanel.Layout.Column = 3;

            % Create GridLayout4
            plannerParamsGrid = uigridlayout(plannerParamsPanel,...
                'RowHeight',{'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'},...
                'RowSpacing',1);

            timeResolutionLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','TimeResolution');
            obj.PlannerParamsStruct.timeResolutionEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' sec'));
            
            replanRateLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','ReplanRate');
            obj.PlannerParamsStruct.replanRateEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' sec'));
            
            timeHorizonLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','TimeHorizon');
            obj.PlannerParamsStruct.timeHorizonEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' sec'));
            
            preferredLaneLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','PreferredLane');
            obj.PlannerParamsStruct.preferredLaneEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',num2str(0));
            
            setSpeedLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','SetSpeed');
            obj.PlannerParamsStruct.setSpeedEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m/s'));
            
            egoFrontExtLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','EgoFrntExt');
            obj.PlannerParamsStruct.egoFrontExtEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m'));
            
            targetFrontExtLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','TargetFrntExt');
            obj.PlannerParamsStruct.targetFrontExtEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m'));
            
            frontSafetyGapLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','FrontSafetyGap');
            obj.PlannerParamsStruct.frontSafetyGapEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m'));
            
            rearSafetyGapLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','RearSafetyGap');
            obj.PlannerParamsStruct.rearSafetyGapEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m'));
            
            egoTTCLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','egoLaneTTC');
            obj.PlannerParamsStruct.egoTTCEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' sec'));
            
            nextTTCLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','nextLaneTTC');
            obj.PlannerParamsStruct.nextTTCEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' sec'));
            
            latDevWeightLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','LatDevWeight');
            obj.PlannerParamsStruct.latDevWeightEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',num2str(0));
            
            timeWeightLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','TimeWeight');
            obj.PlannerParamsStruct.timeWeightEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',num2str(0));
            
            speedWeightLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','SpeedWeight');
            obj.PlannerParamsStruct.speedWeightEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',num2str(0));
            
            maxAccelLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','MaxAccel');
            obj.PlannerParamsStruct.maxAccelEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m/s^2'));
            
            maxCurvatureLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','MaxCurvature');
            obj.PlannerParamsStruct.maxCurvatureEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',num2str(0));
            
            maxYawRateLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','MaxYawRate');
            obj.PlannerParamsStruct.maxYawRateEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' rad/sec'));
            
            minVelocityLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','MinVelocity');
            obj.PlannerParamsStruct.minVelocityEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text',strcat(num2str(0),' m/s'));
            
            currentModeLabel = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','Behaviors Enabled');
            obj.PlannerParamsStruct.currentModeEditField = uilabel(plannerParamsGrid,'HorizontalAlignment','center','Text','UnKnown');
            
            %% Initialize labels and text fields for ego and lead car states
            % Create egoStatePanel
            egoStatePanel = uipanel(obj.GridLayout,'Title', 'Ego State');
            egoStatePanel.Layout.Row = 1;
            egoStatePanel.Layout.Column = 3;

            % Create egoStateGrid
            egoStateGrid = uigridlayout(egoStatePanel,...
                'RowHeight',{'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'},...
                'RowSpacing',1, 'Padding', [10 0 10 0]);

            egoACCLabel = uilabel(egoStateGrid,'Text','Ego Accel','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoACCEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0,'%0.2f'),' m/s^2'),'HorizontalAlignment','center');
            
            egoCurvLabel = uilabel(egoStateGrid,'Text','Ego Curvature','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoCurvEditField = uilabel(egoStateGrid,'Text',num2str(0,'%0.2f'),'HorizontalAlignment','center');
            
            egoYawRateLabel = uilabel(egoStateGrid,'Text','Ego YawRate','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoYawRateEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0,'%0.2f'),' deg/s'),'HorizontalAlignment','center');
            
            egoSetSpeedLabel = uilabel(egoStateGrid,'Text','Current Set Speed','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoSetSpeedEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0, '%0.2f'),' m/s'),'HorizontalAlignment','center');
            
            egoCurrentLaneLabel = uilabel(egoStateGrid,'Text','Current Ego Lane','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoCurrentLaneEditField = uilabel(egoStateGrid,'Text',num2str(0),'HorizontalAlignment','center');
            
            egoGoalLaneLabel = uilabel(egoStateGrid,'Text','Goal Lane','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoGoalLaneEditField = uilabel(egoStateGrid,'Text',num2str(0),'HorizontalAlignment','center');
            
            egoLeadSpeedLabel = uilabel(egoStateGrid,'Text','Lead Car Speed','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.egoLeadSpeedEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0),' m/s'),'HorizontalAlignment','center');
            
            egoHeadwayLabel = uilabel(egoStateGrid,'Text','Headway to lead car','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.headwayToLeadCarEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0,'%0.2f'),' m'),'HorizontalAlignment','center');
            
            egoTtcLabel = uilabel(egoStateGrid,'Text','Lead Car TTC','HorizontalAlignment','center');
            obj.EgoStateParamsStruct.leadCarTTTCEditField = uilabel(egoStateGrid,'Text',strcat(num2str(0,'%0.2f'),' sec'),'HorizontalAlignment','center');
        end
        
        function [globalTrajectories, futureTrajectories, egoActor, targetActorsInWorld]= recreateDataForPlotting(obj, maxTrajectories, maxTrajectoryPoints, egoPoseData, targetActors, numTargetActors, idx)
            % recreateDataForPlotting helps in recreating structure for 
            % plotting the trajectories, capsule list and actors on chase
            % view. It extracts data at the provided time step from the
            % logged data. 
            %
            % It takes below parameters as intputs:
            %   maxTrajectories     - Maximum number of trajectories.
            %   maxTrajectoryPoints - Maximum number of points in trajectory.
            %   egoPoseData         - Logged ego vehicle pose information.
            %   targetActors        - Logged target actor pose information.
            %   numTargetActors     - Number of target actors.
            %   idx                 - Positon of the thumb on the slider.
            % 
            % It returns current global trajectories, future trajectories,
            % ego actor pose and target actor pose.

            % Get global trajectories information
            isNew = reshape(obj.VisualizationInfo.TrajectoryInfo.IsNew.Data, [], size(obj.VisualizationInfo.TrajectoryInfo.IsNew,1));
            optimalTrajectoryIndex = reshape(obj.VisualizationInfo.TrajectoryInfo.OptimalTrajectoryIndex.Data, [], size(obj.VisualizationInfo.TrajectoryInfo.OptimalTrajectoryIndex,1));
            numMIOTrajs = reshape(obj.VisualizationInfo.FutureTrajectories.NumTrajs.Data, size(obj.VisualizationInfo.FutureTrajectories.NumTrajs.Data,1), []);
            targetIDsVec = reshape(obj.VisualizationInfo.FutureTrajectories.TargetIDs.Data, size(obj.VisualizationInfo.FutureTrajectories.TargetIDs.Data,1), []);
            numTrajectories = reshape(obj.VisualizationInfo.TrajectoryInfo.NumTrajectories.Data, [], size(obj.VisualizationInfo.TrajectoryInfo.NumTrajectories,1));
            
            for j=1:maxTrajectories
                tempTimes= reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).Times.Data, maxTrajectoryPoints,size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).Times,1),[]);
                numTrajPoints = reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).NumTrajPoints.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).NumTrajPoints,1), []);
                isValid = reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsValid.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsValid,1), []);
                isEvaluated = reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsEvaluated.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsEvaluated,1), []);
                isColliding = reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsColliding.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).IsColliding,1), []);
                behaviorType = reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).BehaviorType.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).BehaviorType,1), []);
                Trajectory = reshape(reshape(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).Trajectory.Data, size(obj.VisualizationInfo.TrajectoryInfo.GlobalTrajectory(j).Trajectory,1),maxTrajectoryPoints,6, []),maxTrajectoryPoints,6,[]);
                globalTrajectories.GlobalTrajectory(j).Times(1:maxTrajectoryPoints,:) = tempTimes(:,:,idx);
                globalTrajectories.GlobalTrajectory(j).NumTrajPoints = numTrajPoints(:,idx);
                globalTrajectories.GlobalTrajectory(j).IsValid = isValid(:,idx);
                globalTrajectories.GlobalTrajectory(j).IsEvaluated = isEvaluated(:,idx);
                globalTrajectories.GlobalTrajectory(j).IsColliding = isColliding(:,idx);
                globalTrajectories.GlobalTrajectory(j).BehaviorType = behaviorType(:,idx);
                globalTrajectories.GlobalTrajectory(j).Trajectory = Trajectory(:,:,idx);
            end
            globalTrajectories.NumTrajectories = numTrajectories(idx,:);
            globalTrajectories.IsNew = isNew(idx,:);
            globalTrajectories.OptimalTrajectoryIndex = optimalTrajectoryIndex(idx,:);
            
            % Get future trajectories information
            for j=1:numMIOTrajs(idx,:)
                futureTraj = reshape(reshape(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).Trajectory.Data, size(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).Trajectory.Data,1),6, []), maxTrajectoryPoints,6,[]);
                numpts = reshape(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).NumPts.Data, [],size(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).NumPts.Data,1));
                targetIDArray = reshape(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).TargetID.Data, [], size(obj.VisualizationInfo.FutureTrajectories.Trajectories(j).TargetID.Data,1));
                futureTrajectories.Trajectories(j).TargetID = targetIDArray(:,idx);
                futureTrajectories.Trajectories(j).NumPts = numpts(:,idx);
                futureTrajectories.Trajectories(j).Trajectory = futureTraj(:,:,idx);
            end
            futureTrajectories.NumTrajs = numMIOTrajs(idx,:);
            futureTrajectories.TargetIDs = targetIDsVec(:,idx);
            
            % Get ego actor information.
            actor.ActorID =  egoPoseData.Values.ActorID.Data(:,:,idx);
            actor.Position = egoPoseData.Values.Position.Data(:,:,idx);
            actor.Velocity = egoPoseData.Values.Velocity.Data(:,:,idx);
            actor.Roll = egoPoseData.Values.Roll.Data(:,:,idx);
            actor.Pitch = egoPoseData.Values.Pitch.Data(:,:,idx);
            actor.Yaw = egoPoseData.Values.Yaw.Data(:,:,idx);
            actor.AngularVelocity = egoPoseData.Values.AngularVelocity.Data(:,:,idx);
            egoActor = actor;
            
            % Get target actors information from logged data.
            targetActorsInWorld.NumActors = targetActors.Values.NumActors.Data(:,:,idx);
            targetActorsInWorld.Time = targetActors.Values.Time.Data(:,:,idx);
            for actorId=1:numTargetActors
                targetActorsInWorld.Actors(actorId).ActorID = targetActors.Values.Actors(actorId).ActorID.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).Position = targetActors.Values.Actors(actorId).Position.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).Velocity = targetActors.Values.Actors(actorId).Velocity.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).Roll = targetActors.Values.Actors(actorId).Roll.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).Pitch = targetActors.Values.Actors(actorId).Pitch.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).Yaw = targetActors.Values.Actors(actorId).Yaw.Data(:,:,idx);
                targetActorsInWorld.Actors(actorId).AngularVelocity = targetActors.Values.Actors(actorId).AngularVelocity.Data(:,:,idx);
            end
        end
        
        function updateTrajectoryInfoTable(~, trajectoryInfoTable, terminalStates, globalTrajectories, idx)
            % updateTrajectoryInfoTable helps in updating the trajectory
            % information to the uitable of the MATLAB figure.
            %
            % It takes table handle, terminal states, global trajectories
            % and slider values as input.
            
            % Read simulation data to update the table.
            numStates = reshape(terminalStates.Values.NumCombinations.Data, size(terminalStates.Values.NumCombinations.Data,1), []);
            combinations = reshape(terminalStates.Values.Combinations.Data, size(terminalStates.Values.Combinations.Data,1),7,[]);
            mode = reshape(terminalStates.Values.BehaviorType.Data, size(terminalStates.Values.BehaviorType.Data,1), []);
            cost = reshape(terminalStates.Values.Cost.Data, size(terminalStates.Values.Cost.Data,1), []);
            
            % Convert behavior type to string.
            modeString = cell(numStates(idx),1);
            terminalStatesMode = mode(1:numStates(idx),idx);
            for j=1:numStates(idx)
                if terminalStatesMode(j) == BehaviorType.CruiseControl
                    modeString{j}='CC';
                elseif terminalStatesMode(j) == BehaviorType.LeadCarFollowing
                    modeString{j}='LCF';
                elseif terminalStatesMode(j) == BehaviorType.LaneChange
                    modeString{j}='LC';
                else
                    modeString{j}='Unknown';
                end
            end
            
            tabTrajectoryInfo= cell(numStates(idx),12);
            
            % Compute data from trajectory information to update in the table.
            maxYawRate = zeros(numStates(idx),1);
            maxAcc = zeros(numStates(idx),1);
            maxKapa = zeros(numStates(idx),1);
            validity = zeros(numStates(idx),1);
            evaluation = zeros(numStates(idx),1);
            collision = zeros(numStates(idx),1);
            
            numTrajectories = globalTrajectories.NumTrajectories;
            backgroundColor = zeros(numTrajectories,3);
            for j=1:numTrajectories 
                numTrajPoints = globalTrajectories.GlobalTrajectory(j).NumTrajPoints;
                
                % Get the absolute value of curvature.
                curvature = abs(globalTrajectories.GlobalTrajectory(j).Trajectory(1:numTrajPoints,4));
                
                % Get the velocity of trajectory.
                velocity = globalTrajectories.GlobalTrajectory(j).Trajectory(1:numTrajPoints,5);
                
                % Compute yaw-rate.
                yawrate = curvature.*abs(velocity);
                
                % Find the maximum values of curvature, yaw-rate and acceleration.    
                maxAcc(j,:) = max(abs(globalTrajectories.GlobalTrajectory(j).Trajectory(1:numTrajPoints,6)));
                maxKapa(j,:) = max(curvature);
                maxYawRate(j,:) = max(yawrate);
                validity(j,:) = globalTrajectories.GlobalTrajectory(j).IsValid;
                evaluation(j,:) = globalTrajectories.GlobalTrajectory(j).IsEvaluated;
                collision(j,:) = globalTrajectories.GlobalTrajectory(j).IsColliding;
                
                % Update trajectory color.
                if j == globalTrajectories.OptimalTrajectoryIndex
                    backgroundColor(j,:) = [0 1 0];
                else
                    if globalTrajectories.GlobalTrajectory(j).IsValid
                    if globalTrajectories.GlobalTrajectory(j).IsColliding && globalTrajectories.GlobalTrajectory(j).IsEvaluated
                        backgroundColor(j,:) = [1 0 0];
                    elseif ~globalTrajectories.GlobalTrajectory(j).IsEvaluated
                        backgroundColor(j,:) = [1 1 1];
                    end
                    else
                        backgroundColor(j,:) = [0 1 1];
                    end       
                end
            end
            trajectoryInfoTable.RowStriping = 'on';
            trajectoryInfoTable.BackgroundColor = backgroundColor;
            
            
            % Update the cell array with information.
            tabTrajectoryInfo(:,1) = modeString;
            tabTrajectoryInfo(:,2) = num2cell(combinations(1:numStates(idx),7));
            tabTrajectoryInfo(:,3:5) = num2cell(combinations(1:numStates(idx),[1,2,4]));
            tabTrajectoryInfo(:,6) = num2cell(maxAcc(1:numStates(idx),:));
            tabTrajectoryInfo(:,7) = num2cell(maxKapa(1:numStates(idx),:));
            tabTrajectoryInfo(:,8) = num2cell(maxYawRate(1:numStates(idx),:));
            tabTrajectoryInfo(:,9) = num2cell(cost(1:numStates(idx),idx));
            tabTrajectoryInfo(:,10) = num2cell(validity(1:numStates(idx),:));
            tabTrajectoryInfo(:,11) = num2cell(evaluation(1:numStates(idx),:));
            tabTrajectoryInfo(:,12) = num2cell(collision(1:numStates(idx),:));
            
            % Update trajectoryInfoTable with the computed information.
            trajectoryInfoTable.Data = tabTrajectoryInfo;
        end

        function updateEgoStateInformation(obj, egoCurrentLane, refPointOnPath, egoVelocityData, leadCarSpeed, headway, preferredLane, currentMode, idx)
            % updateEgoStateInformation helps in updating the ego state
            % information to the MATLAB figure.
            %
            % It takes below parameters as intputs:
            %   egoCurrentLane  - Current ego lane information.
            %   refPointOnPath  - Reference point of the ego.
            %   egoVelocityData - Ego velocity information.
            %   leadCarSpeed    - Speed of lead vehicle.
            %   headway         - Headway to lead vehicle.
            %   preferredLane   - Ego preferred lane.
            %   currentMode     - Ego vehicle mode.
            %   idx             - Slider value.
            
            % Get required data.
            egoLane          = reshape(egoCurrentLane.Values.Data, size(egoCurrentLane.Values.Data,1),[]);
            egoSetSpeed      = reshape(refPointOnPath.Values.RefVelocity.Data, size(refPointOnPath.Values.RefVelocity.Data,1),[]);
            egoAcc           = reshape(refPointOnPath.Values.RefAccel.Data, size(refPointOnPath.Values.RefAccel.Data,1),[]);
            egoCurvature     = reshape(refPointOnPath.Values.RefCurvature.Data, size(refPointOnPath.Values.RefCurvature.Data,1),[]);
            egoVelocity      = reshape(egoVelocityData.Values.Data, size(egoVelocityData.Values.Data,1),[]);
            leadVehicleSpeed = reshape(leadCarSpeed.Values.Data, size(leadCarSpeed.Values.Data,1),[]);
            leadVehicleDist  = reshape(headway.Values.Data, size(headway.Values.Data,1),[]);
            egoYawRate       = egoCurvature(idx).*egoVelocity(idx);
            leadVehicleTTC   = leadVehicleDist(idx)/leadVehicleSpeed(idx);
            
            %% Update ego and lead vehicle information to the figure.
            % Update ego velocity.
            obj.EgoSpeedEditField.Text = strcat(num2str(egoVelocity(idx),'%0.1f'), ' m/s');
            
            % Update ego acceleration.
            obj.EgoStateParamsStruct.egoACCEditField.Text = strcat(num2str(egoAcc(idx),'%0.2f'),' m/s^2');
            
            % Update ego curvature.
            obj.EgoStateParamsStruct.egoCurvEditField.Text = num2str(egoCurvature(idx),'%0.2f');
            
            % Update ego yaw rate.
            obj.EgoStateParamsStruct.egoYawRateEditField.Text = strcat(num2str(egoYawRate,'%0.2f'),' deg/s');
            
            % Update set speed.
            obj.EgoStateParamsStruct.egoSetSpeedEditField.Text = strcat(num2str(egoSetSpeed(idx), '%0.2f'),' m/s');
            
            % Update ego lane number.
            obj.EgoStateParamsStruct.egoCurrentLaneEditField.Text = num2str(egoLane(idx));
            
            % Update goal lane.
            obj.EgoStateParamsStruct.egoGoalLaneEditField.Text = num2str(preferredLane(idx));
            
            % Update current mode.
            if currentMode == BehaviorType.CruiseControl
                egoModeText = 'CC';
            elseif currentMode == BehaviorType.LeadCarFollowing
                egoModeText = 'LCF';
            elseif currentMode == BehaviorType.LaneChange
                egoModeText = 'LC';
            else
                egoModeText = 'Unknown';
            end
            
            obj.EgoModeEditField.Text = egoModeText;
            
            % Update lead vehicle speed.
            obj.EgoStateParamsStruct.egoLeadSpeedEditField.Text = strcat(num2str(leadVehicleSpeed(idx) + egoVelocity(idx)),' m/s');
            
            % Update distance to lead vehicle.
            obj.EgoStateParamsStruct.headwayToLeadCarEditField.Text = strcat(num2str(leadVehicleDist(idx),'%0.2f'),' m');
            
            % Update TTC to lead vehicle.
            obj.EgoStateParamsStruct.leadCarTTTCEditField.Text = strcat(num2str(abs(leadVehicleTTC),'%0.2f'),' sec');
        end

        function updatePlannerParameters(obj, plannerParams, idx)
            % updatePlannerParameters helps in updating the planner
            % parameters information based on the log data.
               
            % Get planner parameters information
            timeResolution    = reshape(plannerParams.Values.TimeResolution.Data, size(plannerParams.Values.TimeResolution.Data,1),[]);
            replanRate        = reshape(plannerParams.Values.ReplanRate.Data, size(plannerParams.Values.ReplanRate.Data,1),[]);
            timeHorizon       = reshape(reshape(plannerParams.Values.TimeHorizon.Data, size(plannerParams.Values.TimeHorizon.Data,1),[]),size(plannerParams.Values.TimeHorizon.Data,2),[]);
            preferredLane     = reshape(plannerParams.Values.PreferredLane.Data, size(plannerParams.Values.PreferredLane.Data,1),[]);
            setSpeed          = reshape(plannerParams.Values.SetSpeed.Data, size(plannerParams.Values.SetSpeed.Data,1),[]);
            egoFrontExt       = reshape(plannerParams.Values.EgoFrontExt.Data, size(plannerParams.Values.EgoFrontExt.Data,1),[]);
            targetFrontExt    = reshape(plannerParams.Values.TargetFrontExt.Data, size(plannerParams.Values.TargetFrontExt.Data,1),[]);
            frontSafetyGap    = reshape(plannerParams.Values.FrontSafetyGap.Data, size(plannerParams.Values.FrontSafetyGap.Data,1),[]);
            rearSafetyGap     = reshape(plannerParams.Values.RearSafetyGap.Data, size(plannerParams.Values.RearSafetyGap.Data,1),[]);
            egoTTC            = reshape(plannerParams.Values.EgoTTC.Data, size(plannerParams.Values.EgoTTC.Data,1),[]);
            nextTTC           = reshape(plannerParams.Values.NextTTC.Data, size(plannerParams.Values.NextTTC.Data,1),[]);
            latDevWeight      = reshape(plannerParams.Values.LatDevWeight.Data, size(plannerParams.Values.LatDevWeight.Data,1),[]);
            timeWeight        = reshape(plannerParams.Values.TimeWeight.Data, size(plannerParams.Values.TimeWeight.Data,1),[]);
            speedWeight       = reshape(plannerParams.Values.SpeedWeight.Data, size(plannerParams.Values.SpeedWeight.Data,1),[]);
            maxAccel          = reshape(plannerParams.Values.MaxAccel.Data, size(plannerParams.Values.MaxAccel.Data,1),[]);
            maxCurvature      = reshape(plannerParams.Values.MaxCurvature.Data, size(plannerParams.Values.MaxCurvature.Data,1),[]);
            maxYawRateParam   = reshape(plannerParams.Values.MaxYawRate.Data, size(plannerParams.Values.MaxYawRate.Data,1),[]);
            minVelocity       = reshape(plannerParams.Values.MinVelocity.Data, size(plannerParams.Values.MinVelocity.Data,1),[]);
            enableCCBehavior  = reshape(plannerParams.Values.EnableCCBehavior.Data, size(plannerParams.Values.EnableCCBehavior.Data,1),[]);
            enableLCFBehavior = reshape(plannerParams.Values.EnableLCFBehavior.Data, size(plannerParams.Values.EnableLCFBehavior.Data,1),[]);
            enableLCBehavior  = reshape(plannerParams.Values.EnableLCBehavior.Data, size(plannerParams.Values.EnableLCBehavior.Data,1),[]);
            
            %% Update planner parameters in figure.
            obj.PlannerParamsStruct.timeResolutionEditField.Text = strcat(num2str(timeResolution(idx)),' sec');
            
            obj.PlannerParamsStruct.replanRateEditField.Text = strcat(num2str(replanRate(idx)),' sec');
            
            obj.PlannerParamsStruct.timeHorizonEditField.Text = strcat(num2str(timeHorizon(:,idx)'),' sec');
            
            obj.PlannerParamsStruct.preferredLaneEditField.Text = num2str(preferredLane(idx));
            
            obj.PlannerParamsStruct.setSpeedEditField.Text = strcat(num2str(setSpeed(idx)),' m/s');
            
            obj.PlannerParamsStruct.egoFrontExtEditField.Text = strcat(num2str(egoFrontExt(idx)),' m');
            
            obj.PlannerParamsStruct.targetFrontExtEditField.Text = strcat(num2str(targetFrontExt(idx)),' m');
            
            obj.PlannerParamsStruct.frontSafetyGapEditField.Text = strcat(num2str(frontSafetyGap(idx)),' m');
            
            obj.PlannerParamsStruct.rearSafetyGapEditField.Text = strcat(num2str(rearSafetyGap(idx)),' m');
            
            obj.PlannerParamsStruct.egoTTCEditField.Text = strcat(num2str(egoTTC(idx)),' sec');
            
            obj.PlannerParamsStruct.nextTTCEditField.Text = strcat(num2str(nextTTC(idx)),' sec');
            
            obj.PlannerParamsStruct.latDevWeightEditField.Text = num2str(latDevWeight(idx));
            
            obj.PlannerParamsStruct.timeWeightEditField.Text = num2str(timeWeight(idx));
            
            obj.PlannerParamsStruct.speedWeightEditField.Text = num2str(speedWeight(idx));
            
            obj.PlannerParamsStruct.maxAccelEditField.Text = strcat(num2str(maxAccel(idx)),' m/s^2');
            
            obj.PlannerParamsStruct.maxCurvatureEditField.Text = num2str(maxCurvature(idx));
            
            obj.PlannerParamsStruct.maxYawRateEditField.Text = strcat(num2str(maxYawRateParam(idx)),' rad/sec');
            
            obj.PlannerParamsStruct.minVelocityEditField.Text = strcat(num2str(minVelocity(idx)),' m/s');
            
            % Convert the mode to string
            currentMode = '';
            if enableCCBehavior
                currentMode = currentMode + "CC,";
            end
            if enableLCFBehavior
                currentMode = currentMode + "LCF,";
            end
            if enableLCBehavior
                currentMode = currentMode + "LC";
            end
            if ~enableCCBehavior(idx) && ~enableLCFBehavior(idx) && ~enableLCBehavior(idx)
                currentMode = 'None';
            end
            
            obj.PlannerParamsStruct.currentModeEditField.Text = currentMode;
        end

        function plotTrajectories(obj, globalTraj, maxTrajectories)
            % plotTrajectories helps in plotting sampled trajectories on
            % chase view.
            %
            % It takes ego global trajectory and maximum number of
            % trajectories as inputs.
            
            % Get number of trajectories
            numTrajectories = globalTraj.NumTrajectories;
            
            % Get Optimal Trajectory index
            OptimalTrajIdx = uint8(globalTraj.OptimalTrajectoryIndex);
            
            % Plot optimal trajectory in green color
            if ~isempty(OptimalTrajIdx)
                widthChase = 4;
                color = 'g';
                lineStyle = '-';
                traj = globalTraj.GlobalTrajectory(OptimalTrajIdx).Trajectory;
                numTrajPts = globalTraj.GlobalTrajectory(OptimalTrajIdx).NumTrajPoints;
                set(obj.LineHandles(OptimalTrajIdx),'XData',traj(1:numTrajPts,1),'YData',traj(1:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',widthChase);
            else
                set(obj.LineHandles(OptimalTrajIdx),'XData',[],'YData',[]);
            end
            
            % Update color of sampled trajectory based of trajectory info
            % and plot them on both top view and chase view.
            widthChase = 1;
            validTrajIdx = 0;
            for k = 1:maxTrajectories
                if k <= numTrajectories
                    traj = globalTraj.GlobalTrajectory(k).Trajectory;
                    numTrajPts = globalTraj.GlobalTrajectory(k).NumTrajPoints;
                    if globalTraj.GlobalTrajectory(k).IsValid
                        validTrajIdx = validTrajIdx + 1;
                        lineStyle = '-';
                        if globalTraj.GlobalTrajectory(k).IsColliding && globalTraj.GlobalTrajectory(k).IsEvaluated
                            % Trajectory with collision
                            color = 'r';
                        elseif ~globalTraj.GlobalTrajectory(k).IsEvaluated
                            % Trajectory did not get evaluated
                            color = 'k';
                        end
                    else
                        % Trajectory violated a constraint, and therefore was never
                        % evaluated
                        lineStyle = '--';
                        color = 'c';
                    end
                    if isempty(OptimalTrajIdx) || k ~= OptimalTrajIdx
                        set(obj.LineHandles(k),'XData',traj(1:numTrajPts,1),'YData',traj(1:numTrajPts,2),'LineStyle',lineStyle,'Color',color,'LineWidth',widthChase);
                    end
                else
                    % Update sampled trajectories on chase view
                    set(obj.LineHandles(k),'XData',[],'YData',[]);
                end
            end
            
            drawnow
        end

        function plotCapsuleList(obj, futureTrajectory, globalTraj, capList, obstacleProfiles, egoGeom, egoCar)
            % plotCapsuleList helps in plotting capsuleList on chase view.
            %
            % It takes below parameters as intputs:
            %   futureTrajectory- Future ego trajectories.
            %   globalTraj      - Ego global trajectory.
            %   capList         - Ego capusle list.
            %   obstacleProfiles- Actor profiles of all the vehicles.
            %   egoGeom         - Ego vehicle geometry information.
            %   egoCar          - Ego vehicle pose information.
            
            % Get ego actor ID
            EgoActorID = egoCar.ActorID;
            
            % Get Optimal Trajectory index
            OptimalTrajIdx = uint8(globalTraj.OptimalTrajectoryIndex);
            
            % Update target list and predictions for plotting capsule list.
            targetIDs = double(futureTrajectory.TargetIDs(uint8(1):futureTrajectory.NumTrajs));
            actorGeom = repelem(egoGeom(1),futureTrajectory.NumTrajs,1);
            Lia = ismember(capList.ObstacleIDs,targetIDs);
            ReduceIDs = capList.ObstacleIDs(~Lia);
            if ~isempty(ReduceIDs)
                removeObstacle(capList, ReduceIDs);
            end
            
            % Update actor profiles using input vehicle profiles information
            for n=1:numel(targetIDs)
                actorGeom(n).Geometry.Length = obstacleProfiles(targetIDs(n)).Length;
                actorGeom(n).Geometry.Width = obstacleProfiles(targetIDs(n)).Width;
                actorGeom(n).Geometry.Radius = obstacleProfiles(targetIDs(n)).Width/2;
            end
            updateObstacleGeometry(capList, targetIDs, actorGeom);
            
            % Update obstacle poses.
            [~, actorPoses] = obstaclePose(capList, capList.ObstacleIDs);
            for n = uint8(1):futureTrajectory.NumTrajs
                actorPoses(n).States = futureTrajectory.Trajectories(n).Trajectory(uint16(1):futureTrajectory.Trajectories(n).NumPts,1:3);
            end
            % Update the MIOs poses in the capsule list
            updateObstaclePose(capList, targetIDs, actorPoses);
            
            % Update ego pose in to capsule list.
            egoPoses.States = globalTraj.GlobalTrajectory(OptimalTrajIdx).Trajectory(:,1:3);
            updateEgoPose(capList, EgoActorID, egoPoses);
            
            % Plot capsule list on chase view
            hold(obj.HScenViewAxes,'on');
            show(capList,'TimeStep',1:2:capList.MaxNumSteps,'FastUpdate',1,'Parent',obj.HScenViewAxes);
        end

        function updateMIOTable(~, MIOTable, mioInfoData, futureTrajectory, plotColor, idx)
            % updateMIOTable helps in updating the MIO information in
            % MATLAB figure.
            %
            % It takes below parameters as intputs:
            %   MIOTable            - Handle of uitable of MIO information.
            %   mioInfoData         - Logged MIO information.
            %   futureTrajectory    - Future ego trajectories.
            %   plotColor           - Vehicle colors.
            %   idx                 - Slider value.
            
            % Get MIO information
            numMIOS          = reshape(mioInfoData.Values.NumMIOs.Data, size(mioInfoData.Values.NumMIOs.Data,1),[]);
            mioLaneNum       = reshape(mioInfoData.Values.LaneNum.Data, size(mioInfoData.Values.LaneNum.Data,1),[]);
            mioTTC           = reshape(mioInfoData.Values.TTC.Data, size(mioInfoData.Values.TTC.Data,1),[]);
            mioRelativeDist  = reshape(mioInfoData.Values.RelativeDist.Data, size(mioInfoData.Values.RelativeDist.Data,1),[]);
            mioRelativeVelo  = reshape(mioInfoData.Values.RelativeVelo.Data, size(mioInfoData.Values.RelativeVelo.Data,1),[]);
            mioIsSafe        = reshape(mioInfoData.Values.IsSafe.Data, size(mioInfoData.Values.IsSafe.Data,1),[]);
            mioIsFront       = reshape(mioInfoData.Values.IsFront.Data, size(mioInfoData.Values.IsFront.Data,1),[]);
            
            
            if numMIOS(idx) > 0
                tabMIO = cell(numMIOS(idx),6);    
                % Update table
                tabMIO(:,1) = num2cell( mioLaneNum(1:numMIOS(idx),idx)');
                tabMIO(:,2) = num2cell(mioTTC(1:numMIOS(idx),idx)');
                tabMIO(:,3) = num2cell(mioRelativeDist(1:numMIOS(idx),idx)');
                tabMIO(:,4) = num2cell(mioRelativeVelo(1:numMIOS(idx),idx)');
                tabMIO(:,5) = num2cell(mioIsSafe(1:numMIOS(idx),idx)');
                tabMIO(:,6) = num2cell(mioIsFront(1:numMIOS(idx),idx)');
                numTargets = size(plotColor,1);
                mioColor = zeros(futureTrajectory.NumTrajs,3);
                mioIndex = 1;
                for i=1:numTargets
                    index =  i+1 == futureTrajectory.TargetIDs;
                    if any(index)
                        mioColor(mioIndex,:) = plotColor(i,:);
                        mioIndex = mioIndex + 1;
                    end
                end
            else
                mioColor = zeros(1,3);
                tabMIO = cell(1,6);
                tabMIO(:,:) = num2cell(NaN(1,6));
                mioColor(1,:) = ones(1,3);
            end
            
            % Update MIO information in MIO table.
            MIOTable.Data = tabMIO;
            
            % Update color of cells in MIOInfo table
            MIOTable.RowStriping = 'on';
            MIOTable.BackgroundColor = mioColor;
        end

        function figureResizeCallbak(obj,~,~)
            % figureResizeCallbak is a callback function that resizes axes
            % and uitables when figure window is resized.
            obj.HScenViewAxes.OuterPosition = [0 0 1 1];
            obj.TrajectoryInfoTable.Position = [0 0 1 1];
            obj.MIOTable.Position = [0 0 1 1];
        end
    end
end