classdef HelperAnalyzeLCSimulationResults < HelperAnalyzeLCPlannerSimulationResults
%%HelperAnalyzeLCSimulationResults helps in visualizing the 
% simulation results and its corresponding planner information on a MATLAB 
% figure for post simulation analysis of HLC with RRScenario.
%
%   NOTE: This is a helper function for example purposes and
%   may be removed or modified in the future.

%   Copyright 2021-2022 The MathWorks, Inc.

    properties(Access=protected,Hidden=true)
        % Theatre plotter handles.
        HTrackPlotter;
        HRadarDetectionsPlotter;
        HVisionDetectionsPlotter;

        % Enable visualization of tracks and detections.
        Tracks;
        Detections;
        TracksAvailable;
    end

    methods
        function obj = HelperAnalyzeLCSimulationResults(logsout)
            %HelperAnalyzeLCSimulationResults Construct an instance of this
            %class.
            obj@HelperAnalyzeLCPlannerSimulationResults(logsout);
        end
    end
    
    methods(Access=protected)
        function readLogData(obj, logsout)
            %readLogData reads required data from the simulation log. It
            %take logsout as input which contains all the logged signals.
            %It also reads logged signals of tracks and detections.
            
            % Get logged elements.
            readLogData@HelperAnalyzeLCPlannerSimulationResults(obj, logsout);
            
            obj.TracksAvailable = false;
            [~,logsoutIndex] = find(logsout,'Name','target_estimates');
            if isempty(logsoutIndex)
                obj.Tracks = [];
            else
                obj.Tracks = logsout.getElement('target_estimates'); 
                obj.Detections = logsout.getElement('detections');
                obj.TracksAvailable = true;        
            end
        end

        function initializeMATLABFigure(obj)
            %initializeMATLABFigure initializes MATLAB figure and its UI
            %elements.

            % Initialize figure using base class method
            initializeMATLABFigure@HelperAnalyzeLCPlannerSimulationResults(obj);

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
            
            if(obj.TracksAvailable)
            
                hChase(5) = line(obj.HScenViewAxes,0,0,...
                    'LineStyle','none','Marker','o','Color','r','LineWidth',3,'MarkerSize',3);
                set(hChase(5),'XData',[],'YData',[]);
            
                hChase(6) = line(obj.HScenViewAxes,0,0,...
                    'LineStyle','none','Marker','^','Color','b','LineWidth',3,'MarkerSize',3);
                set(hChase(6),'XData',[],'YData',[]);
            
                legend(obj.HScenViewAxes,hChase,...
                    {'optimal','colliding','infeasible','not evaluated','Radar Detections','Vision Detections'},...
                    'Color',[0.94,0.94,0.94],...
                    'AutoUpdate','off',...
                    'Location','bestoutside');
            else
            
            legend(obj.HScenViewAxes,hChase,...
                {'optimal','colliding','infeasible','not evaluated'},...
                'Color',[0.94,0.94,0.94],...
                'AutoUpdate','off',...
                'Location','bestoutside');
            end

            if(obj.TracksAvailable)
                hTheaterPlot = theaterPlot('Parent',obj.HScenViewAxes);
                obj.HTrackPlotter = trackPlotter(hTheaterPlot,'DisplayName','Tracks','HistoryDepth',7);
                obj.HRadarDetectionsPlotter = detectionPlotter(hTheaterPlot,'Marker','o','MarkerFaceColor','r');
                obj.HVisionDetectionsPlotter = detectionPlotter(hTheaterPlot,'Marker','^','MarkerFaceColor','b');
            end
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
            %   sliderValue         - Positon of the thumb on the 
            
            % Retrieve information for plotting and updating from the elements of logged data.
            if(obj.TracksAvailable)
                [~, ~, egoActor, ~,tracksWorld,detectionsData] = ...
                    recreateDataForPlotting(obj, maxTrajectories, maxTrajectoryPoints, obj.LogSignals.egoPoseData, obj.LogSignals.targetActors, numTargetActors, sliderValue,obj.Tracks,obj.Detections);

                [posRadar, posCamera] = parseDetections(obj,detectionsData,egoActor);
                plotDetection(obj.HRadarDetectionsPlotter,posRadar);
                plotDetection(obj.HVisionDetectionsPlotter,posCamera);
                plotTracks(obj, obj.HTrackPlotter,tracksWorld);
            end
            %% Update the planner information and visualization on the figure at initialization.
            
            [egoActor, targetActorsInWorld, futureTrajectory] = ...
                updateSimulationData@HelperAnalyzeLCPlannerSimulationResults(obj, obstacleProfiles, maxTrajectories, egoCar, maxTrajectoryPoints, numTargetActors, egoGeom, sliderValue);
        end

        function [globalTrajectories, futureTrajectories, egoActor, targetActorsInWorld,tracksWorld, detectionsData]= recreateDataForPlotting(obj, maxTrajectories, maxTrajectoryPoints, egoPoseData, targetActors, numTargetActors, idx,varargin)
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
            %   varargin            - Takes tracks and detections as an
            %                       additional inputs.
            % 
            % It returns current global trajectories, future trajectories,
            % ego actor pose target actor pose, tracks and detection
            % information.

            tracksWorld = [];
            detectionsData = [];
            if(nargin == 9)
                tracks = varargin{1};
                detections = varargin{2};
                tracksAvailable = true;
            else
                tracksAvailable = false;
            end
            
            [globalTrajectories, futureTrajectories, egoActor, targetActorsInWorld]= recreateDataForPlotting@HelperAnalyzeLCPlannerSimulationResults(obj, maxTrajectories, maxTrajectoryPoints, egoPoseData, targetActors, numTargetActors, idx);
            
            if(tracksAvailable)
                tracksWorld.NumActors = tracks.Values.NumActors.Data(:,:,idx);
                tracksWorld.Time = tracks.Values.Time.Data(:,:,idx);
                for actorId=1:size(tracks.Values.Actors,1)
                    tracksWorld.Actors(actorId).ActorID = tracks.Values.Actors(actorId).ActorID.Data(:,:,idx);
                    tracksWorld.Actors(actorId).Position = tracks.Values.Actors(actorId).Position.Data(:,:,idx);
                    tracksWorld.Actors(actorId).Velocity = tracks.Values.Actors(actorId).Velocity.Data(:,:,idx);
                    tracksWorld.Actors(actorId).Roll = tracks.Values.Actors(actorId).Roll.Data(:,:,idx);
                    tracksWorld.Actors(actorId).Pitch = tracks.Values.Actors(actorId).Pitch.Data(:,:,idx);
                    tracksWorld.Actors(actorId).Yaw = tracks.Values.Actors(actorId).Yaw.Data(:,:,idx);
                    tracksWorld.Actors(actorId).AngularVelocity = tracks.Values.Actors(actorId).AngularVelocity.Data(:,:,idx);
                end
                for i= 1:size(detections.Values.Detections,1)
                    detectionsData.Detections(i).Time  = detections.Values.Detections(i).Time.Data(:,:,idx);
                    detectionsData.Detections(i).Measurement  = detections.Values.Detections(i).Measurement.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementNoise  = detections.Values.Detections(i).MeasurementNoise.Data(:,:,idx);
                    detectionsData.Detections(i).SensorIndex  = detections.Values.Detections(i).SensorIndex.Data(:,:,idx);
                    detectionsData.Detections(i).ObjectClassID  = detections.Values.Detections(i).ObjectClassID.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.Frame  = detections.Values.Detections(i).MeasurementParameters.Frame.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.OriginPosition = detections.Values.Detections(i).MeasurementParameters.OriginPosition.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.Orientation  = detections.Values.Detections(i).MeasurementParameters.Orientation.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.HasVelocity  = detections.Values.Detections(i).MeasurementParameters.HasVelocity.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.OriginVelocity  = detections.Values.Detections(i).MeasurementParameters.OriginVelocity.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.IsParentToChild  = detections.Values.Detections(i).MeasurementParameters.IsParentToChild.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.HasAzimuth  = detections.Values.Detections(i).MeasurementParameters.HasAzimuth.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.HasElevation  = detections.Values.Detections(i).MeasurementParameters.HasElevation.Data(:,:,idx);
                    detectionsData.Detections(i).MeasurementParameters.HasRange  = detections.Values.Detections(i).MeasurementParameters.HasRange.Data(:,:,idx);
                end
            end
        end

        function [posRadar, posCamera] = parseDetections(obj, detections,egoActor)
            % parseDetections parses detections information.
            pos = zeros(3,numel(detections.Detections));
            sIdx = arrayfun(@(x)x.SensorIndex, detections.Detections);
            uqIdx = unique(sIdx);
            for i = 1:numel(uqIdx)
                if (uqIdx(i) ~=0 )
                    thisIdx = sIdx == uqIdx(i);
                    pos(:,thisIdx) = calculatePositionInScenarioFrame(obj, detections.Detections(thisIdx), egoActor);
                end
            end
            pos = pos(1:2,:)';
            pos = [pos 0.5*ones(1,numel(detections.Detections))'];
            detectionSensorIndex = arrayfun(@(x)x.SensorIndex,detections.Detections);
            radarSensorIndex = 1;
            cameraSensorIndex = [2 3 4 5 6]';
            posRadar = pos(ismember(detectionSensorIndex,radarSensorIndex),:);
            posCamera = pos(ismember(detectionSensorIndex,cameraSensorIndex),:);
        end
               
        function [posScene] = calculatePositionInScenarioFrame(~, detections, egoVehicle)
            % Calculate Cartesian positions for all detections in the "sensor"
            % coordinate frame
            allDets = detections;
            meas = horzcat(allDets.Measurement);
            
            if strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical')
                az = meas(1,:);
                r = meas(2,:);
                el = zeros(1,numel(az));
                [x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
                posSensor = [x;y;z];
                rr = meas(3,:);
                rVec = posSensor./sqrt(dot(posSensor,posSensor,1));
                velSensor = rr.*rVec;
            else
                posSensor = meas;
                velSensor = zeros(3,size(meas,2));
            end
            
            % Transform parameters
            sensorToEgo = detections(1).MeasurementParameters(1);
            R = sensorToEgo.Orientation;
            T = sensorToEgo.OriginPosition;
            if isfield(sensorToEgo,'OriginVelocity')
                Tdot = sensorToEgo.OriginVelocity;
            else
                Tdot = zeros(3,1);
            end
            
            if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
                R = R';
            end
            
            % Position, velocity in ego frame
            posEgo = T + R*posSensor;
            velEgo = Tdot + R*velSensor; % Assume Rdot = 0;
            
            % egoToScenario
            R2 = rotmat(quaternion([egoVehicle.Yaw egoVehicle.Pitch egoVehicle.Roll],'eulerd','ZYX','frame'),'point');
            T = egoVehicle.Position(:);
            Tdot = egoVehicle.Velocity(:);
            posScene = T + R2*posEgo;
            velScene = Tdot + R2*velEgo;
        end

        function plotTracks(~, hTrackPlotter,tracks)
            % Plot tracks on chase view
            pos = zeros(0,3);
            vel = zeros(0,3);
            labels = string.empty(0,1);
            for i = 1:tracks.NumActors
                pos = [pos;tracks.Actors(i).Position];
                vel = [vel;tracks.Actors(i).Velocity];
                labels = [labels;string([tracks.Actors(i).ActorID])];
            
            end
            % % Plot tracks on chase view
            plotTrack(hTrackPlotter, pos, vel, labels);
        end
    
    end
end