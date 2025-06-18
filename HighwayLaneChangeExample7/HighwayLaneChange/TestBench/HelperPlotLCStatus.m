classdef HelperPlotLCStatus < HelperPlotLCPlannerStatus & matlab.System
    %HelperPlotLCStatus helps in visualizing the execution of
    %trajectory on chase view and top view
    %   using MATLAB Chase plot for the ego vehicle
    %     This System object helps in visualizing
    %     1. Sampled trajectories and their status.
    %     2. Optimal trajectory.
    %     3. Capsules used for collision checking.
    %     4. Other target vehicles in the scenario.
    %     5. Tracks obtained from sensor fusion and tracking. 
    %
    %   NOTE: This is a helper function for example purposes and
    %   may be removed or modified in the future.
    
    %   Copyright 2022 The MathWorks, Inc.
    
    % Private properties of the System object
    properties(Access = private)        
        %Plots to display tracks on chase view
        TheaterPlot;
        TrackPlotter;
        
        %Plots to display detections on chase view
        RadarDetectionPlotter;
        VisionDetectionPlotter;

        %Plots to display tracks on top view
        TheaterPlotTop;
        TrackPlotterTop;

        %Plots to display detections on chase view
        RadarDetectionPlotterTop;
        VisionDetectionPlotterTop;
    end

    properties(Nontunable)

        %EnableTracksChaseView Enable Tracks and Detections on Chase View
        EnableTracksChaseView (1, 1) logical = true;

        %EnableTracksTopView Enable Tracks and Detections on Top view
        EnableTracksTopView (1, 1) logical = true;

    end

    properties
        %Source of Target Actors
        ActorsInfo = "Truth";
    end

    % Constants used in the System object.
    properties(Constant, Hidden)

        ActorsInfoSet = matlab.system.StringSet(["Tracks","Truth"]);

    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods
        function obj = HelperPlotLCStatus(varargin)
            obj@HelperPlotLCPlannerStatus()
            % Support name-value pair arguments when constructing object
            if ~obj.EnableChaseView
                obj.EnableTrajChase = false;
                obj.EnableCaplistChaseView = false;
                obj.EnableTracksChaseView = false;
            end
            if ~obj.EnableTopView
                obj.EnableTrajTopView = false;   
                obj.EnableTracksTopView = false;
            end
        end
    end
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl Initializes the properties of System object.
            % The setupImpl method initializes the Figure, chase view,
            % top view,line handle and legend for both top view and chase
            % view.
            setupImpl@HelperPlotLCPlannerStatus(obj);
            
            % Initialize chase view and top view
            if obj.EnableTopView && obj.EnableChaseView
                % Create track plotter
                if (obj.EnableTracksTopView)
                    obj.TheaterPlotTop = theaterPlot('Parent',obj.HAxisTop);
                    obj.TrackPlotterTop = trackPlotter(obj.TheaterPlotTop,'DisplayName','Tracks','HistoryDepth',7);
                    % Create detections plotter
                    obj.RadarDetectionPlotterTop = detectionPlotter(obj.TheaterPlotTop,'Marker','o','MarkerFaceColor','r');
                    obj.VisionDetectionPlotterTop = detectionPlotter(obj.TheaterPlotTop,'Marker','^','MarkerFaceColor','b');

                    % Find all actor patches
                    allPatches = findall(obj.HAxisTop,'Type','Patch');
                    allTags = arrayfun(@(x)x.Tag,allPatches,'UniformOutput',false);
                    actorPatches = allPatches(startsWith(allTags,'ActorPatch'));
                    for i = 1:numel(actorPatches)
                        actorPatches(i).FaceAlpha = 0.3;
                        actorPatches(i).EdgeAlpha = 1;
                    end
                end

                if (obj.EnableTracksChaseView)
                    obj.TheaterPlot = theaterPlot('Parent',obj.HAxisChase);
                    obj.TrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Tracks','HistoryDepth',7);

                    % Create detections plotter
                    obj.RadarDetectionPlotter = detectionPlotter(obj.TheaterPlot,'Marker','o','MarkerFaceColor','r');
                    obj.VisionDetectionPlotter = detectionPlotter(obj.TheaterPlot,'Marker','^','MarkerFaceColor','b');

                    % Find all actor patches
                    allPatches = findall(obj.HAxisChase,'Type','Patch');
                    allTags = arrayfun(@(x)x.Tag,allPatches,'UniformOutput',false);
                    actorPatches = allPatches(startsWith(allTags,'ActorPatch'));
    
                    for i = 1:numel(actorPatches)
                        actorPatches(i).FaceAlpha = 0.3;
                        actorPatches(i).EdgeAlpha = 1;
                    end
                end
            elseif obj.EnableChaseView
                % Create track plotter
                if (obj.EnableTracksChaseView)
                    obj.TheaterPlot = theaterPlot('Parent',obj.HAxisChase);
                    obj.TrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Tracks','HistoryDepth',7);

                    % Create detections plotter
                    obj.RadarDetectionPlotter = detectionPlotter(obj.TheaterPlot,'DisplayName','Radar Detections',...
                        'Marker','o','MarkerFaceColor','r');
                    obj.VisionDetectionPlotter = detectionPlotter(obj.TheaterPlot,'DisplayName','Vision Detections',...
                        'Marker','^','MarkerFaceColor','b');
                end
            else
                % Create track plotter
                if (obj.EnableTracksTopView)
                    obj.TheaterPlotTop = theaterPlot('Parent',obj.HAxisTop);
                    obj.TrackPlotterTop = trackPlotter(obj.TheaterPlotTop,'DisplayName','Tracks','HistoryDepth',7);

                    % Create detections plotter
                    obj.RadarDetectionPlotterTop = detectionPlotter(obj.TheaterPlotTop,'Marker','o','MarkerFaceColor','r');
                    obj.VisionDetectionPlotterTop = detectionPlotter(obj.TheaterPlotTop,'Marker','^','MarkerFaceColor','b');
                end
            end
            
            %% Create legend for the candidate trajectories in the plot
            % Legend for chase view
            if obj.EnableChaseView && obj.EnableTrajChase
                hChase(1) = line(obj.HAxisChase,0,0,...
                    'LineStyle','-','Color','g','LineWidth',3);
                set(hChase(1),'XData',[],'YData',[]);
                
                hChase(2) = line(obj.HAxisChase,0,0,...
                    'LineStyle','-','Color','r','LineWidth',2);
                set(hChase(2),'XData',[],'YData',[]);
                
                hChase(3) = line(obj.HAxisChase,0,0,...
                    'LineStyle','--','Color','c','LineWidth',2);
                set(hChase(3),'XData',[],'YData',[]);
                
                hChase(4) = line(obj.HAxisChase,0,0,...
                    'LineStyle','-','Color','k','LineWidth',2);
                set(hChase(4),'XData',[],'YData',[]);
                if obj.EnableTracksChaseView

                    hChase(5) = line(obj.HAxisChase,0,0,...
                        'LineStyle','none','Marker','o','Color','r','LineWidth',3,'MarkerSize',3);
                    set(hChase(5),'XData',[],'YData',[]);

                    hChase(6) = line(obj.HAxisChase,0,0,...
                        'LineStyle','none','Marker','^','Color','b','LineWidth',3,'MarkerSize',3);
                    set(hChase(6),'XData',[],'YData',[]);

                    hChase(7) = line(obj.HAxisChase,0,0,...
                        'LineStyle','none','Marker','s','Color','k','LineWidth',1,'MarkerSize',10);
                    set(hChase(7),'XData',[],'YData',[]);

                    legend(obj.HAxisChase,hChase,...
                        {'optimal','colliding','infeasible','not evaluated','Radar Detections','Vision Detections','Tracks'},...
                        'Color',[0.94,0.94,0.94],...
                        'AutoUpdate','off');
                else
                    legend(obj.HAxisChase,hChase,...
                        {'optimal','colliding','infeasible','not evaluated'},...
                        'Color',[0.94,0.94,0.94],...
                        'AutoUpdate','off');
                end
            end
            
            % Legend for top view
            if obj.EnableTopView && obj.EnableTrajTopView
                hTop(1) = line(obj.HAxisTop,0,0,...
                    'LineStyle','-','Color','g','LineWidth',3);
                set(hTop(1),'XData',[],'YData',[]);
                hTop(2) = line(obj.HAxisTop,0,0,...
                    'LineStyle','-','Color','r','LineWidth',2);
                set(hTop(2),'XData',[],'YData',[]);
                hTop(3) = line(obj.HAxisTop,0,0,...
                    'LineStyle','--','Color','c','LineWidth',2);
                set(hTop(3),'XData',[],'YData',[]);
                hTop(4) = line(obj.HAxisTop,0,0,...
                    'LineStyle','-','Color','k','LineWidth',2);
                set(hTop(4),'XData',[],'YData',[]);

                if(obj.EnableTracksTopView)
                    hTop(5) = line(obj.HAxisTop,0,0,...
                        'LineStyle','none','Marker','o','Color','r','LineWidth',3,'MarkerSize',3);
                    set(hTop(5),'XData',[],'YData',[]);

                    hTop(6) = line(obj.HAxisTop,0,0,...
                        'LineStyle','none','Marker','^','Color','b','LineWidth',3,'MarkerSize',3);
                    set(hTop(6),'XData',[],'YData',[]);

                    hTop(7) = line(obj.HAxisTop,0,0,...
                        'LineStyle','none','Marker','s','Color','k','LineWidth',1,'MarkerSize',10);
                    set(hTop(7),'XData',[],'YData',[]);

                    legend(obj.HAxisTop,hTop,...
                        {'optimal','colliding','infeasible','not evaluated','Radar Detections','Vision Detections','Tracks'},...
                        'Color',[0.94,0.94,0.94],...
                        'AutoUpdate','off');
                else
                    legend(obj.HAxisTop,hTop,...
                        {'optimal','colliding','infeasible','not evaluated'},...
                        'Color',[0.94,0.94,0.94],...
                        'AutoUpdate','off');
                end
            end
        end
        
        %------------------------------------------------------------------
        function stepImpl(obj, varargin)

            %stepImpl implements the core logic for visualization at every
            %simulation step
            %
            %   The stepImpl implements the logic to plot capsule list and
            %   sampled trajectories on both top view and chase view of the
            %   MATLAB figure.
            EgoActor = varargin{4};
            if(nargin == 8)
                Tracks = varargin{5};
                EstimatedTargets = varargin{6};
                Detections = varargin{7};
            end

            if obj.EnableChaseView || obj.EnableTopView
                if(obj.EnableTracksChaseView || obj.EnableTracksTopView )

                    pos = zeros(0,3);
                    vel = zeros(0,3);
                    labels = string.empty(0,1);

                for i = 1:EstimatedTargets.NumActors                  
                    pos = [pos;EstimatedTargets.Actors(i).Position];
                    vel = [vel;EstimatedTargets.Actors(i).Velocity];
                    labels = [labels;string([Tracks.Tracks(i).TrackID])];
                end
                % Plot tracks on chase view
                if(obj.EnableTracksChaseView)
                    [posRadar, posCamera] = parseDetections(obj,Detections,EgoActor);
                    plotDetection(obj.RadarDetectionPlotter,posRadar);
                    plotDetection(obj.VisionDetectionPlotter,posCamera);
                    plotTrack(obj.TrackPlotter, pos, vel, labels);
                end
                 % Plot tracks on top view
                if(obj.EnableTracksTopView)
                    [posRadar, posCamera] = parseDetections(obj,Detections,EgoActor);
                    plotDetection(obj.RadarDetectionPlotterTop,posRadar);
                    plotDetection(obj.VisionDetectionPlotterTop,posCamera);
                    plotTrack(obj.TrackPlotterTop, pos, vel, labels);
                end

                stepImpl@HelperPlotLCPlannerStatus(obj, varargin{1}, varargin{2}, varargin{3}, varargin{4});

                end
            end
        end
        function [posRadar, posCamera] = parseDetections(obj,detections,egoActor)
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


        function [posScene] = calculatePositionInScenarioFrame(obj,detections, egoVehicle)

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

        function flag = isInactivePropertyImpl(obj,propertyName)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog

            switch propertyName
                case 'EnableTrajTopView'
                     flag = ~obj.EnableTopView;
                case 'EnableTracksTopView'
                    flag = ~obj.EnableTopView;
                    if strcmp(obj.ActorsInfo,'Truth')
                        flag = true;  
                    end
                case 'EnableTrajChase'
                    flag = ~obj.EnableChaseView;
                case 'EnableCaplistChaseView'
                    flag = ~obj.EnableChaseView;
                case 'EnableTracksChaseView'
                    flag = ~obj.EnableChaseView;
                    if strcmp(obj.ActorsInfo,'Truth')
                        flag = true;
                    end
                otherwise
                    flag = false;
            end
        end

        function num = getNumInputsImpl(obj)
            num = 4;
            if strcmp(obj.ActorsInfo,'Tracks')
                num = 7;
            end
        end

        function varargout  = getInputNamesImpl(obj)
             numInputs = getNumInputs(obj);
             varargout = cell(1,numInputs);
             % Return input port names for System block
             varargout{1}  = 'TargetActorsWorld';
             varargout{2}  = 'VisualizationInfo';
             varargout{3}  = 'ObstacleProfiles';
             varargout{4}  = 'EgoActor';

             if(numInputs == 7)
                 varargout{5}  = 'Tracks';
                 varargout{6}  = 'EstimatedTargets';
                 varargout{7}  = 'Detections'; 
                 if(obj.EnableTracksChaseView || obj.EnableTracksTopView)
                     if(~license('test','sensor_fusion_and_tracking') && ~license('test','radar_toolbox') )
                         errordlg('Unble to visualize tracks because license for Sensor Fusion and Tracking Toolbox or Radar Toolbox is not found.');
                     end
                 end
             end
        end
    end
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(....
                "Title","HelperPlotLCStatus",...
                "Text",...
                "Helps in visualizing the execution of trajectory using MATLAB figure in Chase view and Top view");
        end
        function groups = getPropertyGroupsImpl
            % Define property group for tracks 
            tracksGroup = matlab.system.display.Section('Title','System parameters',...
                'PropertyList',{'ActorsInfo'});
            % Define property group for top view.
            topViewGroup = matlab.system.display.Section('Title','Top View parameters',...
                'PropertyList',{'EnableTopView','EnableTrajTopView','EnableTracksTopView'});
            % Define property group for chase view.
            chaseViewGroup = matlab.system.display.Section('Title','Chase View parameters',...
                'PropertyList',{'EnableChaseView','EnableTrajChase','EnableCaplistChaseView','EnableTracksChaseView'});
            % Return an array with the two sections.
            groups = [tracksGroup,topViewGroup, chaseViewGroup];
        end
         
        %------------------------------------------------------------------
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
        
        %------------------------------------------------------------------
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
        
        %------------------------------------------------------------------
        function releaseImpl(obj)
            if obj.EnableChaseView || obj.EnableTopView
                % Modifying the CloseRequestFcn callback to its original state
                set(obj.Figure,'CloseRequestFcn','closereq');
            else
                close(obj.Figure);
            end
        end
    end
    
end