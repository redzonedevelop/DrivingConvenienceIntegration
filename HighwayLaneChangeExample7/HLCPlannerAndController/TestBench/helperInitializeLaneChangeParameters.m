function helperInitializeLaneChangeParameters(scenario, egoActor, roadCenters,options)
% helperInitializeLaneChangeParameters creates base workspace variables required
% for Highway Lane Change related examples
%
% helperInitializeLaneChangeParameters(scenario, egoActor, roadcenters)
% helperInitializeLaneChangeParameters(scenario, egoActor,
% roadcenters,'ActorsInfo','truthBased');
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2021-2022 The MathWorks, Inc.

arguments
    scenario
    egoActor
    roadCenters
    options.ActorsInfo(1,1) string = 'truthBased';
end
actorsInfo = options.ActorsInfo;

%% Default assessments
assessment.TimeGap = 0.8;
assessment.LongitudinalJerkMax = 5;
assessment.LateralJerkMax =  5;
assessment.frontSafetyGap = 20;
assessment.rearSafetyGap = 5;
assessment.egoTTC = 2;
assessment.nextTTC = 5;

% Assign scenario object and assessment to base workspace
assignin('base', 'scenario', scenario);
assignin('base', 'assessment', assessment);

%% Extract scenario information
% Road center information
globalPlanPoints = roadCenters(:,1:2);


egoInitialPose = struct('ActorID', egoActor.ActorID,...
                        'Position', egoActor.Position,...
                        'Velocity', egoActor.Velocity,...
                        'Roll', egoActor.Roll,...
                        'Pitch', egoActor.Pitch,...
                        'Yaw', egoActor.Yaw,...
                        'AngularVelocity', egoActor.AngularVelocity);

assignin('base', 'egoInitialPose', egoInitialPose);
assignin('base', 'egoActorID', egoActor.ActorID);

% Ego set speed (m/s)
egoSetVelocity = hypot(egoActor(1).Velocity(1), egoActor(1).Velocity(2));


%% General Model Parameters
% Simulation sample time (s)
assignin('base', 'Ts', 0.1);

roadCenters = scenario.RoadSegments.RoadCenters;
roadCenters(:,3) = [];

egoActor = scenario.Actors(1);
currentEgoStates = [egoActor.Position(1), egoActor.Position(2), ...
deg2rad(egoActor.Yaw), 0, norm(egoActor.Velocity),...
0];

%% Update road network information
% Create referencePathFrenet object to find the initial state of the
% vehicle.
refPathFrenet = referencePathFrenet(roadCenters);
currentFrenetStates = global2frenet(refPathFrenet, currentEgoStates(1:6));

% Get car width and consider it as minimum lane width
minLaneWidth = egoActor.Width;

mapInfo = getMapInfo(scenario, globalPlanPoints, currentFrenetStates(4),...
                     minLaneWidth);

assignin('base', 'mapInfo', mapInfo);

% Define replan rate for planner
replanRate = 1;
assignin('base','replanRate',replanRate);



%% Define planner parameters
% Define behavior parameters and assign them to base workspace
if(strcmp(actorsInfo,'sensorBased')) 
    timeHorizon = 2:3;
else
    timeHorizon = 1:3;
end

timeResolution = 0.1;
assignin('base', 'timeHorizon' , timeHorizon);
assignin('base','timeResolution',timeResolution);
assignin('base','preferredLane',helperDetectLaneNumber(mapInfo, currentFrenetStates(4)));
assignin('base','maxPlanningHorizon',80);
assignin('base','setSpeed',egoSetVelocity);

% Define safety parameters and assign them to base workspace
assignin('base','frontSafetyGap',30);
assignin('base','rearSafetyGap',10);
assignin('base','egoFrontExt',5);
assignin('base','targetFrontExt',5);
assignin('base','egoTTC',4);
assignin('base','nextTTC',4);


%% Cost weights
assignin('base','latDevCost',1.0);
assignin('base','timeCost',-1.0);
assignin('base','speedCost',1.0);

%% Feasibility Parameters
assignin('base','maxAccel',5.0);
assignin('base','maxCurvature',1.0);
assignin('base','minVelocity',0);
assignin('base','maxYawRate',80.0);

%% Behavior selector flag
assignin('base','enableCCBehavior',1);
assignin('base','enableLCFBehavior',1);
assignin('base','enableLCBehavior',1);

%% Set compute method for the computation of current state of ego vehicle.
% Use "TimeBased' computation method for the
% HighwayLaneChangePlannerTestBench and "PoseBased" for
% HighwayLaneChangeTestBench and HLCPlannerAndControlsTestBench model.
assignin('base','computeMethod',ComputeMethod.PoseBased);

%% Path following Controller Parameters
createPathFollowingControllerParams();

maxStatesPerBehavior = 10;
maxBehaviors = 3;
maxTrajectories = maxBehaviors*maxStatesPerBehavior;
maxTrajectoryPoints = max(timeHorizon)/timeResolution+1;
maxMIOs = 100;
maxGlobalPlanPoints = 5000;
maxNumLanes = 100;
maxTargetActors = 50;

assignin('base','maxStatesPerBehavior', maxStatesPerBehavior);
assignin('base','maxTrajectories', maxTrajectories);
assignin('base','maxTrajectoryPoints', maxTrajectoryPoints);
assignin('base','maxMIOs', maxMIOs);
assignin('base','maxGlobalPlanPoints', maxGlobalPlanPoints);
assignin('base','maxNumLanes', maxNumLanes);
%% Buses Creation  
helperCreateLCPlannerBusObjects(timeHorizon, maxStatesPerBehavior,...
                                maxTrajectories,...
                                maxTrajectoryPoints,...
                                maxNumLanes,...
                                maxGlobalPlanPoints,...
                                maxMIOs);
% Get number of target actors in the scenario
numTargetActors = size(scenario.Actors,2)-1;
assignin('base','numTargetActors',numTargetActors);
maxTracks = 50;
if(strcmp(actorsInfo,'truthBased'))    
    evalin('base', sprintf('helperCreateBusPredictedTrajectory(%d,%d)',...
        maxTargetActors, maxTrajectoryPoints));
    evalin('base', sprintf('helperCreateLCBusActorsEgo(%d)', ...
        length(scenario.actorProfiles) - 1));
elseif(strcmp(actorsInfo,'sensorBased'))
    evalin('base', sprintf('helperCreateBusPredictedTrajectory(%d,%d)',...
        maxTracks, maxTrajectoryPoints));
    evalin('base', sprintf('helperCreateLCBusActorsEgo(%d)', ...
        maxTracks));
end
 evalin('base', sprintf('helperCreateLCBusActorsTruth(%d)', ...
        length(scenario.actorProfiles) - 1));


end

function laneCenters = calculateLaneCenters(laneWidth,roadCenter)
%calculateLaneCenters computes the lane center distance from road
%center.

% Get number of lanes
numLanes = length(laneWidth);

% Initialize lane centers
laneCenters = zeros(1,numLanes);
laneCenters(1) = roadCenter - laneWidth(1)/2;

for i = 2:numLanes
    laneCenters(i) = laneCenters(i-1) - (laneWidth(i-1)+laneWidth(i))/2;
end
end

function mapInfo = getMapInfo(scenario, globalPlanPoints, egoLatDist, minLaneWidth)
% Get number of lanes from road network
numLanes = scenario.RoadSegments(1).NumLanes;

laneSpec = scenario.RoadSegments(1).LaneSpecification;
laneWidth = laneSpec.Width;
roadCenter = sum(laneWidth)/2;
laneCenters = calculateLaneCenters(laneWidth, roadCenter);

% Define maximum possible values for lanes and global plan points
maxNumLanes = 100;
maxGlobalPlanPoints = 5000;

% Initialize map data
mapInfo = struct('NumLanes', numLanes,'LaneWidth', zeros(maxNumLanes,1),...
                 'LaneCenters', zeros(maxNumLanes,1),'NumGlobalPlanPoints',...
                 size(globalPlanPoints,1), 'GlobalPlanPoints',...
                 zeros(maxGlobalPlanPoints,2));

numMarkings = length(laneSpec.Marking);
laneType = laneSpec.Type; % lane type

% Check lane type and update lane center information
for i = 1:numMarkings
    if isequal(laneSpec.Marking(i).Type,LaneBoundaryType.DoubleSolid) % road median?
        if egoLatDist > 0 % ego is on the left road
            laneWidth = laneWidth(1:i-1);
            laneCenters = laneCenters(1:i-1);
            laneType = laneType(1:i-1);
        else
            laneWidth = laneWidth(i:end);
            laneCenters = laneCenters(i:end);
            laneType = laneType(i:end);            
        end
        break;
    end      
end

% Extract driving lane
drivingLane = [laneType.Type]==LaneTypes.Driving & laneWidth>=minLaneWidth;

% Update number of lanes based on valid driving lanes in the scene
numLanes = nnz(drivingLane);
mapInfo.LaneWidth(1:numLanes,:) = laneWidth(drivingLane);
mapInfo.LaneCenters(1:numLanes,:)  = laneCenters(drivingLane);
mapInfo.NumLanes = numLanes;
mapInfo.GlobalPlanPoints(1:mapInfo.NumGlobalPlanPoints,:) = globalPlanPoints;
end
function createPathFollowingControllerParams

    %% Path following Controller(PFC) Constraints
    assignin('base', 'defaultSpacing'   , 10);    % Default spacing        (m)
    assignin('base','max_ac'            ,2);      % Maximum acceleration   (m/s^2)
    assignin('base','min_ac'            ,-3);     % Minimum acceleration   (m/s^2)
    assignin('base','max_dc'            ,-10);    % Maximum deceleration   (m/s^2)
    assignin('base', 'maxSteer'         , 0.26);  % Maximum steering       (rad)
    assignin('base', 'minSteer'         , -0.26); % Minimum steering       (rad)
    assignin('base', 'predictionHorizon', 30);    % Prediction horizon

    % time constant for longitudinal dynamics 1/s/(tau*s+1)
    tau = 0.5;
    assignin('base', 'tau', tau);
    assignin('base','tau2', 0.07);    % Longitudinal time constant (brake)             (N/A)

    % PFC Dynamics modeling parameters
    assignin('base', 'm',  1575);     % Total mass of vehicle                     (kg)
    assignin('base', 'Iz', 2875);     % Yaw moment of inertia of vehicle          (m*N*s^2)
    assignin('base', 'lf', 1.2);      % Longitudinal distance from c.g. to front tires (m)
    assignin('base', 'lr', 1.6);      % Longitudinal distance from c.g. to rear tires  (m)
    assignin('base', 'Cf', 19000);    % Cornering stiffness of front tires        (N/rad)
    assignin('base', 'Cr', 33000);    % Cornering stiffness of rear tires         (N/rad)

end
