function helperSLHighwayLaneChangeSetup(nvp)
% helperSLHighwayLaneChangeSetup creates base workspace variables required
% for Highway Lane Change Example
%
% helperSLHighwayLaneChangeSetup(scenarioFcnName)
% 
% Optional inputs
%   scenarioFcnName:
%     - Name of function which returns scenario which is
%       compatible with HighwayLaneChangeTestBench.slx
%     - Valid values are:
%         "scenario_LC_01_SlowMoving"
%         "scenario_LC_02_SlowMovingWithPassingCar"
%         "scenario_LC_03_DisabledCar"
%         "scenario_LC_04_CutInWithBrake"
%         "scenario_LC_05_SingleLaneChange"
%         "scenario_LC_06_DoubleLaneChange"
%         "scenario_LC_07_RightLaneChange"
%         "scenario_LC_08_SlowmovingCar_Curved"
%         "scenario_LC_09_CutInWithBrake_Curved"
%         "scenario_LC_10_SingleLaneChange_Curved"
%         "scenario_LC_11_MergingCar_HighwayEntry"
%         "scenario_LC_12_CutInCar_HighwayEntry"
%         "scenario_LC_13_DisabledCar_Ushape"
%         "scenario_LC_14_DoubleLaneChange_Ushape"
%         "scenario_LC_15_StopnGo_Curved"[Default]

% Examples of calling this function:
%
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_01_SlowMoving");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_02_SlowMovingWithPassingCar");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_03_DisabledCar");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_04_CutInWithBrake");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_05_SingleLaneChange");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_06_DoubleLaneChange");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_07_RightLaneChange");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_08_SlowmovingCar_Curved");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_09_CutInWithBrake_Curved");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_10_SingleLaneChange_Curved");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_11_MergingCar_HighwayEntry");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_12_CutInCar_HighwayEntry");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_13_DisabledCar_Ushape");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_14_DoubleLaneChange_Ushape");
%  helperSLHighwayLaneChangeSetup("scenarioFcnName","scenario_LC_15_StopnGo_Curved");
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2019-2022 The MathWorks, Inc.

%% Inputs
arguments
    nvp.scenarioFcnName {mustBeMember(nvp.scenarioFcnName,...
        ["scenario_LC_01_SlowMoving";...
        "scenario_LC_02_SlowMovingWithPassingCar";...
        "scenario_LC_03_DisabledCar";...
        "scenario_LC_04_CutInWithBrake";...
        "scenario_LC_05_SingleLaneChange";...
        "scenario_LC_06_DoubleLaneChange";...
        "scenario_LC_07_RightLaneChange";...
        "scenario_LC_08_SlowmovingCar_Curved";...
        "scenario_LC_09_CutInWithBrake_Curved";...
        "scenario_LC_10_SingleLaneChange_Curved";...
        "scenario_LC_11_MergingCar_HighwayEntry";...
        "scenario_LC_12_CutInCar_HighwayEntry";...
        "scenario_LC_13_DisabledCar_Ushape";...
        "scenario_LC_14_DoubleLaneChange_Ushape";...
        "scenario_LC_15_StopnGo_Curved"])} = "scenario_LC_15_StopnGo_Curved";
end

% Load the test bench model
modelName = "HighwayLaneChangeTestBench";
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

assignin('base', 'scenarioFcnName', nvp.scenarioFcnName);

%% Scenario parameters
% Call scenario function
scenarioFcnHandle = str2func(nvp.scenarioFcnName);
[scenario, egoActor, roadCenters] = scenarioFcnHandle();

helperInitializeLaneChangeParameters(scenario, egoActor, roadCenters,"ActorsInfo","sensorBased")

%% Sensor parameters
camera = cameraParams(egoActor);
assignin('base','camera',    camera);
assignin('base','radar',     radarParams());
assignin('base','egoVehicle',     egoActor);

%% Tracking and sensor fusion parameters
% 
assignin('base','assigThresh',  80);       % Tracker assignment threshold          (N/A)
assignin('base','M',            3);        % Tracker M value for M-out-of-N logic  (N/A)
assignin('base','N',            4);        % Tracker N value for M-out-of-N logic  (N/A)
assignin('base','P',            5);        % Tracker P value for deletion logic  (N/A)
assignin('base','Q',            5);        % Tracker Q value for deletion logic  (N/A)
assignin('base','trackRegisterThreshold',0.5); % Threshold for track register 
assignin('base','trackAssignmentThreshold',200);% Threshold for track assignment
assignin('base','numCoasts',    3);        % Number of track coasting steps        (N/A)
assignin('base','numTracks',    100);      % Maximum number of tracks              (N/A)
assignin('base','numSensors',  6);         % Maximum number of sensors      (N/A)

numStates = 4;
numPreviousStatesToStore = 10;
evalin('base',sprintf('helperCreateSVSFBusObjects(%d,%d)',....
    numStates,numPreviousStatesToStore));

%% Vehicle parameters
egoVehDyn = egoVehicleDynamicsParams(scenario);
assignin('base','egoVehDyn', egoVehDyn);

maxTracks  = 30;

evalin('base', sprintf('helperCreateLCBusActorsTruth(%d)',...
                       length(scenario.actorProfiles) - 1));
evalin('base', sprintf('helperCreateLCBusActorsEgo(%d)', ...
                          maxTracks));
assignin('base','actorProfiles', actorProfiles(scenario));

end

function egoVehDyn = egoVehicleDynamicsParams(scenario)
% egoVehicleDynamicsParams vehicle dynamics parameters from scenario
%
% Scenario is in ISO 8855 (North-West-Up) with respect to rear axle
% Returns struct in SAE J670E (North-East-Down) with respect to
% center of gravity (vehicle center)
%
%  egoVehDyn.X0            % Initial position X (m)
%  egoVehDyn.Y0            % Initial position Y (m)
%  egoVehDyn.Yaw0          % Initial yaw (rad)
%  egoVehDyn.VLong0        % Initial longitudinal velocity(m/sec)
%  egoVehDyn.CGToFrontAxle % Distance center of gravity to front axle (m)
%  egoVehDyn.CGToRearAxle  % Distance center of gravity to rear axle (m)

% Ego in ISO 8855 (North-West-Up) with respect to rear axle
ego = scenario.Actors(1);

% Shift reference position to center of gravity (vehicle center)
position_CG = driving.scenario.internal.Utilities.translateVehiclePosition(...
    ego.Position,...     % Position with respect to rear axle (m)
    ego.RearOverhang,... % (m)
    ego.Length,...       % (m)
    ego.Roll,...         % (deg)
    ego.Pitch,...        % (deg)
    ego.Yaw);            % (deg)

% Translate to SAE J670E (North-East-Down)
% Adjust sign of y position to 
egoVehDyn.X0  =  position_CG(1); % (m)
egoVehDyn.Y0  = -position_CG(2); % (m)
egoVehDyn.VX0 =  ego.Velocity(1); % (m)
egoVehDyn.VY0 = -ego.Velocity(2); % (m)

% Adjust sign and unit of yaw
egoVehDyn.Yaw0 = -deg2rad(ego.Yaw); % (rad)

% Longitudinal velocity 
egoVehDyn.VLong0 = hypot(egoVehDyn.VX0,egoVehDyn.VY0); % (m/sec)

% Distance from center of gravity to axles
egoVehDyn.CGToFrontAxle = ego.Length/2 - ego.FrontOverhang;
egoVehDyn.CGToRearAxle  = ego.Length/2 - ego.RearOverhang;

end

function camera = cameraParams(egoVehicle)
% Front Camera sensor parameters
camera(1).Position = [ 2.95, 0, 1.1];% Position with respect to rear axle (m)    
camera(1).Rotation = [0, 1, 0];      % Rotation [roll, pitch, yaw] (deg)
camera(1).FieldOfView     = [45,45]; % Field of view (degrees)
camera(1).DetectionRanges  = [6 250];% Full range of camera (m)
camera(1).ImageSize       = [600, 800];
camera(1).PrincipalPoint  = [400 300];
camera(1).FocalLength     = [900 900]; 
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
camera(1).RangeTolerance = egoVehicle.RearOverhang; % Rear overhang of car 

% Front Left Camera sensor parameters
camera(2).Position  = [2, 0.9, 0.7]; % Position with respect to rear axle (m)
camera(2).Rotation = [0, 1, 65];     % Rotation [roll, pitch, yaw] (deg)
camera(2).FieldOfView     = [45,45]; % Field of view (degrees)
camera(2).DetectionRanges  = [6 80]; % Full range of camera (m)
camera(2).ImageSize       = [800, 1024];
camera(2).PrincipalPoint  = [512 400];
camera(2).FocalLength     = [400 400];
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
camera(2).RangeTolerance =  0; 

% Rear Left Camera sensor parameters
camera(3).Position = [2.8,0.9,0.7];  % Position with respect to rear axle (m)
camera(3).Rotation = [0, 1, 140];    % Rotation [roll, pitch, yaw] (deg)
camera(3).FieldOfView     = [45,45]; % Field of view (degrees)
camera(3).DetectionRanges  = [6 100];% Full range of camera (m)
camera(3).ImageSize       = [720 1280];
camera(3).PrincipalPoint  = [640 360];
camera(3).FocalLength     = [720 720];
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
camera(3).RangeTolerance =  ...
    egoVehicle.Length - egoVehicle.RearOverhang ; %Vehicle length - Rear overhang of car

% Front Right Camera sensor parameters
camera(4).Position = [2,-0.9,0.7];   % Position with respect to rear axle (m)
camera(4).Rotation = [0, 1, -65];    % Rotation [roll, pitch, yaw] (deg)
camera(4).FieldOfView     = [45,45]; % Field of view (degrees)
camera(4).DetectionRanges  = [6 80]; % Full range of camera (m)
camera(4).ImageSize       = [800 1024];
camera(4).PrincipalPoint  = [512 400];
camera(4).FocalLength     = [400 400];
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
camera(4).RangeTolerance = 0 ; 

% Rear Right Camera sensor parameters
camera(5).Position = [2.8,-0.9,0.7]; % Position with respect to rear axle (m)
camera(5).Rotation = [0, 1, -140];   % Rotation [roll, pitch, yaw] (deg)
camera(5).FieldOfView     = [45,45]; % Field of view (degrees)
camera(5).DetectionRanges  = [6 100];% Full range of camera (m)
camera(5).ImageSize       = [720 1280];
camera(5).PrincipalPoint  = [640 360];
camera(5).FocalLength     = [720 720];
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
camera(5).RangeTolerance =  ...
    egoVehicle.Length - egoVehicle.RearOverhang; %Vehicle length - Rear overhang

end

function radar = radarParams()
% Radar sensor parameters
radar.FieldOfView     = [48,5];   % Field of view (degrees)
radar.DetectionRanges = [1,160];  % Ranges (m)
radar.Position        = ...       % Position with respect to rear axle (m)
    [3.7 0 0.2];
radar.Rotation = [ 0, 0, 0];      % [roll, pitch, yaw] (deg)
% Range tolerance to be added to detection range to consider full 
% length of vehicle in the sensor coverage
radar.RangeTolerance = 0;
end

