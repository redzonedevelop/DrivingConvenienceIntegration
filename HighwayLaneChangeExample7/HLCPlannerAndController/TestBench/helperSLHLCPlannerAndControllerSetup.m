function helperSLHLCPlannerAndControllerSetup(nvp)
% helperSLHLCPlannerAndControllerSetup creates base workspace variables required
% for Highway Lane Change Planner And Controls Example
%
% helperSLHLCPlannerAndControllerSetup(scenarioFcnName)
% 
% Optional inputs
%   scenarioFcnName:
%     - Name of function which returns scenario which is
%       compatible with HLCPlannerAndControllerTestBench.slx
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
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_01_SlowMoving");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_02_SlowMovingWithPassingCar");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_03_DisabledCar");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_04_CutInWithBrake");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_05_SingleLaneChange");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_06_DoubleLaneChange");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_07_RightLaneChange");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_08_SlowmovingCar_Curved");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_09_CutInWithBrake_Curved");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_10_SingleLaneChange_Curved");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_11_MergingCar_HighwayEntry");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_12_CutInCar_HighwayEntry");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_13_DisabledCar_Ushape");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_14_DoubleLaneChange_Ushape");
%  helperSLHLCPlannerAndControllerSetup(scenarioFcnName = "scenario_LC_15_StopnGo_Curved");
%
% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2021-2022 The MathWorks, Inc.
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
modelName = "HLCPlannerAndControllerTestBench";
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

assignin('base', 'scenarioFcnName', nvp.scenarioFcnName);
%% Scenario parameters
% Call scenario function
scenarioFcnHandle = str2func(nvp.scenarioFcnName);
[scenario, egoActor, roadCenters] = scenarioFcnHandle();

helperInitializeLaneChangeParameters(scenario, egoActor, roadCenters);
%% Vehicle parameters
egoVehDyn = egoVehicleDynamicsParams(scenario);
assignin('base','egoVehDyn', egoVehDyn);
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