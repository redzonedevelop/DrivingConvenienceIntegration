% helperSLHighwayLaneChangeCleanUp script clears all workspace variables
% created during execution of this example.

% This is a helper script for example purposes and may be removed or
% modified in the future.

% Copyright 2019-2022 The MathWorks, Inc.

helperSLHLCPlannerAndControllerCleanUp;

clearBuses({ ...
    'BusConcatenationGlobal',...
    'BusConcatenationDetectionsMeasurementParametersGlobal',...
    'BusConcatenationDetectionsGlobal',...
    'BusDetectionConcatenation1',...
    'BusDetectionConcatenation1Detections',...
    'BusDetectionConcatenation2',...
    'BusDetectionConcatenation2Detections',...
    'BusDetectionConcatenation2DetectionsMeasurementParameters',...
    'BusDrivingRadarDataGenerator',...
    'BusDrivingRadarDataGeneratorDetections',...
    'BusDrivingRadarDataGeneratorDetectionsMeasurementParameters',...
    'BusEgoPose',...
    'BusObjectDetections1',...
    'BusObjectDetections1Detections',...
    'BusRadarDetectionGenerator1DetectionsObjectAttributes',...
    'BusTracker',...
    'BusTrackerTracks',...
    'BusTrackerJPDA',...
    'BusTrackerJPDATracks',...
    'BusTrackerTracks',...
    'BusTruth',...
    'BusVision',...
    'BusVisionDetections',...
    'BusVisionDetectionsObjectAttributes',...
    'BusVisionDetectionsMeasurementParameters'});

clear actorProfiles;
clear camera;
clear egoVehicle;
clear M;
clear N;
clear numCoasts;
clear numSensors;
clear numTargets;
clear numTracks;
clear P;
clear Q;
clear radar;
clear tout;
clear assigThresh;
clear trackAssignmentThreshold;
clear trackRegisterThreshold;

function clearBuses(buses)
matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
end
