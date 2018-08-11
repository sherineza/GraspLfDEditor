% This is the runnable perception script. In the future this should be in
% only for initialisation and another function looped for detection.
function markers = Run_DetectMarkers(id,vrep)
%% Get image

%The vision sensor HANDLE
[returnCode , kinect] = vrep.simxGetObjectHandle(id,'kinect_rgb',vrep.simx_opmode_blocking);
[returnCode, resolution, kinect_rgb]=vrep.simxGetVisionSensorImage2(id, kinect, 0, vrep.simx_opmode_oneshot_wait);
% imshow(kinect_rgb);

%% Detect Markers

%% Compile Markers

%% FAKE GET MARKER
%Eventually loop for detecting all markers in scene
[returnCode, marker0] = vrep.simxGetObjectHandle(id, 'Marker0',vrep.simx_opmode_blocking);
[returnCode, pos_marker0] = vrep.simxGetObjectPosition(id, marker0,-1,vrep.simx_opmode_blocking);
[returnCode, orient_marker0] = vrep.simxGetObjectOrientation(id, marker0,-1,vrep.simx_opmode_blocking);
marker0=struct('handle',marker0);
marker0.pose=pos_marker0;
marker0.orient=orient_marker0;

[returnCode, marker1] = vrep.simxGetObjectHandle(id, 'Marker1',vrep.simx_opmode_blocking);
[returnCode, pos_marker1] = vrep.simxGetObjectPosition(id, marker1,-1,vrep.simx_opmode_blocking);
[returnCode, orient_marker1] = vrep.simxGetObjectOrientation(id, marker1,-1,vrep.simx_opmode_blocking);
marker1=struct('handle',marker1);
marker1.pose=pos_marker1;
marker1.orient=orient_marker1;

markers=[marker0, marker1];