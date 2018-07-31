function Run_RecordDemo (id,vrep,simulationNb)
% This runnable script is to grasp the object in a pre-determined position
% (specified in the V-rep child script) and log data of a demonstration run.
% The positions and orientation of frames and points are logged + video of
% demo.

% The grasping motion is set in the V-rep script of the robot and is
% activated through a signal signifying the start of recording data.
addpath(genpath('C:/Users/elzaatas/Documents/Matlab-Vrep'));
%% Api link to V-rep
% In case this function was run without inputs, provide API link to V-rep
% simulationNb guide (date logged: 260718)
%    19991 -> 'C:\Users\elzaatas\Documents\Matlab-Vrep\GraspLfD\GraspLfD.ttt'

if (nargin < 1)
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    simulationNb=19991; %Change according to the simlation being ran to record demo
    id=vrep.simxStart('127.0.0.1',simulationNb,true,true,5000,5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
end

%% Initialise Data Structs
switch simulationNb
    case 19991
        frame_names = {'Marker_Blue', 'Marker_Green', 'Frame0'};
        point_names = {'Tip'};
    otherwise
        frame_names = {};
        point_names = {};
end
frames = struct([]);
points = struct([]);

for i = 1:length(frame_names)
    [res, handle]=vrep.simxGetObjectHandle(id, frame_names{i},vrep.simx_opmode_blocking);
    frames(i).name=frame_names(i); frames(i).handle = handle;
end

for i = 1:length(point_names)
    [res, handle]=vrep.simxGetObjectHandle(id, point_names{i},vrep.simx_opmode_blocking);
    points(i).name=point_names(i); points(i).handle = handle;
end

%% Initialise Demo info
% Paths
file_demoLog= sprintf('demo_grasp_%s.mat', datestr(now,'yyyymmddTHHMM'));
path_demoLog = ('C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Demonstrations/');
file_demoVideo = sprintf('demo_grasp_%s.avi', datestr(now,'yyyymmddTHHMM'));

%Parameters
samplingRate=10; %per second

%Video
[returnCode, camera]=vrep.simxGetObjectHandle(id,'Vision_sensor',vrep.simx_opmode_blocking);
outputVideo = VideoWriter(strcat(path_demoLog,file_demoVideo));
outputVideo.FrameRate = samplingRate;
open(outputVideo);


%% Record
t=0; demoInProgress=1;

returnCode= vrep.simxSetIntegerSignal(id, 'demoInProgress', 1,vrep.simx_opmode_oneshot);
while demoInProgress ~=2
    t=t+1
    for i = 1:length(frames)
        [returnCode, frames(i).pos(t,:)]=vrep.simxGetObjectPosition(id,frames(i).handle,-1,vrep.simx_opmode_streaming);
        [returnCode, frames(i).orient(t,:)]=vrep.simxGetObjectOrientation(id,frames(i).handle,-1,vrep.simx_opmode_buffer);
    end
    for i=1:length(points)
        [returnCode, points(i).pos(t,:)]=vrep.simxGetObjectPosition(id,points(i).handle,-1,vrep.simx_opmode_buffer);
        [returnCode, points(i).orient(t,:)]=vrep.simxGetObjectOrientation(id,points(i).handle,-1,vrep.simx_opmode_buffer);
    end
    [returnCode, resolution, image]=vrep.simxGetVisionSensorImage2(id, camera, 0, vrep.simx_opmode_oneshot_wait);
    writeVideo(outputVideo,image);
    [returnCode, demoInProgress]= vrep.simxGetIntegerSignal(id, 'demoInProgress', vrep.simx_opmode_oneshot);
    pause(1/samplingRate);
end
vrep.simxStopSimulation(id,vrep.simx_opmode_oneshot);
%% Save Log
save(strcat(path_demoLog,file_demoLog),'frames','points','t');
close(outputVideo);
end
