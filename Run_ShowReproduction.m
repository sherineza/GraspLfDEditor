function Run_ShowReproduction()
%This function is show the reproduction from the GMR in V-rep. I.e.
%transfer the coordinates of the new frames and also move the tip of the
%robot according to inverse kinematics and the points of the trajectory
%that are reproduced.
addpath(genpath('C:/Users/elzaatas/Documents/Matlab-Vrep'));

%% Conenct to V-rep
if (nargin < 1) %in the case when the id and v-rep may be given as input
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    simulationNb=19992;
    id=vrep.simxStart('127.0.0.1',simulationNb,true,true,5000,5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
end

path_reprod='C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Reproductions/';
files = dir(strcat(path_reprod,'*.mat'));
filenb=length(files);
load(strcat(path_reprod,files(filenb).name)); %return the first file. It contains several reproductions.
disp(strcat('reproducing file: ',files(filenb).name));
%Choosing the row out of the many reproductions
nbReprods= length(r);
for I=1:nbReprods
    %% Load reproduction data (r) from the output of the GMR
    data=r(I);
    
    
    %% Get Handles
    frame_names = {'Marker_Blue', 'Marker_Green','Frame0'};
    frames = struct([]);
    for i = 1:length(frame_names)
        [res, handle]=vrep.simxGetObjectHandle(id, frame_names{i},vrep.simx_opmode_blocking);
        frames(i).name=frame_names(i); frames(i).handle = handle;
    end
    [res, target]=vrep.simxGetObjectHandle(id, 'BarrettHand',vrep.simx_opmode_blocking);
    %% Initialise positions
    %the dummy's need to be attached to the parents in the V-rep script so that
    %the full objects move.
    for i=1:length(frames)
        pTmp=data.p(i);
        [returnCode]=vrep.simxSetObjectPosition(id,frames(i).handle,-1, pTmp.b(2:4),vrep.simx_opmode_blocking);
        %         orient=SpinCalc('DCMtoEA123',pTmp.A(2:4,2:4),0.1,0);
        orient=pTmp.b(end-2:end);
        [returnCode]=vrep.simxSetObjectOrientation(id,frames(i).handle,-1,orient,vrep.simx_opmode_blocking);
    end
    
    %% Passing on joint movements according to the trajectory generated
    for i=1:length(data.Data(1,:))
        pos=data.Data(1:3,i);
                orient=data.Data(4:6,i);
%                 [returnCode]=vrep.simxSetObjectOrientation(id,target,-1,orient,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetObjectPosition(id,target,-1, pos,vrep.simx_opmode_blocking);
        %pause(0.5);i
    end
    
    
end
disp('end')
end

