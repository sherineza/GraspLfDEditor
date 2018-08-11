% Main Options
clear all;

%% Setting up Api connection
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);
simulationNb=19991;
id=vrep.simxStart('127.0.0.1',simulationNb,true,true,5000,5);

if id < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

%% Initialisations

%%  Understanding environment
markers = Run_DetectMarkers(id,vrep);

%% Demonstrate (for now move in predetermined position and log data)
%Run_RecordDemo(id, vrep,simulationNb);
%% LfD from logged data

%% Representing learnt data

%% Edit learnt data and resave as general policy

