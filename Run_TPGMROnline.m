function Run_TPGMR01
addpath(genpath('C:/Users/elzaatas/Documents/Matlab-Vrep/'));

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% model.nbStates = 3; %Number of Gaussians in the GMM
% model.nbFrames = 2; %Number of candidate frames of reference
% model.nbVar = 3; %Dimension of the datapoints in the dataset (here: t,x1,x2)
% model.dt = 1E+2; %Time step duration
% model.params_diagRegFact = 1E-8; %Optional regularization term
% nbData = 200; %Number of datapoints in a trajectory
nbRepros = 3; %Number of reproductions with new situations randomly generated


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% s(n).Data0 is the n-th demonstration of a trajectory of s(n).nbData datapoints, with s(n).p(m).b and 's(n).p(m).A describing
% the context in which this demonstration takes place (position and orientation of the m-th candidate coordinate system)
load('C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Demonstrations/DataGMM01.mat'); %REPLACE THIS WITH OWN DATA OF SIMILAR DIMENSIONS
nbData=model.nbperDemo;
nbSamples=model.nbSamples;
nbSamples=2;
%% Observations from the perspective of each candidate coordinate system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data' contains the observations in the different coordinate systems: it is a 3rd order tensor of dimension D x P x N,
% with D=3 the dimension of a datapoint, P=2 the number of candidate frames, and N=TM the number of datapoints in a
% trajectory (T=200) multiplied by the number of demonstrations (M=5)
%
% Data = zeros(model.nbVar, model.nbFrames, nbSamples*nbData);
% for n=1:nbSamples
% 	for m=1:model.nbFrames
% 		Data(:,m,(n-1)*nbData+1:n*nbData) = s(n).p(m).A \ (s(n).Data0 - repmat(s(n).p(m).b, 1, nbData));
% 	end
% end

%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
%model = init_tensorGMM_kmeans(Data, model);
 model = init_tensorGMM_timeBased(Data, model);
model = EM_tensorGMM(Data, model);

%Precomputation of covariance inverses
for m=1:model.nbFrames
    for i=1:model.nbStates
        model.invSigma(:,:,m,i) = inv(model.Sigma(:,:,m,i));
    end
end

%% Connect API
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

%initialise
if model.nbFrames ==2
    frame_names = {'Marker_Blue', 'Frame0'};
else
    frame_names = {'Marker_Blue', 'Marker_Green', 'Frame0'};
end
frames = struct([]);
for i = 1:length(frame_names)
    [res, handle]=vrep.simxGetObjectHandle(id, frame_names{i},vrep.simx_opmode_blocking);
    frames(i).name=frame_names(i); frames(i).handle = handle;
    [returnCode, pos]=vrep.simxGetObjectPosition(id,frames(i).handle,-1,vrep.simx_opmode_streaming);
    [returnCode, orient]=vrep.simxGetObjectOrientation(id,frames(i).handle,-1,vrep.simx_opmode_streaming);
end
[res, target]=vrep.simxGetObjectHandle(id, 'Tip',vrep.simx_opmode_blocking);

%% Reproductions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with GMR...');
DataIn(1,:) = 1:nbData; %1:nbData;
in = 1;
out = 2:model.nbVar;
MuGMR = zeros(length(out), nbData, model.nbFrames);
SigmaGMR = zeros(length(out), length(out), nbData, model.nbFrames);
orientfeaturesin=model.orientfeatin;
%Gaussian mixture regression
for m=1:model.nbFrames
    %Compute activation weights
    for i=1:model.nbStates
        H(i,:) = model.Priors(i) * gaussPDF(DataIn, model.Mu(in,m,i), model.Sigma(in,in,m,i));
    end
    H = H ./ (repmat(sum(H),model.nbStates,1)+realmin);
    
    for t=1:nbData
        %Compute conditional means
        for i=1:model.nbStates
            MuTmp(:,i) = model.Mu(out,m,i) + model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * (DataIn(:,t) - model.Mu(in,m,i));
            MuGMR(:,t,m) = MuGMR(:,t,m) + H(i,t) * MuTmp(:,i);
        end
        %Compute conditional covariances
        for i=1:model.nbStates
            SigmaTmp = model.Sigma(out,out,m,i) - model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * model.Sigma(in,out,m,i);
            SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
        end
        SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) - MuGMR(:,t,m) * MuGMR(:,t,m)' + eye(length(out)) * model.params_diagRegFact;
    end
end

while true
    disp('iteration')
    MuTmp = zeros(length(out), nbData, model.nbFrames);
    SigmaTmp = zeros(length(out), length(out), nbData, model.nbFrames);
    s=struct();
    
    %Set context parameters (read new frame params)
    for nbframe = 1:length(frames)
        [returnCode, pos]=vrep.simxGetObjectPosition(id,frames(nbframe).handle,-1,vrep.simx_opmode_streaming);
        [returnCode, orient]=vrep.simxGetObjectOrientation(id,frames(nbframe).handle,-1,vrep.simx_opmode_streaming);
        pTmp(nbframe).A=eye(length(Data(:,1,1)));
        Rotf_g=[eul2rotm([0,0,orient(1)])*eul2rotm([0,orient(2),0])*eul2rotm([orient(3),0,0])];
        
        if orientfeaturesin ==true
            pTmp(nbframe).b=[0,pos,[0 0 0 0 0 0 0 0 0]]';
            pTmp(nbframe).A(2:4,2:4)=Rotf_g;
            pTmp(nbframe).A(5:7,5:7)=Rotf_g;
            pTmp(nbframe).A(8:10,8:10)=Rotf_g;
            pTmp(nbframe).A(11:13,11:13)=Rotf_g;
        else
            pTmp(nbframe).b=[0,pos]';
            pTmp(nbframe).A(2:4,2:4)=Rotf_g;
        end

    end
    r.p = pTmp;
    
    %Linear transformation of the retrieved Gaussians
    for m=1:model.nbFrames
        MuTmp(:,:,m) = pTmp(m).A(2:end,2:end) * MuGMR(:,:,m) + repmat(pTmp(m).b(2:end),1,nbData);
        for t=1:nbData
            SigmaTmp(:,:,t,m) = pTmp(m).A(2:end,2:end) * SigmaGMR(:,:,t,m) * pTmp(m).A(2:end,2:end)';
        end
    end
    
    %Product of Gaussians (fusion of information from the different coordinate systems)
    for t=1:nbData
        SigmaP = zeros(length(out));
        MuP = zeros(length(out), 1);
        for m=1:model.nbFrames
            
            SigmaP = SigmaP + inv(SigmaTmp(:,:,t,m));
            MuP = MuP + SigmaTmp(:,:,t,m) \ MuTmp(:,t,m);
        end
        r.Sigma(:,:,t) = inv(SigmaP);
        r.Data(:,t) = r.Sigma(:,:,t) * MuP;
    end
    
    data=r;
    relativeto=-1;
    %%%%%%%%%% 
    %Actually replicating one of the demos to see if data is recorded
    %properly data.Data(nboffeatures, nboftime)
%     data=testdemodata;relativeto=frames(1).handle;
    %%%%%%%%%%

    for i=1:length(data.Data(1,:))
        pos=data.Data(1:3,i);
        if orientfeaturesin ==true
            %orient=data.Data(4:6,i);
            orient=rotm2eul([data.Data(4:6,i),data.Data(7:9,i),data.Data(10:12,i)],'XYZ')
            [returnCode]=vrep.simxSetObjectOrientation(id,target,relativeto,orient,vrep.simx_opmode_blocking)
        end
        %disp('moving');
        [returnCode]=vrep.simxSetObjectPosition(id,target, relativeto, pos,vrep.simx_opmode_blocking);
    end
    pause(10);    
end

end

