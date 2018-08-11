function Run_TPGMR01
addpath(genpath('C:/Users/elzaatas/Documents/Matlab-Vrep/pbdlib-matlab-master-7ea05b8c6bee2284a79727ca086b0719dcb76b65'));

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
load('./Demonstrations/DataGMM01.mat'); %REPLACE THIS WITH OWN DATA OF SIMILAR DIMENSIONS
nbData=model.nbperDemo;
nbSamples=model.nbSamples;
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

%% Reproductions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with GMR...');
DataIn(1,:) = 1:nbData; %1:nbData;
in = 1;
out = 2:model.nbVar;
MuGMR = zeros(length(out), nbData, model.nbFrames);
SigmaGMR = zeros(length(out), length(out), nbData, model.nbFrames);
		
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

for n=1:nbSamples+nbRepros
	MuTmp = zeros(length(out), nbData, model.nbFrames);
	SigmaTmp = zeros(length(out), length(out), nbData, model.nbFrames);
	
	%Set context parameters
	if n<=nbSamples
		%Reproductions for the task parameters used to train the model
		pTmp = s(n).p;
	else
		%Reproductions for new random task parameters
		for m=1:model.nbFrames
			id = ceil(rand(2,1)*nbSamples);
			w = rand(2); 
			w = w / sum(w);
			pTmp(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
			pTmp(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
		end
	end
	r(n).p = pTmp;

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
		r(n).Sigma(:,:,t) = inv(SigmaP);
		r(n).Data(:,t) = r(n).Sigma(:,:,t) * MuP;
	end
end

%% Save reproductions
save(strcat('Reproductions/reprod_grasp_',datestr(now,'yyyymmddTHHMM'),'.mat'),'r');
end

