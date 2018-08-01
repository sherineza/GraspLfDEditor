function Run_demoLog2GMMinput
% This function transforms the data saved from the demonstration to input
% data for the GMM learning function.
%% Get demo Log file names
path_demoLog=('C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Demonstrations/');
filenames = dir(strcat(path_demoLog,'demo_grasp_*.mat'));
filenames={filenames.name};

% filenames_demoLog = {'demo_grasp_20180731T1220.mat','demo_grasp_20180730T1622.mat',...
%     'demo_grasp_20180730T1621.mat', 'demo_grasp_20180730T1620.mat',...
%     'demo_grasp_20180730T1618.mat'}
%% Load data + manipulate
Data =[];
    for i=1:length(filenames)
         load(strcat(path_demoLog,filenames{i})); 
         for j =1:t
             data_temp = [];
             for  nbframe = 1:length(frames)
                 TransMat = [[eul2rotm(frames(nbframe).orient(j,:)), frames(nbframe).pos(j,:)'];[0 0 0 1]];
                 relative_pos=TransMat*[points.pos(j,:),1]';
                 relative_orient=TransMat*[points.orient(j,:),1]';
                 data_temp(:,nbframe) = [j;relative_pos(1:3);relative_orient(1:3)]; 
             end
             Data(:,:,end+1)=data_temp;
         end
         % Manipulation

         %Data should be n x m x g matrix: n is number of dimension of data
         %including the time stamp; m is number of frames. so we have
         %dimensions relative to each frame; g is number of TxD, T being the
         %number of time steps in each demo and D number of Demonstrations
    end
 Data=Data(:,:,2:end);
 
 model=struct();
 model.nbStates=3;
 model.nbVar=7;
 model.nbFrames=3;
 model.dt=0.01;
 model.params_diagRegFact=1e-8;
 
 %% Save GMM input
 save(strcat(path_demoLog,'DataGMM01'),'Data','model');

end