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
Data =[];s=struct();s.p=struct();
for i=1:length(filenames)
    load(strcat(path_demoLog,filenames{i}));
    nbTimeSteps(i)=t;
    
    %For data of moving points:
    %Data should be n x m x g matrix: n is number of dimension of data
    %including the time stamp; m is number of frames. so we have
    %dimensions relative to each frame; g is number of TxD, T being the
    %number of time steps in each demo and D number of Demonstrations
    
    for j =1:t
        data_temp = [];
        for  nbframe = 1:length(frames)
            orient=[0,0,1;0,1,0;1,0,0]*frames(nbframe).orient(j,:)';
            TransMat = [[eul2rotm(orient'), frames(nbframe).pos(j,:)'];[0 0 0 1]];
            relative_pos=TransMat*[points.pos(j,:),1]';
            relative_orient=TransMat*[points.orient(j,:),1]';
            data_temp(:,nbframe) = [j;relative_pos(1:3);relative_orient(1:3)];
        end
        Data(:,:,j,i)=data_temp;
    end
    
    %For data of frames:
    %Struct s contains n rows (n is number of demonstrations). Each row
    %contains struct p with m rows (m being number of frames). Each row
    %contains A 3x3 matrix for orientation and b 3x1 vector for position.
    for nbframe=1:length(frames)
        s(i).p(nbframe).A=eye(7);
        orient=[0,0,1;0,1,0;1,0,0]*frames(nbframe).orient(10,:)';
        rotmat=eul2rotm(orient');
        s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:),0,0,0]';
        s(i).p(nbframe).A(5:end,5:end)=rotmat;
    end
end

% Making all demonstrations have same length, length of the smallest
% demonstration.
shorten_demos=false;
if shorten_demos == true
    
    nbperDemo = min(nbTimeSteps);
    % nbperDemo=10; %JUST ADDED THIS TO CHECK IF last POSITION IS REGESTERED WELL.A: it isn't;
    DataGMM01=[];
    for i=1:length(filenames)
        DataGMM01= cat(3,DataGMM01,Data(:,:,1+(nbTimeSteps(i)-nbperDemo):nbTimeSteps(i),i));
    end
    Data=DataGMM01;
else
    nbperDemo=max(nbTimeSteps);
    DataGMM01=[];
    for i=1:length(filenames)
        diff=nbperDemo-nbTimeSteps(i);
        for j=1:diff
            indx=ceil(rand()*nbTimeSteps(i));
            Data(:,:,indx+1:end,i)=Data(:,:,indx:end-1,i);
        end
        DataGMM01=cat(3,DataGMM01,Data(:,:,2:nbperDemo,i));
    end
    Data=DataGMM01;
end
%% Generating the model struct of parameters
model=struct();
model.nbStates=3;
model.nbVar=7;
model.nbFrames=3;
model.dt=0.01;
model.params_diagRegFact=1e-8;
model.nbperDemo=nbperDemo;
model.nbSamples=length(filenames);
%% Save GMM input
save(strcat(path_demoLog,'DataGMM01'),'Data','model','s');

end