function Run_demoLog2GMMinput
% This function transforms the data saved from the demonstration to input
% data for the GMM learning function.
%% Get demo Log file names
path_demoLog=('C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Demonstrations/');
filenames = dir(strcat(path_demoLog,'demo_grasp_201808*.mat'));
filenames={filenames.name};

%% Load data + manipulate
orientfeaturesin =true;

Data =[];s=struct();s.p=struct();nbTimeSteps=[];
for i=1:length(filenames)
    load(strcat(path_demoLog,filenames{i}));
    nbTimeSteps(i)=t;
    
    
    
    %For data of moving points:
    %Data should be n x m x g matrix: n is number of dimension of data
    %including the time stamp; m is number of frames. so we have
    %dimensions relative to each frame; g is number of TxD, T being the
    %number of time steps in each demo and D number of Demonstrations
    
    for j =2:t
        data_temp = [];
        for  nbframe = 1:length(frames)
            
            orient=frames(nbframe).orient(j,:);
            Rotf_g=[eul2rotm([0,0,orient(1)])*eul2rotm([0,orient(2),0])*eul2rotm([orient(3),0,0])];
            Homf_g=[[Rotf_g,[frames(nbframe).pos(j,:)]'];[0,0,0,1]];
            
            orientp=points.orient(j,:);
            Rotp_g=[eul2rotm([0,0,orientp(1)])*eul2rotm([0,orientp(2),0])*eul2rotm([orientp(3),0,0])];
            Homp_g=[[Rotp_g,[points.pos(j,:)]'];[0,0,0,1]];
            
            relative_pos=Rotf_g*(points.pos(j,:)-frames(nbframe).pos(j,:))';
            
            
            % [time, posx,posy,posz,orientalpha, orientbeta, orientgamma]
            relative_orient=Rotp_g*inv(Rotf_g);
            relative_orient=rotm2eul(relative_orient);
            relative_orient=SpinCalc('EA321toEA123',relative_orient./3.14*180,1,0);
            if orientfeaturesin ==true
                data_temp(:,nbframe) = [j;relative_pos(1:3);relative_orient(1:3)'./180*3.1415];
            else
                
                data_temp(:,nbframe) = [j;relative_pos(1:3)];
            end
            
        end
        Data(:,:,j,i)=data_temp;
    end
    
    %For data of frames:
    %Struct s contains n rows (n is number of demonstrations). Each row
    %contains struct p with m rows (m being number of frames). Each row
    %contains A 3x3 matrix for orientation and b 3x1 vector for position.
    for nbframe=1:length(frames)
        s(i).p(nbframe).A=eye(length(Data(:,1,1)));
        orient=frames(nbframe).orient(10,:);
        Rotf_g=[eul2rotm([0,0,orient(1)])*eul2rotm([0,orient(2),0])*eul2rotm([orient(3),0,0])];
        
        if orientfeaturesin ==true
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:),orient]';
            s(i).p(nbframe).A(2:4,2:4)=Rotf_g;
            %              s(i).p(nbframe).A(5:end,5:end)=Rotf_g;
        else
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:)]';
            s(i).p(nbframe).A(end-2:end,end-2:end)=Rotf_g;
        end
    end
    
end

% Making all demonstrations have same length, length of the smallest
% demonstration or interpolating to the longest length
shorten_demos=false;
if shorten_demos == true
    nbperDemo = min(nbTimeSteps);
    % nbperDemo=10; %JUST ADDED THIS TO CHECK IF last POSITION IS REGESTERED WELL.A: it isn't;
    DataGMM01=[];
    for i=1:2:length(filenames)
        DataGMM01= cat(3,DataGMM01,Data(:,:,1+(nbTimeSteps(i)-nbperDemo):nbTimeSteps(i),i));
    end
    Data=DataGMM01;
else
    nbperDemo=max(nbTimeSteps);
    DataGMM01=[];
    for i=1:length(filenames)
        diff=nbperDemo-nbTimeSteps(i);
        for j=1:diff
            indx=ceil(rand()*(nbTimeSteps(i)-1))+1;
            Data(:,:,indx+1:end,i)=Data(:,:,indx:end-1,i);
            Data(:,:,indx,i)=(Data(:,:,indx-1,i)+Data(:,:,indx+1,i))/2;%Interpolating step
        end
        Data(1,1,:,i)=1:nbperDemo;Data(1,2,:,i)=1:nbperDemo;Data(1,3,:,i)=1:nbperDemo;
        DataGMM01=cat(3,DataGMM01,Data(:,:,2:nbperDemo,i));
    end
    
    Data=DataGMM01;
end
%% Generating the model struct of parameters
model=struct();
model.nbStates=3;
model.nbVar=length(Data(:,1,1));
model.nbFrames=length(frames);
model.dt=0.01;
model.params_diagRegFact=1e-8;
model.nbperDemo=nbperDemo;
model.nbSamples=length(filenames);
model.orientfeatin=orientfeaturesin;

%% Save GMM input
save(strcat(path_demoLog,'DataGMM01'),'Data','model','s');

end