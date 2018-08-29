function Run_demoLog2GMMinput
% This function transforms the data saved from the demonstration to input
% data for the GMM learning function.
%% Get demo Log file names
path_demoLog=('C:/Users/elzaatas/Documents/Matlab-Vrep/GraspLfD/Demonstrations/');
filenames = dir(strcat(path_demoLog,'demo_RRgraspfix_fast_201808*.mat'));
filenames={filenames.name};

%% Load data + manipulate
orientfeaturesin =1; %0 for only position; 1 for pos+orient(total 12); 2 for pos+pos..(total 12); 3 for pos+rand(total 4)

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
            Rotf_g1=[eul2rotm([orient(3),0,0])*eul2rotm([0,orient(2),0])*eul2rotm([0,0,orient(1)])];
            Rotf_g1=SpinCalc('EA123toDCM',orient.*180/3.1415,0.1,0);
            %Homf_g=[[Rotf_g,[frames(nbframe).pos(j,:)]'];[0,0,0,1]];
            
            orientp=points.orient(j,:);
            Rotp_g=[eul2rotm([0,0,orientp(1)])*eul2rotm([0,orientp(2),0])*eul2rotm([orientp(3),0,0])];
            %Rotp_g=[eul2rotm([orientp(3),0,0])*eul2rotm([0,orientp(2),0])*eul2rotm([0,0,orientp(1)])];
            
            %Homp_g=[[Rotp_g,[points.pos(j,:)]'];[0,0,0,1]];
            
            % [time, posx,posy,posz,orientalpha, orientbeta, orientgamma]
%             relative_orient=inv(Rotf_g)*Rotp_g;
%             relative_orient=rotm2eul(relative_orient,'XYZ');
            relative_pos=Rotf_g*(points.pos(j,:)-frames(nbframe).pos(j,:))';
            relative_orient=[inv(Rotf_g1)*Rotp_g*[1,0,0]';inv(Rotf_g1)*Rotp_g*[0,1,0]';inv(Rotf_g1)*Rotp_g*[0,0,1]'];
            if orientfeaturesin ==1
                data_temp(:,nbframe) = [j;relative_pos(1:3);relative_orient(1:9)];
            elseif orientfeaturesin ==0       
                data_temp(:,nbframe) = [j;relative_pos(1:3)];
            elseif orientfeaturesin ==2
                data_temp(:,nbframe) = [j;relative_pos(1:3);relative_pos(1:3);relative_pos(1:3);relative_pos(1:3)];
            elseif orientfeaturesin ==3
                data_temp(:,nbframe) = [j;relative_pos(1:3);rand(1,3)];
            end
            
        end
        Data(:,:,j,i)=data_temp;
    end
    % Saving a sample demo
    testdemodata.Data=Data(2:end,1,2:end,i)
    
    %For data of frames:
    %Struct s contains n rows (n is number of demonstrations). Each row
    %contains struct p with m rows (m being number of frames). Each row
    %contains A 3x3 matrix for orientation and b 3x1 vector for position.
    for nbframe=1:length(frames)
        s(i).p(nbframe).A=eye(length(Data(:,1,1)));
        orient=frames(nbframe).orient(10,:);
        Rotf_g1=[eul2rotm([0,0,orient(1)])*eul2rotm([0,orient(2),0])*eul2rotm([orient(3),0,0])];
        Rotf_g=[eul2rotm([orient(3),0,0])*eul2rotm([0,orient(2),0])*eul2rotm([0,0,orient(1)])];
        Rotf_g1=SpinCalc('EA123toDCM',orient.*180/3.1415,0.1,0);
        
        if orientfeaturesin ==1
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:),[0 0 0 0 0 0 0 0 0]];
            s(i).p(nbframe).A(2:4,2:4)= Rotf_g1;
            s(i).p(nbframe).A(5:7,5:7)=Rotf_g1;
            s(i).p(nbframe).A(8:10,8:10)=Rotf_g1;
            s(i).p(nbframe).A(11:13,11:13)=Rotf_g1;
        elseif orientfeaturesin ==0
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:)]';
            s(i).p(nbframe).A(end-2:end,end-2:end)=Rotf_g1;
        elseif orientfeaturesin ==2
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:),frames(nbframe).pos(10,:),frames(nbframe).pos(10,:),frames(nbframe).pos(10,:)];
            s(i).p(nbframe).A(2:4,2:4)= Rotf_g1;
            s(i).p(nbframe).A(5:7,5:7)=Rotf_g1;
            s(i).p(nbframe).A(8:10,8:10)=Rotf_g1;
            s(i).p(nbframe).A(11:13,11:13)=Rotf_g1;
        elseif orientfeaturesin ==3
            s(i).p(nbframe).b=[0,frames(nbframe).pos(10,:),0]';
            s(i).p(nbframe).A(end-2:end,end-2:end)=Rotf_g1;            
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
        Data(1,1,:,i)=1:nbperDemo;Data(1,2,:,i)=1:nbperDemo;%Data(1,3,:,i)=1:nbperDemo;
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
save(strcat(path_demoLog,'DataGMM01'),'Data','model','s', 'testdemodata');

end