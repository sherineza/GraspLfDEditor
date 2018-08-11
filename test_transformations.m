%orientation vectors are in XYZ order
framepos=[-0.30000019073486328, -0.3750004768371582, 0.70000004768371582]';
orient=[0, 0, 0.34906601905822754]';
tippos=[-0.50202029943466187, -0.30823945999145508, 1.7014809846878052]';
orientp=[-1.5717028379440308, -1.2218291759490967, -1.5713059902191162]';

Rotp_g=[eul2rotm([0,0,orientp(1)])*eul2rotm([0,orientp(2),0])...
    *eul2rotm([orientp(3),0,0])];
% Rotp_g=eul2rotm([[0,0,1;0,1,0;1,0,0]*orientp]');
% Rotp_g=[eul2rotm([orientp(3),0,0])*eul2rotm([0,orientp(2),0])...
%     *eul2rotm([0,0,orientp(1)])];
Rotp_g2=eul2rotm(orientp');
%ASSUMING ZYX
% orientp=[1.89,-0.24,-2.517];
% Rotp_g=eul2rotm(orientp);
% Homp_g=[[Rotp_g,[tippos]];[0,0,0,1]];

Rotf_g=[eul2rotm([0,0,orient(1)])*eul2rotm([0,orient(2),0])...
    *eul2rotm([orient(3),0,0])];
% Rotf_g=eul2rotm([[0,0,1;0,1,0;1,0,0]*orient]');
% Rotf_g=[eul2rotm([orient(3),0,0])*eul2rotm([0,orient(2),0])...
%     *eul2rotm([0,0,orient(1)])];

% orient=[0,0,-0.349];
 Rotf_g2=eul2rotm(orient');
Homf_g=[[Rotf_g,[framepos];[0,0,0,1]]];

tiptoframe=[-0.21267041563987732, -0.0063601136207580566, 1.0014810562133789];
orientpf=[0, -1.5707963705062866, 0.0003420801367610693];

Answer=Rotf_g*(tippos-framepos)
%% calculating orientation
relative_orient=inv(Rotf_g)*Rotp_g;
relative_orient=Rotp_g*inv(Rotf_g);
relative_orient=rotm2eul(relative_orient).*(180/3.1415);
answer=SpinCalc('EA321toEA123',relative_orient,1,0).*(3.14/180)

answer=Rotp_g*inv(Rotf_g)*orientp

%% new simpler example 
eul1=[0,0,0]; %XYZ
eul21=[0,0,-90]; %XYZ
eul31=[90,-89.9,0];%XYZ
eul32=[90,0,0];%XYZ
Rot21=SpinCalc('EA123toDCM',eul21,0.1,0);
Rot31=SpinCalc('EA123toDCM',eul31,0.1,0);
Rot32=Rot31*inv(Rot21);
answer=SpinCalc('DCMtoEA123',Rot32,0.1,0)
answer=Rot21*eul31'