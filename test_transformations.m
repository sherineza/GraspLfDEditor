%orientation vectors are in XYZ order
framepos=[-0.5252,0.0758,0.8]';
orient=[1.5708,-0.648,1.5708]';
tippos=[-0.5611,0.028,0.8244]';
orientp=[-2.72,0.323,2.189]';

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
 %Rotf_g=[eul2rotm([orient(3),0,0])*eul2rotm([0,orient(2),0])*eul2rotm([0,0,orient(1)])];
Rot21=SpinCalc('EA123toDCM',orient'.*180/3.1415,0.1,0);
sub=Rotf_g-Rot21

% orient=[0,0,-0.349];
 Rotf_g2=eul2rotm(orient');
Homf_g=[[Rotf_g,[framepos];[0,0,0,1]]];

tiptoframe=[0.024361848831176758, 0.00041744112968444824, 0.059914588928222656];
orientpf=[3.0940303802490234, -1.044482946395874, -3.1403391361236572];

Answer=Rotf_g*(tippos-framepos)
Answer=Rot21*(tippos-framepos);

%% calculating orientation
relative_orient=inv(Rotf_g)*Rotp_g;
relative_orient=Rotp_g*inv(Rotf_g);
relative_orient=rotm2eul(relative_orient).*(180/3.1415);
answer=SpinCalc('EA321toEA123',relative_orient,1,0).*(3.14/180);
Rot21=SpinCalc('EA123toDCM',orient'.*180/3.1415,0.1,0)
Rot31=SpinCalc('EA123toDCM',orientp'.*180/3.1415,0.1,0)
%answer=Rotp_g*inv(Rotf_g);
answer=inv(Rot21)*Rot31
answer=SpinCalc('DCMtoEA123',answer,0.1,0).*(3.14/180)
%% new simpler example 
eul1=[0,0,0]; %XYZ
eul21=[0,0,-90]; %XYZ
eul31=[90,-89.9,0];%XYZ
eul32=[90,0,0];%XYZ
Rot21=SpinCalc('EA123toDCM',eul21,0.1,0);
Rot31=SpinCalc('EA123toDCM',eul31,0.1,0);
Rot32=Rot31*inv(Rot21);
answer=SpinCalc('DCMtoEA123',Rot32,0.1,0);
answer=Rot21*eul31';

%% new sample from v-rep
orientp_f=[-2.7454957962036133, -0.00031022191978991032, 0.00034181162482127547];
orientp_g=[-1.5717034339904785, -1.2218291759490967, -1.5713067054748535];
orientf_g=[1.5707964897155762, 0.82573246955871582, 1.5707962512969971];
Rot21=SpinCalc('EA123toDCM',orientf_g.*180/3.14,0.1,0);
Rot31=SpinCalc('EA123toDCM',orientp_g.*180/3.14,0.1,0);
Rot32=Rot31*inv(Rot21);
answer=SpinCalc('DCMtoEA123',Rot32,0.1,0)./180*3.14
%Didn't work
%% Try with quaternions
quatp_f=[0.98045241832733154, -0.00013704596494790167, -0.00018570569227449596, -0.19675621390342712]
quatp_g=[-0.1228916123509407, -0.69645726680755615, -0.12261532247066498, 0.69628328084945679]
quatf_g=[-0.65860551595687866, 0.25736904144287109, -0.65860545635223389, -0.25736925005912781]

%% Try with matrices

mat=reshape([5.9604644775390625e-08, -0.67801892757415771, 0.73504459857940674, -0.048860318958759308, 2.384185791015625e-07, -0.73504471778869629, -0.67801892757415771, -0.14279952645301819, 1.0000001192092896, 2.9802322387695313e-07, -1.1920928955078125e-07, 0.80010998249053955],[4,3]);
matf_g=[mat';[0,0,0,1]];
mat=reshape([-0.00017452239990234375, 0.34192749857902527, -0.93972635269165039, -0.47006773948669434, 0.00042751431465148926, 0.93972629308700562, 0.34192740917205811, -0.33095651865005493, 0.99999988079071045, -0.00034208595752716064, -0.00031018257141113281, 1.7014844417572021],[4,3]);
matp_g=[mat';[0,0,0,1]];
mat=reshape([0.99999988079071045, -0.00034181159571744502, -0.00031022191978991032, 0.90137439966201782, -0.00019565658294595778, -0.92257404327392578, 0.38582026958465576, 0.42389065027236938, -0.00041808048263192177, -0.38582015037536621, -0.92257392406463623, -0.18203216791152954],[4,3]);
matp_f=[mat';[0,0,0,1]];
orientp_f=[-2.7454957962036133, -0.00031022191978991032, 0.00034181162482127547];
posp_f=[0.90137439966201782, 0.42389065027236938, -0.18203216791152954];

matanswer=inv(matf_g)*matp_g
answer1=rotm2eul(matanswer(1:3,1:3),'XYZ')

