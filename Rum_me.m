clear;
clc;
close all;
tic
disp('▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓')
disp(' _______________ _____ _____ _____')
disp(' / ____/ ____/ | |__ // ___// ___/')
disp(' / __/ / / __/ /| |______ /_ </ __ \/ __ \ ')
disp(' / /___/ /_/ / ___ /_____/__/ / /_/ / /_/ / ')
disp('/_____/\____/_/ _|_| __/____/\____/\____/_____ ____ ______ ')
disp(' / __ \/ __ \/ __ )/ __ \/_ __/ / ____/ __ \/ __ \/ ____/')
disp(' / /_/ / / / / __ / / / / / / / / / / / / / / / __/ ')
disp(' / _, _/ /_/ / /_/ / /_/ / / / / /___/ /_/ / /_/ / /___ ')
disp('/_/ |_|\____/_____/\____/ /_/ \____/\____/_____/_____/ ')
disp('Name = Oliver Spurrell')
disp('Student Number = 982170')
disp('▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓')

%% Load map data and run options
% Check if the necessary data files are available
if ~isfile('predata.mat')
predata();
load predata.mat
else
load predata.mat
end
if ~isfile('mapdata.mat')
mapmaker();
load mapdata.mat
else
load mapdata.mat
end
fprintf('\n IMPORT = DONE\n -----------------------------
')
% Turns off nearly Singular Matrix warning as I find it annoying and given
% the cordinates magnitude its just going to go off
% The warning is turned back on at the end, don't worry :)
warning('off','MATLAB:nearlySingularMatrix')
%% Runs Function list
% Compute the waypoints using the AS_V10 function
WPs =
AS_V10(MAP_D.C,MAP_D.MAP,pre.vn,pre.theta,pre.Tag,pre.t0,pre.v0);
fprintf('\n WAYPOINTS = DONE \n -----------------------------
')
% Generate the robot's trajectory using the waypoints
TRAJ = Traj_V10(WPs,pre.dt,pre.Tag,MAP_D.C);
fprintf('\n TRAJECTORY = DONE \n -----------------------------
')
% Generate the final path for the robot using the trajectory
RPATH =
PATHING(TRAJ,MAP_D.C,pre.Tag,pre.theta,pre.dt,pre.gamma,pre.l);

fprintf('\n PATHING = DONE \n -----------------------------
')
% Visualize the waypoints and trajectory on a map
[f1,f2] = Plots(MAP_D.C,WPs,TRAJ,RPATH,MAP_D.MAP_BOM,pre.Tag);
fprintf('\n PLOTS = DONE \n ')
fprintf('\n\n ----- DONE -----\n\n')
toc
% Turns nearly Singular Matrix warning back on
warning('on','MATLAB:nearlySingularMatrix')
%% Function List
%% 1. Waypoint Maker V10
% 1.0 => Waypoint hub function
function [PATH] = AS_V10(C,MAP_DATA,vn,theta,Tag,t0,v0)
% Extract the map from the MAP_DATA variable
MAP = MAP_DATA(:,:,2);
%% Notes
% AS_V10 - Control functions related to the creation of WPMs
%
% Outputs:
% PATH - A structure array containing the WPMs
% WPMs - [x y t Vx Vy Wd;]
%
%% Function
% Get the size of the map
[H,W]=size(MAP);
% Get the number of primary coordinates (start and end points)
Z = height(C);
% Loop through the primary coordinates
for ic = 1:Z
% Compute the shortest path between the start and end points using the
A* algorithm
PATH.(Tag{1,ic}) = AP(C(ic,1),C(ic,2),MAP,MAP_DATA(:,:,ic+2),H,W);
fprintf('\n Path %i: A* algorithm = DONE',ic)
% Reverse the path to make it start from the end point
PATH.(Tag{1,ic})=flip(PATH.(Tag{1,ic}));
% Compute the time and velocity data for the path
PATH.(Tag{2,ic}) = WP(PATH.(Tag{1,ic}),t0,v0,vn,theta(ic:ic+1));
fprintf('\n Path %i: WPM = DONE',ic)
% Set the initial time for the next path to be 5 seconds after the end
time of the current path
t0 = PATH.(Tag{2,ic})(end,3)+5;
end
end
% 1.1 => A star algorithm
function WPM = AP(X1,Y1,MAP,GRM,H,W)
%% Notes
% AP - Compute the shortest path between the start and end points using
the A* algorithm
%
% Inputs:
% X1 - x-coordinate of the starting point
% Y1 - y-coordinate of the starting point
% MAP - matrix representing the map
% GRM - matrix representing the goal nodes
% H - height of the map
% W - width of the map
%

% Outputs:
% WPM - Array of seconary waypoints for the path
%% Matrix Setup
% Initialize the matrix of g-scores, f-scores, and the open and closed
sets
[GS,FS] = deal(zeros([H,W]),single(inf([H,W])));
[open,closed,Xp,Yp] = deal(zeros([H,W]));
%% Heuristic-matrix
% Initialize the heuristic matrix using the distance to the nearest goal
node
hn = single(zeros([H,W]));
Ns = [-1,-1;0,-1;1,-1;-1,0;1,0;-1,1;0,1;1,1];
% Ns - matrix of relative coordinates of neighbors
for k=1:size(GRM,1)
for j=1:size(GRM,2)
if MAP(k,j)==0
hn(k,j)=(min(sqrt(sum(abs(find(GRM==1)-[j k]).^2,2))));
end
end
end
%% Set the initial f-score and open set
FS(Y1,X1) = hn(Y1,X1);
closed(MAP(:,:,1)==1) = 1;
open(Y1,X1) = 1;
%% Compute the path using the A* algorithm
while true
% Find the cell with the minimum f-score
[YC,XC] = find(FS==min(min(FS)));
[YC,XC] = deal(YC(1),XC(1));
% Check if reached the goal
if GRM(YC,XC)==1
Home=1;break
end
[open(YC,XC),FS(YC,XC),closed(YC,XC)]=deal(0,inf,1);
% Loop through the neighbors of the current cell
for p=1:8
% Compute the coordinates of the neighbor
[i,j,Yn,Xn]=deal(Ns(p,1),Ns(p,2),YC+Ns(p,1),XC+Ns(p,2));
% Skip the neighbor if it is outside the map
if Yn<1||Yn>H||Xn<1||Xn>W
continue
end
% Skip the neighbor if it is in the closed set or is a wall
if(closed(Yn,Xn)==0) && (MAP(Yn,Xn)==0)
% Compute the tentative g-score for the neighbor
t_gS = GS(YC,XC) + hypot(i,j);
if open(Yn,Xn)==0
open(Yn,Xn)=1;
elseif t_gS >= GS(Yn,Xn)
continue
end

[Yp(Yn,Xn),Xp(Yn,Xn),GS(Yn,Xn),FS(Yn,Xn)] =
deal(YC,XC,t_gS,t_gS+hn(Yn,Xn));
end
end
end
%% Brute force the optimal path
% Compute the optimal path using the parent matrix
k=2;
OP(1,:)=[YC XC];
while Home
CXD=Xp(YC,XC);
YC=Yp(YC,XC);
XC=CXD;
OP(k,:)=[YC XC];
k=k+1;
if (((XC== X1)) &&(YC==Y1))
break
end
end
% Reverse the path and convert it to world coordinates
OP(:,1) = 90 - OP(:,1);
WPM=fliplr(OP);
end
% 1.2 => Calculates times and velocities
function WPM = WP(WPM,t0,v0,vn,theta)
% WP - Calculate and update the values in a waypoint matrix
%
% Inputs:
% WPM - The waypoint matrix to update
% t0 - The initial time
% v0 - The initial velocity
% vn - The nominal velocity
% theta - The range of angles
%
% Outputs:
% WPM - The updated waypoint matrix
% Define the number of paths in the waypoint matrix
Ps = height(WPM)-1;
% Set the initial time and velocity
WPM(1,3) = t0;
WPM(1,4:5) = v0;
% Iterate over the number of paths
for p = 1:Ps
% Calculate the distance between the current and next waypoints
D_n = norm(WPM(p+1,1:2)-WPM(p,1:2));
% If this is not the last path, update the velocity for the next
waypoint
if p~=Ps
WPM(p+1,4:5) = vn.*(WPM(p+2,1:2)-WPM(p+1,1:2))/...
norm(WPM(p+2,1:2)-WPM(p+1,1:2));
end
% Update the time for the next waypoint based on the distance and
target velocity
WPM(p+1,3) = WPM(p,3)+(D_n/vn);
end
% Calculate the constant angular velocity based on the angle range and
time duration of the path

W = (theta(2)-theta(1))/(WPM(end,3)-WPM(1,3));
% Set the sixth column of the waypoint matrix to the calculated angular
velocity
WPM(:,6) = W;
end
%% 2. Trajectory calculation
% 2.0 => Trajectory hub function
function [TRAJ] = Traj_V10(WPs,dt,Tag,C)
for ic = 1:height(C)
TRAJ.(Tag{3,ic}) = Traj_calc(WPs.(Tag{2,ic}),dt);
fprintf('\n Path %i: Trajectory = DONE',ic)
end
end
% 2.1 => Smoothed 2D Trajectory algorithm
function [TM]=Traj_calc(WPM,dt)
xt = WPM(1,1); yt = WPM(1,2);
Vxt = WPM(1,4); Vyt = WPM(1,5) ;
tt = WPM(1,3);
Ps = height(WPM)-1;
lc = 1;
for pc = 1:Ps
ax =
t_a(WPM(pc,1),WPM(pc,3),WPM(pc,4),WPM(pc+1,1),WPM(pc+1,3),WPM(pc+1,4));
ay =
t_a(WPM(pc,2),WPM(pc,3),WPM(pc,5),WPM(pc+1,2),WPM(pc+1,3),WPM(pc+1,5));
for t = WPM(pc,3)+dt:dt:WPM(pc+1,3)
lc = lc + 1;
xt(lc) = t_c(ax,t); yt(lc) = t_c(ay,t);
Vxt(lc) = t_s(ax,t); Vyt(lc) = t_s(ay,t);
tt(lc) = tt(lc-1)+dt;
end
end
TM(:,1) = xt;
TM(:,2) = yt;
TM(:,3) = tt;
TM(:,4) = Vxt;
TM(:,5) = Vyt;
TM(:,6) = WPM(5,6);
end
% 2.2 => Trajectory coefficent calculation
function a = t_a(c1,t1,vc1,c2,t2,vc2)
b = [c1;vc1;c2;vc2];
A = [1,t1,t1^2,t1^3,t1^4;...
0,1,2*t1,3*t1^2,4*t1^3;...
1,t2,t2^2,t2^3,t2^4;...
0,1,2*t2,3*t2^2,4*t2^3];
B = [0,0,0,0,0;...
0,0,0,0,0;...
0,0,t2-t1,3*(t2^2-t1^2)/2,2*(t2^3-t1^3);...
0,0,3*(t2^2-t1^2)/2,3*(t2^3-t1^3),9*(t2^4-t1^4)/2;...

0,0,2*(t2^3-t1^3),9*(t2^4-t1^4)/2,36*(t2^5-t1^5)/5];...
C = [eye(5) zeros(5,4)];
M1 = [B,A';A,zeros(4,4)];
V1 = [zeros(5,1);b];
a = C*inv(M1)*V1;
end
% 2.3 => Trajectory cordinate value
function ct = t_c(a,t)
% Function calculates cordinate along a trajectory path
ct = a(1)+ a(2)*t + a(3)*t^2 + a(4)*t^3 + a(5)*t^4;
end
% 2.4 => Trajectory velocity value
function vt = t_s(a,t)
% Function calculates velocities along a trajectory path
vt = a(2) + 2*a(3)*t + 3*a(4)*t^2 + 4*a(5)*t^3;
end
%% 3. Control of swedish wheel
% 3.0 => Control hub function
function [RPATH] = PATHING(TRAJ,C,Tag,theta,dt,gamma,l)
alt = 0;
for ic = 1:height(C)
if ic ~= 1
alt = -5;
end
[RPATH.(Tag{4,ic}),RPATH.(Tag{5,ic}),RPATH.(Tag{6,ic})] =
Swedish_Robot(TRAJ.(Tag{3,ic}),theta(ic),dt,gamma,l,alt);
fprintf('\n Path %i: Swedish Path = DONE',ic)
end
end
% 3.1 => Motion of swedish wheeled robot
function [APath,BPath,CPath]= Swedish_Robot(TM,theta,dt,gamma,l,alt)
[x,y] = deal(TM(1,1),TM(1,2));
[xa,ya,tha,ta] = deal(x,y,theta,TM(1,3)+alt);
[EXs,EYs,vya,vxa,wda,wv1,wv2,wv3] = deal(0);
[EIX,EIY,EX,EY,ELX,ELY] = deal(0,0,0,0,0,0);
[kp,ki,kd] = deal(1,0,0);
for ic = 1:height(TM)
t = TM(ic,3);
%forwardfeed
vxd1 = TM(ic,4); vyd1 = TM(ic,5);
wd = TM(ic,6);
%feedback
EIX = EIX + dt*EX; EIY = EIY + dt*EY;
EDX = (EX-ELX)/dt; EDY = (EY-ELY)/dt;
ELX = EX; ELY = EY;
xref = x; yref = y;
xrefd = TM(ic,1); yrefd = TM(ic,2);
EX=xrefd-xref; EY=yrefd-yref;
vxd2 = kp*EX+ki*EIX+kd*EDX;
vyd2 = kp*EY+ki*EIY+kd*EDY;
vxd =vxd1 + vxd2 + 0.1*sin(5*t);
vyd =vyd1 + vyd2 + 0.1*sin(5*t);

v = [vxd;vyd;wd];
v_wheel = mm(theta,gamma,l)*v/cos(gamma);
robot_state = [x;y;theta];
V1 = v_wheel(1);V2=v_wheel(2);V3=v_wheel(3);
robot_state = robot_state +
dt*inv(mm(theta,gamma,l))*cos(gamma)*[V1;V2;V3];
x = robot_state(1); y = robot_state(2);theta = robot_state(3);
xa = [xa;x]; ya = [ya;y];
tha = [tha;theta]; ta = [ta;t];
EXs = [EXs;EX]; EYs = [EYs;EY];
vxa = [vxa;vxd]; vya = [vya;vyd];
wda = [wda;wd]; wv1 = [wv1;v_wheel(1)];
wv2 = [wv2;v_wheel(2)]; wv3 = [wv3;v_wheel(3)];
end
[APath(:,1),APath(:,2),APath(:,3),APath(:,4)] = deal(xa,ya,tha,ta);
[BPath(:,1),BPath(:,2),BPath(:,3),BPath(:,4),BPath(:,5)] =
deal(EXs,EYs,vxa,vya,wda);
[CPath(:,1),CPath(:,2),CPath(:,3)] =
deal(wv1,wv2,wv3);
end
% 3.2 =>
function [M] = mm(theta,gamma,l)
M=[cos(theta+gamma),sin(theta+gamma),((cos(theta)*sin(theta+gamma)-
sin(theta)*cos(theta+gamma))/sqrt(3))*l;...
cos(theta+deg2rad(120)+gamma),sin(theta+deg2rad(120)+gamma),((cos(theta+de
g2rad(120))*sin(theta+deg2rad(120)+gamma)-
sin(theta+deg2rad(120))*cos(theta+deg2rad(120)+gamma))/sqrt(3))*l;...

cos(theta-deg2rad(120)+gamma),sin(theta-
deg2rad(120)+gamma),((cos(theta-deg2rad(120))*sin(theta-
deg2rad(120)+gamma)-sin(theta-deg2rad(120))*cos(theta-
deg2rad(120)+gamma))/sqrt(3))*l];

end
%% 4. Plots
% 4.0 => Plot hub function
function [f1,f2] = Plots(C,WPs,TRAJ,RPATH,MAP,Tag)
f1 = Plot1(C,WPs,TRAJ,MAP,Tag,RPATH);
f2 = Plot2(C,MAP,Tag,RPATH);
end
% 4.1 => Figure 1
function [f] = Plot1(C,WPs,TRAJ,MAP,Tag,RPATH)
figure(1);
% Subplot 1 => Straight line path to each checkpoint
subplot(2,2,1);hold on;
show(MAP); xlim([0,120]); ylim([0,120])
plot([C(:,1);C(1,1)],90-[C(:,2);C(1,2)],'c')
plot(C(:,1),90-C(:,2),'ro');
title('Straight line path');
hold off;

% Subplot 2 => Astar generated path
subplot(2,2,2);hold on;
show(MAP); xlim([0,120]); ylim([0,120])
for ic = 1:height(C)
plot(WPs.(Tag{1,ic})(:,1),WPs.(Tag{1,ic})(:,2),'c')
end
plot(C(:,1),90-C(:,2),'ro');
title('Astar generated path');
hold off;
% Subplot 3 => Trajectory generated path
subplot(2,2,3);hold on;
show(MAP); xlim([0,120]); ylim([0,120])
for ic = 1:height(C)
plot(TRAJ.(Tag{3,ic})(:,1),TRAJ.(Tag{3,ic})(:,2),'c')
end
plot(C(:,1),90-C(:,2),'ro');
title('Trajectory generated path');
hold off;
% Subplot 4 => Trajectory generated path
subplot(2,2,4);hold on;
show(MAP); xlim([0,120]); ylim([0,120])
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,1),RPATH.(Tag{4,ic})(:,2),'c')
end
plot(C(:,1),90-C(:,2),'ro');
title('Path of robot');
hold off;
% Set the size of the figure window
f = figure(1);
f.WindowState = 'maximized';
end
% 4.2 => Figure 2
function [f] = Plot2(C,MAP,Tag,RPATH)
figure(2);
% Subplot 1 => Trajectory generated path
subplot(5,4,[3 4 7 8 11 12]);hold on;
show(MAP); xlim([0,120]); ylim([0,100])
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,1),RPATH.(Tag{4,ic})(:,2),'g')
quiver(RPATH.(Tag{4,ic})(1:50:end,1),RPATH.(Tag{4,ic})(1:50:end,2),cos(RPA
TH.(Tag{4,ic})(1:50:end,3)),sin(RPATH.(Tag{4,ic})(1:50:end,3)),0.2);
end
plot(C(:,1),90-C(:,2),'ro');
title('Trajectory generated path');
hold off;
% Subplot 2 => Trajectory generated path
subplot(5,4,[1 2]);hold on;
xlim([0,51]); ylim([0,120])
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{4,ic})(:,1),'b')
end
title('X position');
xlabel('Time (seconds)')
ylabel('x (meters)')
hold off;

% Subplot 3 => Trajectory generated path
subplot(5,4,[5 6]);hold on;
xlim([0,51]); ylim([0,120])
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{4,ic})(:,2),'b')
end
title('Y position');
xlabel('Time (seconds)')
ylabel('y (meters)')
hold off;
% Subplot 4 => Trajectory generated path
subplot(5,4,[9 10]);hold on;
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{4,ic})(:,3),'b')
end
title('Theta');
xlabel('Time (seconds)')
ylabel('theta (rad)')
hold off;
% Subplot 5 => Trajectory generated path
subplot(5,4,[13 14]);hold on;
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{5,ic})(:,3),'b')
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{5,ic})(:,4),'r')
end
title('x and y velocities');
hold off;
% Subplot 5 => Trajectory generated path
subplot(5,4,[15 16]);hold on;
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{5,ic})(:,1),'b')
end
title('Error in x pos');
hold off;
% Subplot 5 => Trajectory generated path
subplot(5,4,[17 18]);hold on;
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{5,ic})(:,5),'b')
end
title('Angular velovity');
hold off;
% Subplot 5 => Trajectory generated path
subplot(5,4,[19 20]);hold on;
for ic = 1:height(C)
plot(RPATH.(Tag{4,ic})(:,4),RPATH.(Tag{5,ic})(:,2),'b')
end
title('Error in y pos');
hold off;

% Set the size of the figure window
f = figure(2);

f.WindowState = 'maximized';
