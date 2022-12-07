% External torque observers - comparison
% 2020, Stanislav Mikhel

clear all; close all
% get library
if not(libisloaded('observers'))
    loadlibrary('observers.so','observers.h')
end
% methods 
%libfunctions observers -full

% read file
file = 'BoostingData2022_12_5_17_51_24.csv';

time = readmatrix(file,'Range','E2:E100000');
n = length(time);
msec = zeros(n,1);
for i = 1:1:n-1
    if(time(i)>time(i+1))
        msec(i) = time(i+1)+1000-time(i);
    else
        msec(i) = time(i+1)-time(i);
    end    
end
mean(msec)
std(msec)

pos1 = readmatrix(file,'Range','L2:L100000');
pos2 = -readmatrix(file,'Range','M2:M100000');
pos3 = readmatrix(file,'Range','N2:N100000');

vel1 = readmatrix(file,'Range','O2:O100000');
vel2 = -readmatrix(file,'Range','P2:P100000');
vel3 = readmatrix(file,'Range','Q2:Q100000');

cur1 = readmatrix(file,'Range','F2:F100000');
cur2 = readmatrix(file,'Range','G2:G100000');
cur3 = readmatrix(file,'Range','H2:H100000');

cur1d = readmatrix(file,'Range','I2:I100000');
cur2d = readmatrix(file,'Range','J2:J100000');
cur3d = readmatrix(file,'Range','K2:K100000');
Ts = mean(msec)/1000;
tm   = readmatrix(file,'Range','A2:A100000')*Ts;      % time
q =   [deg2rad(pos1), deg2rad(pos2+90)];
qd =  [vel1, vel2];
cur = [cur1*1000/30, cur2*1000/60];    % torques

acc(:,1) = gradient(qd(:,1));
acc(:,2) = gradient(qd(:,2));

JNT = 2;
K = [1,1];    % current to torque, N/A
res = libpointer('doublePtr',zeros(1,JNT));

tau = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getRobotTorque',res,q(i,:),qd(i,:),acc(i,:));
    tau(i,:) = res.Value(:);
end
figure
plot(tm, cur)
hold on
plot(tm, tau(:,1))
plot(tm, tau(:,2))
legend('cur1','cur2','tau1','tau2')
title('tau')
figure
% use 
% calllib('observers','reset',id)
% to reset observer state

% ========== Momentum Observer ================

% configuration
Kmo = [10,10];  

% get observer ID
id_mo = calllib('observers','configMomentumObserver',-1,Kmo);  %    -1 means create and configure

ext_mo = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_mo,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_mo(i,:) = res.Value(:);
end

plot(tm,ext_mo(:,1));
hold on
plot(tm,ext_mo(:,2));
legend('ext\_mo1','ext\_mo2')
title('Momentum Observer')
figure;

% ======= Disturbance Observer ===============

% configuration
sigma = 21;
xeta = 18;
beta = 50;

% get ID
id_dis = calllib('observers','configDisturbanceObserver',-1,sigma,xeta,beta);

ext_dis = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_dis,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_dis(i,:) = res.Value(:);
end

plot(tm,ext_dis);
title("Disturbance observer");
figure;

% ======= Sliding Mode Observer ============

% configuration
S1 = [20,30];
T1 = 2*sqrt(S1);
S2 = [10,10];
T2 = 2*sqrt(S2);

% get ID
id_sm = calllib('observers','configSlidingModeObserver',-1,T1,S1,T2,S2);

ext_sm = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_sm,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_sm(i,:) = res.Value(:);
end

plot(tm,ext_sm);
title("Sliding Mode Observer");
figure;

% ========= Kalman Filter Observer =======

% configuration 
S = zeros(JNT,JNT);
H = eye(JNT,JNT);
Q = blkdiag(0.002*eye(JNT,JNT), 0.3*eye(JNT,JNT));
R = 0.05*eye(JNT,JNT);

% get ID
id_kf = calllib('observers','configDistKalmanObserver',-1,S,H,Q,R);

ext_kf = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_kf,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_kf(i,:) = res.Value(:);
end

plot(tm,ext_kf);
title("Kalman Filter Observer");
figure;

% ========= Kalman Filter Continous System Observer ===== 

% configuration 
%S = zeros(JOINT_NO,JOINT_NO);
%H = eye(JOINT_NO,JOINT_NO);
%Q = blkdiag(0.2*eye(JOINT_NO,JOINT_NO), 30*eye(JOINT_NO,JOINT_NO));
%R = 0.0005*eye(JOINT_NO,JOINT_NO);

% get ID
%id_kfe = calllib('observers','configDistKalmanObserverExp',-1,S,H,Q,R);

%ext_kfe = zeros(size(cur));

%for i = 2:size(cur,1)
%    calllib('observers','getExternalTorque',id_kfe,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
%    ext_kfe(i,:) = res.Value(:); 
%end

% plot(tm,ext_kfe);
% title("Continous Kalman Filter Observer");
% figure;

% ====== Filtered Dynamics ==============

% cinfiguration 
cutOff = 8;  % rad/s
timeStep = 0.01;  % s

% get ID
id_df = calllib('observers','configFilterDynObserver',-1,cutOff,timeStep);

ext_df = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_df,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_df(i,:) = res.Value(:);
end

plot(tm,ext_df);
title("Filtered dynamics");
figure;

% ===== Filtered Range ===================

% cinfiguration 
%cutOff = 8;  % rad/s
%timeStep = 0.01;  % s
delta = 0.1;

% get ID
id_dr1 = calllib('observers','configFilterRangeObserver',-1,cutOff,timeStep, delta);
id_dr2 = calllib('observers','configFilterRangeObserver',-1,cutOff,timeStep,-delta);

ext_up = zeros(size(cur));
ext_low = zeros(size(cur));
for i = 2:size(cur,1)
    calllib('observers','getExternalTorque',id_dr1,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_up(i,:) = res.Value(:);
    calllib('observers','getExternalTorque',id_dr2,res,q(i,:),qd(i,:),K.*cur(i,:),tm(i)-tm(i-1));
    ext_low(i,:) = res.Value(:);
end

n = 1;  % joint index
plot(tm,[ext_up(:,n),ext_low(:,n),ext_df(:,n)]); 
title("Range");

% =============== exit ===================

calllib('observers','freeAll'); % clear memory

unloadlibrary('observers');