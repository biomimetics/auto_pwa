%%Normalize data to regular time updates
%Usage: point to folder with robot and optitrak files

function [runData, Dir] = tNormData

%%Get filenames
Dir=uigetdir('..');
dirinfo = dir(Dir);


for i = 1:length(dirinfo)

    j = strfind(dirinfo(i).name, 'Tracking');
    if (~isempty(j));
        optFName = fullfile(Dir, dirinfo(i).name);
    end

    k = strfind(dirinfo(i).name, 'robot');
    if (~isempty(k));
        robotFName = fullfile(Dir, dirinfo(i).name);
    end

    l = strfind(dirinfo(i).name, 'control');
    if (~isempty(l));
        controlFName = fullfile(Dir, dirinfo(i).name);
    end
end

f = strfind(Dir, '\');
params.saveName = [Dir(f(end)+1:end)];
params.config.outlierThreshold = 4;

params.config.legScale  = 95.8738e-6;   % 16 bit to radian
params.config.xlScale   = (1/4096) * 9.81;   %XL read to m/s^2 (\pm 8g range)
params.config.gyroScale = (1/16.384) * (pi/180);   %Gyro read to rad/s (\pm 2000 deg/s range)

%Process Data
% Optitrak, Control Log, Robot Data
%Robot
robotData.Robot = pullRobot(robotFName, params);

%Control
[robotData.Control, params] = pullControl(controlFName, params);

%Optitrak
robotData.Optitrak = pullOptitrak(optFName, params);

%%%% Synchronization
[runData] = timeSync(robotData);

%% Time normalization

tMin = [...
        robotData.Robot.t(runData.sync.Robot.range(1)), 
        robotData.Optitrak.t(runData.sync.Optitrak.range(1)), 
        robotData.Control.t(runData.sync.Control.range(1))];

tMax = [...
        robotData.Robot.t(runData.sync.Robot.range(2)), 
        robotData.Optitrak.t(runData.sync.Optitrak.range(2)), 
        robotData.Control.t(runData.sync.Control.range(2))];

t = (0:0.001:max(tMax-tMin))';

start = runData.sync.Control.range(1); stop = runData.sync.Control.range(2);
tTempControl = runData.Control.t(start:stop);
tempOutliers = runData.Control.outliers;
tempOutliers(find(tempOutliers < start)) = [];
tempOutliers(find(tempOutliers > stop)) = [];
tTempControl(tempOutliers - (start+1)) = [];

fNames = fieldnames(runData.Control);
for i = 1:length(fNames)-1
    eval(['tempIntVec = runData.Control.' fNames{i} '(start:stop,:);']);
    tempIntVec(tempOutliers - (start+1),:) = [];
    eval(['runData.interp.Control.' fNames{i} '= interp1(tTempControl, tempIntVec,t,''cubic'');']);
end

start = runData.sync.Optitrak.range(1); stop = runData.sync.Optitrak.range(2);
tTempOptitrak = runData.Optitrak.t(start:stop);
tempOutliers = runData.Optitrak.outliers;
tempOutliers(find(tempOutliers < start)) = [];
tempOutliers(find(tempOutliers > stop)) = [];
tTempOptitrak(tempOutliers - (start-1),:) = [];

fNames = fieldnames(runData.Optitrak);
for i = 1:length(fNames)-1
    eval(['tempIntVec = runData.Optitrak.' fNames{i} '(start:stop,:);']);
    tempIntVec(tempOutliers - (start-1),:) = [];
    eval(['runData.interp.Optitrak.' fNames{i} '= interp1(tTempOptitrak, tempIntVec,t,''cubic'');']);
end

start = runData.sync.Robot.range(1); stop = runData.sync.Robot.range(2);
tTemp = runData.Robot.t(start:stop);
tempOutliers = runData.Robot.outliers;
tempOutliers(find(tempOutliers < start)) = [];
tempOutliers(find(tempOutliers > stop)) = [];
tTemp(tempOutliers  - (start-1)) = [];

fNames = fieldnames(runData.Robot);
for i = 1:length(fNames)-1
    eval(['tempIntVec = runData.Robot.' fNames{i} '(start:stop,:);']);
    tempIntVec(tempOutliers - (start+1),:) = [];
    eval(['runData.interp.Robot.' fNames{i} '= interp1(tTemp, tempIntVec,t,''cubic'');']);
end

%Find real (not dummy) observations
oRealIdx = knnsearch(t,tTempOptitrak);
truncate = find(diff(oRealIdx)==0);
oRealIdx(truncate)=[];
oRealIdx(end)=[];
runData.interp.Optitrak.RealIdx = oRealIdx;

cRealIdx = knnsearch(t,tTempControl);
truncate = find(diff(cRealIdx)==0);
cRealIdx(truncate)=[];
cRealIdx(end)=[];
runData.interp.Control.RealIdx = cRealIdx;

runData.interp.t = t;

runData.params = params;

saveName = [Dir, '\'  params.saveName];
eval(['Trial' params.saveName '= runData']);
save(saveName, ['Trial' params.saveName] );


function [robotData] = timeSync(runData) 

% Find Start of Motion
% Control: based on commanded leg speed
robotData = runData;
config.motionThreshold = 1e-3;
config.robotStart = 500;
config.treadmillStartup = 200/100;  %Time to assume treadmill is at full speed
config.optDelay = -0.030;
config.robotDelay = -0.001;

c = diff(robotData.Control.state(:,1));
[~, ctrlStart] = findpeaks(c,'NPEAKS',1,'MINPEAKHEIGHT',config.motionThreshold);
start.Control = robotData.Control.t(ctrlStart);

% Robot Based on beggining of leg movement, finds end of longest stretch of zeros in 1st half
a = diff(robotData.Robot.legPos(1:end/2,1));
x = a';
i = [ find(x(1:end-1) ~= x(2:end)) length(x) ];
l = diff([ 0 i ]);
v = x(i);
j = find(v == 0);
[~, maxidx] = max(l(j));
k = j(maxidx);
  
start.Robot = robotData.Robot.t(config.robotStart); %maybe not do this

% Optitrack: based on beginning of run
o = diff(robotData.Optitrak.position(:,1));
[~, oStart] = findpeaks(o,'NPEAKS',1,'MINPEAKHEIGHT',config.motionThreshold);
start.Optitrak = robotData.Optitrak.t(oStart);

%Rough normalization
robotData.Control.t = robotData.Control.t - ones(size(robotData.Control.t)) .* start.Control;
robotData.Robot.t = robotData.Robot.t - ones(size(robotData.Robot.t)) .* start.Robot;
robotData.Optitrak.t = robotData.Optitrak.t - ones(size(robotData.Optitrak.t)) .* start.Optitrak;


% Fine tuning based on sync pulse. Use control signal as base

%Optitrak
x = robotData.Optitrak.syncMarks;
c = robotData.Control.sync;

fallCut = [1];      %   Falling Edge detector
preFall = find(diff([0; x]) < 0 & x == 5);
for i = 1:length(preFall)
    if (mean(diff(x(preFall(i)-20:preFall(i)-1)).^2) ~= 0) || (mean(x(preFall(i)+1:preFall(i)+5)) ~= 5)
        fallCut = [fallCut; i];
    end
end
preFall(fallCut) = [];

oSyncFall = preFall;
cSyncFall = find(diff([0;c])<0);

[~,D] = knnsearch(robotData.Control.t(cSyncFall),robotData.Optitrak.t(oSyncFall));
cut = find(abs(D)>1); D(cut) = [];

% Find lag/lead
[nl, ld] = knnsearch(cSyncFall, oSyncFall);

laglead = sign(trimmean(oSyncFall(:) - cSyncFall(nl), 50)); 


start.OptFine = trimmean(D,10) + config.optDelay;
robotData.Optitrak.t = robotData.Optitrak.t - laglead .* ones(size(robotData.Optitrak.t)) .* start.OptFine - config.treadmillStartup;

%Robot
x = robotData.Robot.sync;
c = robotData.Control.sync;
rSyncFall = find(diff([0; x]) < 0);
cSyncFall = find(diff([0; c]) < 0);

[~,D] = knnsearch(robotData.Control.t(cSyncFall),robotData.Robot.t(rSyncFall));
cut = find(abs(D)>1); D(cut) = [];

% Find lag/lead
[nl, ld] = knnsearch(cSyncFall, rSyncFall);
laglead = sign(trimmean(rSyncFall(:) - cSyncFall(nl), 50)); 


start.RobotFine = trimmean(D,10) + config.robotDelay;
robotData.Robot.t = robotData.Robot.t - laglead .* ones(size(robotData.Robot.t)) .* start.RobotFine - config.treadmillStartup;

robotData.Control.t = robotData.Control.t - config.treadmillStartup;

%Find data range
% Start
[~ , sync.Robot.range(1)] = min(abs(robotData.Robot.t));
[~ , sync.Control.range(1)] = min(abs(robotData.Control.t));
[~ , sync.Optitrak.range(1)] = min(abs(robotData.Optitrak.t));

% End
tMax = min([max(robotData.Robot.t), max(robotData.Optitrak.t), max(robotData.Control.t)]); %Check Fault

[~ , sync.Robot.range(2)] = min(abs(robotData.Robot.t - tMax));
[~ , sync.Control.range(2)] = min(abs(robotData.Control.t - tMax));
[~ , sync.Optitrak.range(2)] = min(abs(robotData.Optitrak.t - tMax));


sync.Robot.offset = start.Robot + start.RobotFine;
sync.Optitrak.offset = start.Optitrak + start.OptFine;
sync.Control.offset = start.Control;
sync.tMax = tMax;
sync.config = config;

robotData.sync = sync;


%% Optitrak
function optitrak = pullOptitrak(fname, params)
initOffset = 50;    %Normalize N samples in

oRaw = importOTdata(fname);

offsetVector = ones(size(oRaw.Trackables.Position(:,1)));

optitrak.t = oRaw.Trackables.TrackedTimestamps - oRaw.Trackables.TrackedTimestamps(1) .* offsetVector;

temp = oRaw.Trackables.MarkerTracked;

syncMarks = sum(temp,2);
optitrak.syncMarks = syncMarks(:);

%Get Position x,y,z in analysis frame
offset = mean(oRaw.Trackables.Position(1:initOffset,:)); 
rawPosition(:,1) = oRaw.Trackables.Position(:,3) - offsetVector .* offset(1,3);
rawPosition(:,2) = -(oRaw.Trackables.Position(:,1) - offsetVector .* offset(1,1));
rawPosition(:,3) = -(oRaw.Trackables.Position(:,2) - offsetVector .* offset(1,2));

optitrak.position = rawPosition;

%Get Euler Angles Yaw, Pitch, Roll
eulerScale = pi/180;
rawEuler(:,1) = -oRaw.Trackables.Euler(:,1) .* eulerScale;
rawEuler(:,2) = -oRaw.Trackables.Euler(:,3) .* eulerScale;
rawEuler(:,3) = -oRaw.Trackables.Euler(:,2) .* eulerScale;

optitrak.euler = rawEuler;

%Get Quaternions, Swapping x y z to analysis frame, moving scalar to beggining

rawQuat(:,1) =  oRaw.Trackables.Quaternion(:,4);
rawQuat(:,2) =  oRaw.Trackables.Quaternion(:,3);
rawQuat(:,3) = -oRaw.Trackables.Quaternion(:,1);
rawQuat(:,4) = -oRaw.Trackables.Quaternion(:,2);
optitrak.quaternions = rawQuat;

for i = 1:length(optitrak.t)
    dcm = quat2dcm(rawQuat(i,:));
    [psi(i), theta(i), phi(i)] = quat2angle(rawQuat(i,:), 'ZYX');
end

optitrak.realEuler = [psi', theta', phi'];


fNames{1} = 'position';
fNames{2} = 'euler';
fNames{3} = 'quaternions';
optitrak.outliers = findOutliers(optitrak, fNames, params);

%Control Data
function [Control, params] = pullControl(fname, params)
controlRaw = importdata(fname,',');

params.Kp       = controlRaw.data (1);
params.Ki       = controlRaw.data (2);
params.Kd       = controlRaw.data (3);
params.Kaw      = controlRaw.data (4);
params.Kff      = controlRaw.data (5);
params.Kx       = controlRaw.data (6);
params.Ky       = controlRaw.data (7);
params.Ktheta   = controlRaw.data (8);
params.d        = controlRaw.data (9);
params.Ks       = controlRaw.data (10);
params.Ka        = controlRaw.data(11);
params.minV     = controlRaw.data(12);
params.maxV     = controlRaw.data(13);
params.Ox       = controlRaw.data(14);
params.Oy       = controlRaw.data(15);
params.Otheta   = controlRaw.data(16);

controlRaw = importdata(fname,',',3);
controlRaw.data(end,:) = [];

tRaw = controlRaw.data(:,1) - controlRaw.data(1,1) .* ones(size(controlRaw.data(:,1)));

% Make time continuous
tRollover = find(diff(tRaw)<0);
tOffsetVec = zeros(length(tRaw),length(tRollover));

for i = 1:length(tRollover)
    tOffsetVec(tRollover(i,1)+1:end,i) = ones(size(tOffsetVec(tRollover(i,1)+1:end)),1)*60;
end

tOffsetVec = sum(tOffsetVec,2);
Control.t = tRaw + tOffsetVec;

Control.sync = controlRaw.data(:,2);
Control.fault = controlRaw.data(:,3);
%State x,y,theta
Control.state = controlRaw.data(:,4:6);
%Command Vleft[4], VRight[4]
Control.command = controlRaw.data(:,(end-7):end);
%Treadmill Velocity Vt, RPM
Control.treadSpeed = controlRaw.data(:,7:8);

fNames{1} = 'state';
Control.outliers = findOutliers(Control, fNames, params);


function Robot = pullRobot(fname, params)

robotRaw = importdata(fname);
offsetVector = ones(size(robotRaw.data(:,1)));
Robot.t = (robotRaw.data(:,1) - offsetVector .* robotRaw.data(1,1)) ./ 1e6;
Robot.sync = robotRaw.data(:,17);

%Remove offsets
for i = 6:15
    robotRaw.data(:,i) = robotRaw.data(:,i) - offsetVector .* robotRaw.data(1,i);
end


% Leg Position L,R TODO CALIBRATE
Robot.legPos = robotRaw.data(:,2:3) * params.config.legScale;
% Commanded Leg Position L,R
Robot.legCommand = robotRaw.data(:,4:5);  
% Duty Cycle L,R
Robot.DC = robotRaw.data(:,6:7);
% BEMF L,R
Robot.BEMF = -robotRaw.data(:,14:15);
% VBatt
Robot.vBatt = robotRaw.data(:,16);

%Compute Power
vref = 3.3;             % MAKE SURE THESE ARE RIGHT
vdivide = 3.7/2.7;
vbatt = robotRaw.data(:,16); %last column of data is batt voltage in adc counts
vbatt = vbatt*vdivide*vref/1023; %configert battery voltage to Volts
emf(:,1) = vdivide*vref/1023*(-robotRaw.data(:,14));
emf(:,2) = vdivide*vref/1023*(-robotRaw.data(:,15));        
pwm(:,1) = -robotRaw.data(:,6)/4000;
pwm(:,2) = -robotRaw.data(:,7)/4000;    
pwm(find(pwm(:,1) < -4000 )) = -4000;
pwm(find(pwm(:,2) < -4000 )) = -4000;
%Calculate Motor current...
%This is VERY wrong
R = 3.3;
motori(:,1) = (vbatt - emf(:,1))/R;
motori(:,2) = (vbatt - emf(:,2))/R;

motorp(:,1) = motori(:,1).*vbatt.*pwm(:,1);
motorp(:,2) = motori(:,2).*vbatt.*pwm(:,2);

Robot.motorVoltage(:,1) = vbatt.*pwm(:,1);
Robot.motorVoltage(:,2) = vbatt.*pwm(:,2);
Robot.motorCurrent = motori;
Robot.motorPower = motorp;


calTime = 500;
% Accelerometer: Robot frame, x,y,z

XL(:,1) = robotRaw.data(:,11) .* params.config.xlScale;
XL(:,2) = robotRaw.data(:,12) .* params.config.xlScale;
XL(:,3) = robotRaw.data(:,13) .* params.config.xlScale;

offsetVector = ones(size(XL(:,1)));
offset = mean(XL(1:calTime,:)) + [0, 0, -9.81];     %assumes z is aligned with gravity
for i = 1:3
    Robot.XL(:,i) = XL(:,i) - offset(:,i) .* offsetVector;
end

% Gyroscope: Robot frame, x,y,z

Gyro(:,1) =  robotRaw.data(:,8)  .* params.config.gyroScale;
Gyro(:,2) =  robotRaw.data(:,9)  .* params.config.gyroScale;
Gyro(:,3) =  robotRaw.data(:,10) .* params.config.gyroScale;

offset = mean(Gyro(1:calTime,:)) + [0, 0, 0];
for i = 1:3
    Robot.GYRO(:,i) = Gyro(:,i) - offset(:,i) .* offsetVector;
end

%Compensate accelerometer 
rIMU = [6.39, 6.33, 18.09] .* 1e-3;
for i=1:length(Robot.XL)
    Robot.compXL(i,:) = Robot.XL(i,:) - cross(Robot.GYRO(i,:),cross(Robot.GYRO(i,:),rIMU));
end

%Find outliers
% Check gyro, acc,

fNames{1} = 'GYRO';
fNames{2} = 'XL';
Robot.outliers = findOutliers(Robot, fNames, params);


function [outliers] = findOutliers(inStruct, fNames, params)
    % Feed me struct. rawr.
    outliers = [];
    for i = 1:length(fNames)
        eval(['width = size(inStruct.' fNames{i} ',2);'])
        for ii = 1:width
            eval(['vecStd = std(inStruct.' fNames{i} '(:,ii));']);
            eval(['vecMean = mean(inStruct.' fNames{i} '(:,ii));']);
            eval(['outliers = [outliers; find(abs(inStruct.' fNames{i} '(:,ii)-vecMean) > params.config.outlierThreshold .* vecStd )];']);
            eval(['outliers = [outliers; find(isnan(inStruct.' fNames{i} '(:,ii)))];']);
        end
    end
    outliers = unique(outliers);

