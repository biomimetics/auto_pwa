% Assumes runData is in workspace

%Accelerometer compensation
figure;
hold on
plot(runData.Robot.XL(:,3),'*');
plot(runData.Robot.compXL(:,3));
hold off

figure;
dif = runData.Robot.compXL(:,3)-runData.Robot.XL(:,3);
trimmean(dif,20)
plot(dif);

% Summary: dif is almost always positive, Not a huge difference in acceleration
% Drift seems ok for the length of the trial

vOP = diff(runData.interp.Optitrak.position(:,3)) ./ 0.001;
vXL = cumtrapz(runData.interp.runData.Robot.compXL(:,3)) ;

figure;
hold on
plot(vXL,'*');
plot(vOP);
hold off

%Get variances

XL(:,1) =  calibration(:,12) .* xlScale;
XL(:,2) = -calibration(:,13) .* xlScale;
XL(:,3) =  calibration(:,11) .* xlScale;
offsetVector = ones(size(XL(:,1)));
offset = mean(XL) + [0, 0, -9.81];
for i = 1:3
    XL(:,i) = XL(:,i) - offset(:,i) .* offsetVector;
end

var(XL)

gyroScale = (1/16.384) * (pi/180);  
Gyro(:,1) =  calibration(:,9) .* gyroScale;
Gyro(:,2) = -calibration(:,10) .* gyroScale;
Gyro(:,3) =  calibration(:,8) .* gyroScale;
offsetVector = ones(size(Gyro(:,1)));
offset = mean(Gyro) + [0, 0, 0];
for i = 1:3
    Gyro(:,i) = Gyro(:,i) - offset(:,i) .* offsetVector;
end

var(Gyro)

LegScale = 2*pi/(2^16);
Leg(:,1) =  calibration(:,2) .* LegScale;
Leg(:,2) =  calibration(:,3) .* LegScale;
offsetVector = ones(size(Leg(:,1)));
offset = mean(Leg) + [0, 0];
for i = 1:2
    Leg(:,i) = Leg(:,i) - offset(:,i) .* offsetVector;
end

var(Leg)

%Optitrak Var


euler = odata.Trackables.Euler(1:1.25e4,:);
offsetVector = ones(size(euler(:,1)));
offset = mean(euler) + [0, 0, 0];
for i = 1:3
    euler(:,i) = euler(:,i) - offset(:,i) .* offsetVector;
end

var(euler)

%Test forward integration of gyros

rNames = fieldnames(runData.interp.Robot);
for i=1:length(rNames)
    eval(['motionCalib.Robot.' rNames{i} ' = runData.interp.Robot.' rNames{i} '(145000:end-7000,:);']);

end

oNames = fieldnames(runData.interp.Optitrak);
oNames(6) = [];
for i=1:length(oNames)
    eval(['motionCalib.Optitrak.' oNames{i} ' = runData.interp.Optitrak.' oNames{i} '(147000:end-7000,:);']);

end
motionCalib.Optitrak.RealIdx = runData.interp.Optitrak.RealIdx(find(runData.interp.Optitrak.RealIdx > 147000));
motionCalib.Robot.t = motionCalib.Robot.t + 2.*ones(size(motionCalib.Robot.t));
for i=1:length(rNames)
    eval(['motionCalib.Robot.' rNames{i} '(1:1000,:) = [];']);
    eval(['motionCalib.Robot.' rNames{i} '(end-999:end,:) = [];']);

end
motionCalib.t = 0:0.001:(length(motionCalib.Robot.t)/1000-0.001);
motionCalib.interp.Robot = motionCalib.Robot;
motionCalib.interp.Optitrak = motionCalib.Optitrak;
motionCalib.interp.Optitrak.RealIdx = motionCalib.interp.Optitrak.RealIdx - motionCalib.interp.Optitrak.RealIdx(1) .* ones(size(motionCalib.interp.Optitrak.RealIdx));
for i = 1:3
    motionCalib.interp.Optitrak.euler(:,i) = motionCalib.interp.Optitrak.euler(:,i).* (pi/180);
end


%Gravity compensate accelerometer

    psi = motionCalib.interp.Optitrak.euler(:, 1);
    theta = motionCalib.interp.Optitrak.euler(:, 2);
    phi = motionCalib.interp.Optitrak.euler(:, 3);
    
    gOffset = 9.81 .* [-sin(theta), sin(phi) .* cos(theta), cos(phi) .* cos(theta)];

    motionCalib.interp.Robot.compXL = motionCalib.interp.Robot.XL - gOffset;
    
figure;
hold on
plot(robotData.Robot.t,robotData.interp.Robot.GYRO(:,3));
plot(robotData.Optitrak.t,robotData.interp.Optitrak.euler(:,1));
hold off

motionCalib.interp.Optitrak.vel = diff([0 0 0; motionCalib.interp.Optitrak.position])./0.001; %Implement this in the import code

save('motionCalib.mat', 'motionCalib')



%Kalman Filter debugging
xvar = ans.vFilt(1:3,1:3,:);

for i = 1:length(xvar)
    v(i) = trace(xvar(:,:,i));
end

%Uncertaintly grows linearly
%Acceleration is zero
%Velocity
figure;
hold on
oIdx = robotData.interp.Optitrak.RealIdx(2:end/2);
plot(robotData.t(oIdx), diff([robotData.interp.Optitrak.position(oIdx,1);0])./0.001,'*');figure(gcf);
t = 0:0.001:1.999;
plot(t, xFilt(4,:),'-r');
plot(t, cumtrapz(t, xFilt(7,:)),'-g');
hold off

close all;
%Position
figure;
hold on
oIdx = robotData.interp.Optitrak.RealIdx(2:end/2);
plot(robotData.t(oIdx), robotData.interp.Optitrak.position(oIdx,1),'*');
plot(robotData.t, cumtrapz(robotData.t, cumtrapz(robotData.t, robotData.interp.Robot.XL(:,1))),'r');
hold off
%Velocity
figure;
hold on
oIdx = robotData.interp.Optitrak.RealIdx(2:end/2);
plot(robotData.t(oIdx), diff([robotData.interp.Optitrak.position(oIdx,1);0])./0.01,'*');
plot(robotData.t, cumtrapz(robotData.t, robotData.interp.Robot.XL(:,1)),'r');
hold off

%Acceleration
figure;
hold on
oIdx = robotData.interp.Optitrak.RealIdx(2:end/2);
plot(robotData.t(oIdx), diff([diff([robotData.interp.Optitrak.position(oIdx,1);0])./0.001 ;0])./0.001,'*');
plot(robotData.t, robotData.interp.Robot.XL(:,1),'r')
hold off

%Quaternion conversion
quat = motionCalib.Optitrak.quaternions;
phi = -atan2(2*(quat(:,4).*quat(:,1)+quat(:,2).*quat(:,3)),(1-2*(quat(:,2).*quat(:,2)+quat(:,3).*quat(:,3))));

figure;
hold on
plot(phi);
plot(motionCalib.Optitrak.euler(:,3).*pi/180,'r');
hold off

theta = asin(2*(quat(:,4).*quat(:,2)+quat(:,1).*quat(:,3)));

figure;
hold on
plot(theta);
plot(motionCalib.Optitrak.euler(:,2).*pi/180,'r');
hold off

psi = atan2(2*(quat(:,4).*quat(:,3)+quat(:,2).*quat(:,3)),(1-2*(quat(:,2).*quat(:,2)+quat(:,1).*quat(:,1))));

figure;
hold on
plot(psi);
plot(motionCalib.Optitrak.euler(:,1).*pi/180,'r');
hold off

%Better edge detection for optitrak
x = robotData.Optitrak.syncMarks;
fallCut = [1];
preFall = find(diff([0; x]) < 0 & x == 5);
for i = 1:length(preFall)
    if (mean(diff(x(preFall(i)-20:preFall(i)-1)).^2) ~= 0) || (mean(x(preFall(i)+1:preFall(i)+5)) ~= 5)
        fallCut = [fallCut; i];
    end
end
preFall(fallCut) = [];
diff(preFall)


dcm = quat2dcm(kalman.xFilt(10:13,:)')
for i = 1:length(dcm)
    aComp(:,i) = dcm(:,:,1) * kalman.xFilt(7:9,i) + [0; 0; -1] .*9.81;
end
vComp = cumtrapz((0:0.001:(length(aComp)-1)/1000)',aComp');

figure;
hold on
plot(detrend(vComp(:,1)),'--r');
plot(ans.xFilt(4,:));
hold off

% Kalman Smoother Verification
close all;
t = 0:0.001:length(smooth.xsmooth)/1000 -0.001;
% X
figure;
hold on
plot(runData.t,runData.interp.Optitrak.position,'r--');
plot(t,smooth.xsmooth(1:3,:));
hold off

% XDOT
figure;
hold on
plot(runData.t,diff([runData.interp.Optitrak.position; 0 0 0])./0.001,'r--');
plot(t,smooth.xsmooth(4:6,:));
hold off

% EULER

quat = smooth.xsmooth(10:13,:)';

null = 1;
one = 2;
two = 3;
three = 4;

phi = -atan2(2*(quat(:,null).*quat(:,one)+quat(:,two).*quat(:,three)),(1-two*(quat(:,two).*quat(:,two)+quat(:,three).*quat(:,three))));

theta = asin(2*(quat(:,null).*quat(:,two)+quat(:,one).*quat(:,three)));

psi = atan2(2*(quat(:,null).*quat(:,three)+quat(:,two).*quat(:,three)),(1-two*(quat(:,two).*quat(:,two)+quat(:,one).*quat(:,one))));

smoothEuler = [phi, theta, psi];

figure;
hold on
plot(runData.t,runData.interp.Optitrak.euler,'r--');
plot(t,smoothEuler);
hold off

%%% Hold onto some code from timeSyncS
%Trim data to run start / end
% Find soonest end time
tMax = min([max(robotData.Robot.t), max(robotData.Optitrak.t), max(robotData.Control.t)]);
tMin = max([min(robotData.Robot.t), min(robotData.Optitrak.t), min(robotData.Control.t)]);

rLowClip = find(robotData.Robot.t < tMin); 

rNames = fieldnames(robotData.Robot);
for i=1:length(rNames)
    eval(['robotData.Robot.' rNames{i} '(rLowClip,:)=[];']);
end

oLowClip = find(robotData.Optitrak.t < tMin); 
oNames = fieldnames(robotData.Optitrak);
for i=1:length(oNames)
    eval(['robotData.Optitrak.' oNames{i} '(oLowClip,:)=[];']);

end

cLowClip = find(robotData.Control.t < tMin); 
cNames = fieldnames(robotData.Control);
for i=1:length(cNames)
    eval(['robotData.Control.' cNames{i} '(cLowClip,:)=[];']);

end


rHighClip = find(robotData.Robot.t > tMax); 
for i = 1:length(rNames)
    eval(['robotData.Robot.' rNames{i} '(rHighClip,:)=[];']);
end

oHighClip = find(robotData.Optitrak.t > tMax); 
for i = 1:length(oNames)
    eval(['robotData.Optitrak.' oNames{i} '(oHighClip,:)=[];']);
end

cHighClip = find(robotData.Control.t > tMax); 
for i = 1:length(cNames)
    eval(['robotData.Control.' cNames{i} '(cHighClip,:)=[];']);
end

%Re-Offset Time
robotData.Optitrak.t = robotData.Optitrak.t - laglead .* ones(size(robotData.Optitrak.t)) .* start.OptFine;
robotData.Robot.t = robotData.Robot.t - laglead .* ones(size(robotData.Robot.t)) .* start.RobotFine;

robotData.Robot.t   = robotData.Robot.t - tMin .* ones(size(robotData.Robot.t));
robotData.Optitrak.t = robotData.Optitrak.t - tMin .* ones(size(robotData.Optitrak.t));
robotData.Control.t = robotData.Control.t - tMin .* ones(size(robotData.Control.t));


% Time normalization

%Interpolate Data to even timestamps
t = 0:0.001:5; % NOPE
t = t';
runData.t = t;

cNames = fieldnames(runData.Control);
for i = 1:length(cNames)
    eval(['runData.interp.Control.' cNames{i} '= interp1(runData.Control.t, runData.Control.' cNames{i} ',runData.t,''cubic'');']);
end

rNames = fieldnames(runData.Robot);
for i = 1:length(rNames)
    eval(['runData.interp.Robot.' rNames{i} '= interp1(runData.Robot.t, runData.Robot.' rNames{i} ',runData.t,''cubic'');']);
end

oNames = fieldnames(runData.Optitrak);
for i = 1:length(oNames)
    eval(['runData.interp.Optitrak.' oNames{i} '= interp1(runData.Optitrak.t, runData.Optitrak.' oNames{i} ',runData.t,''cubic'');']);
end

%Find real (not dummy) observations
oRealIdx = knnsearch(t,runData.Optitrak.t);
truncate = find(diff(oRealIdx)==0);
oRealIdx(truncate)=[];
oRealIdx(end)=[];
runData.interp.Optitrak.RealIdx = oRealIdx;

cRealIdx = knnsearch(t,runData.Control.t);
truncate = find(diff(cRealIdx)==0);
cRealIdx(truncate)=[];
cRealIdx(end)=[];
runData.interp.Control.RealIdx = cRealIdx;

%%% Done
figure;
hold on
plot(runData.Control.state(runData.sync.Control.range(1):runData.sync.Control.range(2),1)/100);
plot(runData.Optitrak.position(runData.sync.Optitrak.range(1):runData.sync.Optitrak.range(2),1), 'r');
hold off

[~, tStartIdx] = min(abs(t - max(tMin)));
[~, tStopIdx]  = min(abs(t - min(tMax)));

runData.interp.t = t(tStartIdx+1 : tStopIdx-1);

fNames = fieldnames(runData.interp.Robot);
for i=1:length(fNames)
    eval(['len = length(runData.interp.Robot.' fNames{i} ');']);
    clip = [(1:tStartIdx)' ; (tStopIdx:len)'];
    eval(['runData.interp.Robot.' fNames{i} '(clip,:)=[];']);
end

fNames = fieldnames(runData.interp.Optitrak);
for i=1:length(fNames)
    eval(['len = length(runData.interp.Optitrak.' fNames{i} ');']);
    clip = [(1:tStartIdx)' ; (tStopIdx:len)'];
    eval(['runData.interp.Optitrak.' fNames{i} '(clip,:)=[];']);
end

fNames = fieldnames(runData.interp.Control);
for i=1:length(fNames)
    eval(['len = length(runData.interp.Control.' fNames{i} ');']);
    clip = [(1:tStartIdx)' ; (tStopIdx:len)'];
    eval(['runData.interp.Control.' fNames{i} '(clip,:)=[];']);
end

%% Test integration of Gyros


    
    psi = runData.interp.Optitrak.euler(:,1);
    theta = runData.interp.Optitrak.euler(:,2);
    phi = runData.interp.Optitrak.euler(:,3);    

quat = runData.interp.Optitrak.quaternions;

dcm = quat2dcm(quat);





    
    for i = 1:length(runData.interp.t)
        dcm = quat2dcm(quat(i,:));
        phi(i) = atan2(dcm(3,2),dcm(3,3));
        theta(i) = -asin(dcm(3,1));
        psi(i) = atan2(dcm(2,1),dcm(1,1));

        [psi(i), theta(i), phi(i)] = quat2angle(quat(i,:), 'ZYX');
        
        R = [...
                0   sin(phi(i))*sec(theta(i))     cos(phi(i))*sec(theta(i));
                0   cos(phi(i))                   -sin(phi(i));
                1   sin(phi(i))*tan(theta(i))     cos(phi(i))*tan(theta(i))];

        eulerDot(i,:) = (R * runData.interp.Robot.GYRO(i,:)');
    end
    
%Yaw Rate
figure;
hold on
plot(eulerDot(:,1),'r');
plot(diff(runData.interp.Optitrak.euler(:,1))./0.001);
hold off

%Pitch Rate
figure;
hold on
plot(eulerDot(:,2),'r');
plot(diff(runData.interp.Optitrak.euler(:,2))./0.001);
hold off

%Roll Rate
figure;
hold on
plot(eulerDot(:,3),'r');
plot(diff(runData.interp.Optitrak.euler(:,3))./0.001);
hold off

%Roll Angle
figure;
hold on
plot(detrend(cumtrapz(runData.interp.t, eulerDot(:,3))),'r');
plot(phi);
title('Roll')
hold off

%Pitch Angle
figure;
hold on
plot(cumtrapz(runData.interp.t, eulerDot(:,2)),'r');
plot(theta);
title('Pitch');
hold off

%Yaw Angle
figure;
hold on
plot(cumtrapz(runData.interp.t, eulerDot(:,1)),'r');
plot(psi);
hold off


