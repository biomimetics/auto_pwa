function [runData] = dataToState
%Produces Kalman filtered states from IMU, optitrack data

% X: 		position
% Q:		Quaternions
% a_b: 	Accelerometer bias
% w_b: 	Gyro bias
% legs:	leg position. Left, Left-Right

[runData, Dir] = tNormData;

params.maxState = 17;

params.X_i = 1:3;
params.X_dot_i = 4:6;
params.X_ddot_i = 7:9;
params.euler_i = 10:12;
params.gyro_i = 13:15;
params.legs_i = 16:17;

nX = params.maxState;


params.vars.X = 		[0.0292; 0.1072; 0.0906] 	.* 1e-8; 
params.vars.Xdot = 		[0.0292; 0.1072; 0.0906] 	.* 1e-2; % Is Guess! HAH!
params.vars.XL = 		[11; 18; 26].* 1e-4;    %12,13,11 = x,y,z 
params.vars.euler = 	[0.148; 0.047; 3.696] * pi/180 .* 1e-4;	
params.vars.GYRO = 		[0.1627; 0.0589; 0.16] 	.* 1e-2; 
params.vars.legs = 		[1.47; 1.47] 		.* 1e-4; 

params.dt = .001;
params.N = length(runData.interp.t);

initX = getInitialState(runData, params);
initV = eye(params.maxState);

%Kalman filter parameters
y = makeMeasurements(runData, params);
nY = size(y,1);

D = zeros(nY,1);

C(:,:,1) = eye(nY);
C(:,:,2) = eye(nY);

C(1:6,1:6,2) 	= zeros(6);
C(10:12,10:12,2)= zeros(3);
	
%Oops
% Overestimate Q and R for EM
B = 0; u = 0;
Q = 1e-3* eye(nX); R = eye(nX);
Q(1:6,1:6) = eye(6)*1e-10;
Q(7:9,7:9) = eye(3)*1;
Q(10:12,10:12) = eye(3)*1e-10;
Q(13:15,13:15) = eye(3)*1;

rOpt = diag([...
	params.vars.X;
	params.vars.Xdot;
	params.vars.XL;
	params.vars.euler;
	params.vars.GYRO;
	params.vars.legs]);

R(:,:) = rOpt;

init_V = eye(params.maxState);

params.Q = Q;
params.R = R;
params.C = C;
params.init_V = init_V;

runData.fuse.params = params;

        
% [states.xFilt, states.xPred, states.vFilt] = kalmanFilterSmoother(params, runData, y, B, C, D, u, Q, R, initX, init_V); % Not smoothed
[runData.fuse.xfilt, runData.fuse.xpred, runData.fuse.Vfilt, runData.fuse.loglik, runData.fuse.xsmooth, runData.fuse.Vsmooth, runData.fuse.Q, runData.fuse.R] = kalmanFilterSmoother(params, runData, y, B, C, D, u, Q, R, initX, init_V); % Smoothed
% for i = 1:5
% 	tic
% 	[runData.fuse.xfilt, runData.fuse.xpred, runData.fuse.Vfilt, runData.fuse.loglik, runData.fuse.xsmooth, runData.fuse.Vsmooth, runData.fuse.params.Q, runData.fuse.params.R] = kalmanFilterSmoother(params, runData, y, B, C, D, u, runData.fuse.params.Q, runData.fuse.params.R, initX, init_V); % Smoothed
% 	runData.fuse.ll(i) = runData.fuse.loglik;
% 	toc
% end

saveName = [Dir, '\'  runData.params.saveName];
eval(['Trial' runData.params.saveName '= runData'])
save(saveName, ['Trial' runData.params.saveName] );


function measurements =  makeMeasurements(runData, params)
	% y = [p a e g l]
	measurements = NaN(3*4+2, params.N);

	measurements(params.X_i, :) 	= runData.interp.Optitrak.position';
	measurements(params.X_dot_i, :) = (diff([runData.interp.Optitrak.position; 0 0 0]) ./ params.dt)';
	measurements(params.X_ddot_i, :)= runData.interp.Robot.compXL';
	measurements(params.euler_i, :) = runData.interp.Optitrak.realEuler';
	measurements(params.gyro_i, :) 	= runData.interp.Robot.GYRO';
	measurements(params.legs_i, :) 	= runData.interp.Robot.legPos';
    



function initialState = getInitialState(robotData, params)

	initialState(params.X_i,1) = robotData.interp.Optitrak.position(1,:)';
	initialState(params.euler_i,1) = robotData.interp.Optitrak.realEuler(1,:)';
	initialState(params.X_dot_i,1) = ((robotData.interp.Optitrak.position(2,:) - robotData.interp.Optitrak.position(1,:)) ./ params.dt)';
	initialState(params.X_ddot_i,1) = robotData.interp.Robot.compXL(1,:)';
	initialState(params.gyro_i,1) = robotData.interp.Robot.GYRO(1,:)';
	initialState(params.legs_i,1) = robotData.interp.Robot.legPos(1,:)';


