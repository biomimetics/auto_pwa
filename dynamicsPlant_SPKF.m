function nextState = dynamicsPlant_SPKF(robotData, state, params)
%robotData contains one appropriate sample of gyro, accelerometer and leg @ next time step
 
 next_state = zeros(size(state));
 
%IMU position vector
rIMU = [6.39, 6.33, 18.09] .* 1e-3;
g = 9.81;

% Vdot Update
%Compensated IMU data, Ignoring earth's rotation, noise vector for now.
compGyro = robotData.Gyro - state(params.w_bi);
compXl = robotData.Xl - cross(compGyro,cross(compGyro,rIMU)) - state(params.a_bi);

DCM = quat2dcm(state(params.Q_i));
xVelDot = DCM * compXl + [0; 0; 1] .* g;

% Quaternion update

wp  = compGyro(1);
wq  = compGyro(2);
wq  = compGyro(3);

Omega = [...
			 0  wp wq wr;
			-wp 0 -wr wq;
			-wq wr 0 -wp;
			-wr -wq wp 0];

s = 0.5*norm([wp, wq, wr] .* dt);
lambda = 1 - norm(state(Q_i))^2;
nu = 1;		%Serves as lagrange multiplier to ensure quaternion norm ~1

quatUpdate = eye(4) .* (cos(s)+nu*lambda*params.dt) - (0.5 * sin(s)/s * params.dt) .* Omega;


nextState(params.X_i) = state(params.X_i) + state(params.X_dot_i) .* params.dt;
nextState(params.X_dot_i) = state(params.X_dot_i) + xVelDot .* params.dt;
nextState(params.Q_i) = quatUpdate * state(params.Q_i);
nextState(params.a_bi) = state(params.a_bi) + randn(params.vars.a) .* params.dt;
nextState(params.legs_i) = robotData.legPos;