%Forward integrate accelerometer, gyro
function nextState = dynamicsUpdate_int(State, params)
% x = [p v a theta thetadot]
%forward integrate on IMU reads

    nextState(params.X_ddot_i)  = 0; 		
    nextState(params.gyro_i)    = 0;
    nextState(params.legs_i)    = State(params.legs_i);   
    nextState(params.euler_i)   = State(params.euler_i) + params.dt * gyro2EulerDot(State, params);

    % Quaternion update
    % gyro = State(params.gyro_i);
    % wp  = gyro(1);
    % wq  = gyro(2);
    % wr  = gyro(3);

    % Omega = [...
    %              0  wp wq wr;
    %             -wp 0 -wr wq;
    %             -wq wr 0 -wp;
    %             -wr -wq wp 0];

    % s = 0.5*norm([wp, wq, wr] .* params.dt);
    % lambda = 1 - norm(State(params.quaternions_i))^2;
    % nu = 1;     %Serves as lagrange multiplier to ensure quaternion norm ~1

    % quatUpdate = (eye(4) .* (cos(s)+nu*lambda*params.dt) - (0.5 * sin(s)/s * params.dt) .* Omega) * State(params.quaternions_i);
    % % quatUpdate = quatUpdate/norm(quatUpdate);

    % nextState(params.quaternions_i)   = quatUpdate;
    
    % DCM = quat2dcm(State(params.quaternions_i)');
    % xVelDot = DCM * State(params.X_ddot_i) + [0; 0; -1] .* 9.81;
    euler = State(params.euler_i);  %yaw pitch roll
    psi = euler(1);
    theta = euler(2);
    phi = euler(3);
    DCM = angle2dcm(psi, theta, phi, 'ZYX');
    xVelDot = DCM * State(params.X_ddot_i) + [0; 0; -1] .* 9.81;


    nextState(params.X_dot_i)   = State(params.X_dot_i) + State(params.X_ddot_i)* params.dt;
    nextState(params.X_i)       = State(params.X_i) + State(params.X_dot_i).* params.dt;


function eulerDot = gyro2EulerDot(State, params)

    euler = State(params.euler_i);  %yaw pitch roll
    psi = euler(1);
    theta = euler(2);
    phi = -euler(3);

    R = [...
            0   sin(phi)*sec(theta)     cos(phi)*sec(theta);
            0   cos(phi)                -sin(phi);
            1   sin(phi)*tan(theta)     cos(phi)*tan(theta)];

    eulerDot = R * State(params.gyro_i);
