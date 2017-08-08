% output time series of transformations
function leg_frame = crankKinematics(leg_length, crank_angle)

abduction_max = pi/4;
abduction_angle = abduction_max*(sin(crank_angle)<0);

sweep_amp = pi/4;
sweep_angle = sweep_amp*cos(crank_angle);

R = angle2dcm(0,sweep_angle,abduction_angle);
T = R*[0 0 -leg_length]';
leg_frame = [R T; 0 0 0 1];
