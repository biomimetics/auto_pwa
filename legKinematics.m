function leg_frames = legKinematics(body_frames, crank_angle, leg_length)

hip_offsets = [0.05 0.03 0.0]';
hip_coords = [1 0 -1 1 0 -1; 1 1 1 -1 -1 -1; 1 1 1 1 1 1];
hip_angle = [pi pi pi 0 0 0];
    
n_leg = 6;
hip_frames = zeros(4,4,n_leg);

for l = 1:n_leg
    hip_frames(:,:,l) = ...
        [angle2dcm(hip_angle(l),0,0) hip_coords(:,l).*hip_offsets; 0 0 0 1];
end

n_t = size(body_frames,3);

leg_frames = zeros(4,4,n_leg,n_t);

leg_crank_idx = [1 1 1 2 2 2];
leg_offsets = [pi 0 pi 0 pi 0];

for t = 1:n_t
    for l = 1:n_leg
        leg_angle = crank_angle(leg_crank_idx(l),t) + leg_offsets(l);
        leg_frames(:,:,l,t) = body_frames(:,:,t) * hip_frames(:,:,l) * ...
            crankKinematics(leg_length, leg_angle);
    end
end
