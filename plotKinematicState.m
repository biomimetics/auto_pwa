function animation = plotKinematicState(animation, body_frame, leg_frames,...
    leg_length)

% Construct rectangular robot body wireframe
body_dim = [0.06 0.03 0.01];
body_points = [[1 -1 1 -1 1 -1 1 -1]*body_dim(1); ...
               [1 1 -1 -1 1 1 -1 -1]*body_dim(2); ...
               [1 1 1 1 -1 -1 -1 -1]*body_dim(3)];

body_edges = [1 1 1 2 2 3 3 4 5 5 6 7; 2 3 5 4 6 4 7 8 6 7 8 8];

body = ones(4,12,2);
body(1:3,:,1) = body_points(:,body_edges(1,:));
body(1:3,:,2) = body_points(:,body_edges(2,:));

body(:,:,1) = body_frame*body(:,:,1);
body(:,:,2) = body_frame*body(:,:,2);

xdata = squeeze(body(1,:,:))';
ydata = squeeze(body(2,:,:))';
zdata = squeeze(body(3,:,:))';

if ~isfield(animation.wf,'body')
    animation.wf.body = plot3(xdata,ydata,zdata,'r',...
        'Parent',animation.wf.axes);
else
    for l = 1:length(animation.wf.body)
        set(animation.wf.body(l),'XData',xdata(:,l),'YData',ydata(:,l),...
            'ZData',zdata(:,l))
    end
end

% Construct frames and lines for legs
leg = [0 0 0 1; 0 0 leg_length 1]';
n_l = size(leg_frames,3);
legz = zeros(4,n_l,2);
for l = 1:n_l
    legz(:,l,:) = leg_frames(:,:,l) * leg;
end

xdata = squeeze(legz(1,:,:))';
ydata = squeeze(legz(2,:,:))';
zdata = squeeze(legz(3,:,:))';

if ~isfield(animation.wf,'legs')
    animation.wf.legs = plot3(xdata,ydata,zdata,'b',...
        'Parent',animation.wf.axes);
else
    for l = 1:length(animation.wf.legs)
        set(animation.wf.legs(l),'XData',xdata(:,l),'YData',ydata(:,l),...
            'ZData',zdata(:,l))
    end    
end