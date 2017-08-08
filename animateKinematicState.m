function animateKinematicState(trial,vobj)

if nargin < 2
    vobj = [];
end

function pause_callback(handle, eventdata)
    g = guidata(handle);
    g.pause = get(handle,'Value');
    guidata(handle,g);
end

function record_callback(handle, eventdata)
    g = guidata(handle);
    g.record = get(handle,'Value');
    guidata(handle,g);
end

function stop_callback(handle, eventdata)
    g = guidata(handle);
    g.running = ~get(handle,'Value');
    guidata(handle,g);
end

function frame_callback(handle, eventdata)
    g = guidata(handle);
    g.frame = round(get(handle,'Value'));
    g.update = true;
    guidata(handle,g);
end

t_clip = 10000:11000;
body_pos = extractTrialState(trial, 'X_i',t_clip);
body_angle = extractTrialState(trial, 'euler_i',t_clip);
leg_length = 0.025;
crank_angle = extractTrialState(trial,'legs_i',t_clip);

n_t = size(body_pos,2);
body_R = angle2dcm(body_angle(1,:),body_angle(2,:),body_angle(3,:));
body_frame = zeros(4,4,n_t);
body_frame(1:3,1:3,:) = body_R;
body_frame(1:3,4,:) = body_pos; 
body_frame(4,4,:) = 1;

leg_frames = legKinematics(body_frame,crank_angle,leg_length);

fig = figure(1);
animation.wf.fig = fig;
clf

gd = guidata(fig);
gd.running = true;
gd.pause = true;
gd.record = false;
gd.update = true;
gd.frame = 1;
guidata(fig,gd);

set(fig,'Toolbar','Figure');
ax = axes('Parent',animation.wf.fig,'Units','Normalized',...
    'OuterPosition',[0,.1,1,.9]);
animation.wf.axes = ax;

hold(ax,'on');
axis(ax,'manual');
axis(ax,'equal');
axis(ax,[-0.3 0.3 -0.15 0.15 -0.05 0.05]);
grid(ax,'on');
    
slider = uicontrol('Parent',fig,'Style','Slider',...
    'Value',1,'Min',1,'Max',n_t,'SliderStep',[1/n_t 10/n_t],...
    'Units','Normalized','Position',[.1,0,.7,.1],...
    'Callback',@frame_callback);
PauseButton = uicontrol('Parent',fig,'Style','ToggleButton','Value',1,...
    'String','Pause','Units','Normalized','Position',[0,0,.1,.1],...
    'Callback',@pause_callback);
RecordButton = uicontrol('Parent',fig,'Style','ToggleButton',...
    'String','Rec','Units','Normalized','Position',[.8,0,.1,.1],...
    'Callback',@record_callback);
StopButton = uicontrol('Parent',fig,'Style','Pushbutton','String','Stop',...
    'Units','Normalized','Position',[.9,0,.1,.1],...
    'Callback',@stop_callback);

function updateDisplay(obj,event)
    gd = guidata(fig);
    if(gd.update)
        gd.update = false;
        guidata(fig,gd);
        title(ax,sprintf('Frame %d',gd.frame))
        set(slider,'Value',gd.frame);
        
        animation = plotKinematicState(animation, body_frame(:,:,gd.frame),...
            leg_frames(:,:,:,gd.frame), leg_length);
        
        if ~isempty(vobj)
            animation = plotFrontSideVid(animation,vobj,gd.frame/1000);
        end
        
        drawnow
        
        if(gd.record)
            %set(ax,'OuterPosition',[0,0,1,1]);
            print('-f1','-dpng','-r300','-noui',...
                sprintf('frame/%04d',gd.frame))
            %set(ax,'OuterPosition',[0,.1,1,.9]);
        end
    end
    
    gd = guidata(fig);
    
    if(gd.running && ~gd.pause && gd.frame < n_t-2)
        gd.frame = gd.frame + 1;
        gd.update = true;
        guidata(fig,gd);
    end
end

animation.vid.front.buf.buf_idx = 0;
animation.vid.side.buf.buf_idx = 0;

animation.vid.front.buf = bufferUpdate(animation.vid.front.buf, vobj.front);
animation.vid.side.buf = bufferUpdate(animation.vid.side.buf, vobj.side);

update_timer = timer('TimerFcn',{@updateDisplay},'Period',0.05,...
    'ExecutionMode','fixedRate','BusyMode','drop');

start(update_timer)

while(gd.running)
    pause(0.01)
    animation.vid.front.buf = bufferUpdate(animation.vid.front.buf, vobj.front);
    animation.vid.side.buf = bufferUpdate(animation.vid.side.buf, vobj.side);
    gd = guidata(fig);
end

stop(update_timer)
delete(update_timer)
clear functions;
end