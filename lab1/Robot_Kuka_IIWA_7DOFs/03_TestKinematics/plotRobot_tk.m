function plotRobot(robot, stateData, robotVisualization)
%plotRobot Animate the robot motion
%   Given a ROBOT, the array of states defined in STATEDATA, visualize the
%   robot position by animating it inside a for-loop. The frame display
%   interval set by DISPINTERVAL controls the number of frames between each
%   subsequent animation (increase to speed up animation), while the
%   optional value TARGETPOS adds a marker at the specified target
%   position.
% Input
%   robot: robot model
%   stateData: Joint position vector
%   robotVisualization: controls if the robot meshes should be visible "on"
%                       or not "off"

% Visualization Prameters
lengthEF=0.2;
fontSize=15;
queueSize=5;
fw=2.0;
workSpace=[-fw fw];


% Figure size and position
sz=get(0,'ScreenSize');

% The plot will use the complete screen ('true'), false for smaller window
fullScreen=true;

if fullScreen
    f_position=sz;
else
    f_position=[100 400 floor(0.5*sz(3:4))];
end

% Define number of moving robot joints
numInputs = numel(homeConfiguration(robot));

% Create figure and hold it
hf=figure(1);
set(gcf,'Visible','on');
show(robot, stateData(1, 1:numInputs)', 'visuals',robotVisualization, 'collision', 'off');
hold on
hf.Position=f_position;

% Get the axis handler and define the worksapce
ax = hf.CurrentAxes;
ax.XLim=workSpace;
ax.YLim=workSpace;
ax.ZLim=[-0.5 fw];

% Initialize additional markers, coordinate frames
efMarker=initEF(eye(4,4),lengthEF,0,fontSize,queueSize);

cf_names={'cf_w','cf_0','cf_1','cf_2','cf_3','cf_4','cf_5','cf_6','cf_7'};
number_links=numel(cf_names);

for i=1:number_links
    cfMarker{i}=initGenericCF(lengthEF,0,cf_names{i},fontSize);
end

tag='';
textHandler=text(0.0,-0.7,-0.2,tag,'fontsize',18,'fontweight','b');
text(0.0,-0.50,1,'click on the screen several times!','fontsize',18,'fontweight','b','color','g');

% Loop through values at specified interval and update figure
for j = 1:1:length(stateData)
    % Display manipulator model
    qDisp = stateData(j, 1:numInputs)';
    show(robot, qDisp, 'Frames', 'on', 'PreservePlot', false, 'visuals',robotVisualization,'collision', 'off');
    title(sprintf('Frame = %d of %d', j, length(stateData)));

    % Update CFs poses
    % Kinematic Parameters L1,L2,L3,and L4
    %TODO: finish the function kukaIIWA7_params
    L=kukaIIWA7_params;

    % HT robot base (link0) wrt world cf. In this case is I (4x4)
    %TODO: Define the Ht for the robot base (link 0) wrt the wcf (w)
    T0_W=eye(4);

    disp('Joint Position-------------------------');
    q=qDisp;
    % Get the absolute HTs for all the joints with the new joint position
    %TODO: Finish the function getAbsoluteHT_kukaIIWA7
    [~, HT_W] = getAbsoluteHT_kukaIIWA7(q,L, T0_W);

    % Update the CF markers
    for i=1:number_links
        updateCF(cfMarker{i},HT_W{i},lengthEF);
    end

    %Update EF pose
    updateEF(efMarker,HT_W{end},eye(4),eye(4),lengthEF,queueSize);

    % Update text 
    [tag]=sprintf('q=[%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f]deg',rad2deg(q(1)), ...
        rad2deg(q(2)),rad2deg(q(3)),rad2deg(q(4)),rad2deg(q(5)),rad2deg(q(6)),rad2deg(q(7)));
    textHandler.String=tag;

    % Update figure
    drawnow

    w = waitforbuttonpress;
%     pause;
end

% Close Figure
close(hf)

end %function