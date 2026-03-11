function Xout=plotRobot(u)
%PLOTROBOT Animate the robot motion
% u(1): current time
% u(2:5): robot joint positions
% u(6:7): plot view point
% u(8)  : controls if the additional CF and EF markers should be visible
%           1: show additional markers, 0: no additional markers 
% u(9:12): robot joint velocities

% global variables to have persistant markers
persistent robot1 iviz samples vsr cfMarker number_links cf_names fh 
persistent Mellips sxe elipseHandler textHandler

% current time
t=u(1);

% Visualization Prameters
lengthEF=0.1;
fontSize=10;
fw=2.0;
workSpace=[-fw fw];
plotUpdateFreq=20; % 20 hz visualization
label_offset=0.3;
colorS='kr';
smodel_name=get_param(bdroot, 'Name');
dt=str2double(get_param(smodel_name,'FixedStep'));

% Figure size and position
sz=get(0,'ScreenSize');
fullScreen=false;

if fullScreen
    f_position=sz;
else
    f_position=[100 400 floor(0.5*sz(3:4))];%[1591 -1079 1920 988];
end

% defines how the axis view point is controlled, 1: from simulink model, 0:
% from plot
axis_view_control=0;

%% Joint State
% Joint positions
Q=u(2:5);
%Joint Velocities
Qp=u(9:12);

%% Manipulability and EF velocities
%TODO: finish the function manipulability_abb_irb4
[HT_W,x4_W, x4p_w, w4p_w, w4_w,Heax_W,S,U,w,EAxis_W] = manipulability_abb_irb4(Q,Qp,t,dt);

%% Plot initialization
if(t==0)
   
    % Close all open figures
    close all
    
    % Get robot name
    rName=evalin('base', 'robot_name');
    
    % Load robot model
    robot1 = loadrobot(rName,"DataFormat","column","Gravity",[0 0 -9.81]);
        
    %% Initialize the robot visualization
    iviz = interactiveRigidBodyTree(robot1,'ShowMarker',false);
    
    % Get the figure handler and set its position and size
    fh=gcf;
    fh.Position=f_position;
    
    % Get the axis handler and define the worksapce
    ax = fh.CurrentAxes;
    ax.XLim=workSpace;
    ax.YLim=workSpace;
    ax.ZLim=workSpace;
    
    %% CF initialization
    cf_names={'cf_w','cf_0','cf_1','cf_2','cf_3','cf_{ef}'};
    number_links=numel(cf_names);
    
    for i=1:number_links
        cfMarker{i}=initGenericCF(lengthEF,0,cf_names{i},fontSize);
    end
    
    %% Initialization of the unitary ellipsoid object
    % You can change the unitary dimension in case that the ellipsoid is
    % too big compared with the robot size.
    unitDim=1.0;
    [x, y, z] = ellipsoid(0,0,0,unitDim,unitDim,unitDim,10);
    elipseHandler=surf(x, y, z);
    elipseHandler.FaceAlpha=0.5;
    
    sxe=size(x);
    sA=sxe(1)*sxe(2);
    
    
    %transpose to transform the x,y,z matrices in vectors row-wise to be able
    %to stack them in a extended matrix of homogeneous point (x,y,z,1) Mellips
    xrow=reshape(x,1,sA);
    yrow=reshape(y,1,sA);
    zrow=reshape(z,1,sA);
    
    Mellips=[xrow;yrow;zrow;ones(1,sA)];
    
    %initialization of text objects to display information in the plot
    tag='';
   
    
    textHandler{1}=text(x4_W(1)+label_offset,x4_W(2)+label_offset,x4_W(3)+label_offset,tag, 'color',[0 0 0],'fontsize',18,'fontweight','b');
    textHandler{2}=text(0.0,0.0,-0.2,tag,'fontsize',18,'fontweight','b');
    textHandler{3}=text(0.0,0.0,-0.8,tag,'color','m','fontsize',18,'fontweight','b');
    textHandler{4}=text(1.0,0.0,2.0,tag,'color','g','fontsize',18,'fontweight','b');
    textHandler{5}=text(1.0,0.0,1.8,tag,'color','g','fontsize',18,'fontweight','b');
    textHandler{6}=text(1.0,0.0,1.6,tag,'color','g','fontsize',18,'fontweight','b');
    textHandler{7}=text(-1.5,0.0,2.0,tag,'color','r','fontsize',18,'fontweight','b');
    textHandler{8}=text(-1.5,0.0,1.8,tag,'color','r','fontsize',18,'fontweight','b');
    textHandler{9}=text(-1.5,0.0,1.6,tag,'color','r','fontsize',18,'fontweight','b');
    
    
    
    % Shows figure with all the objects
    showFigure(iviz)
    
    % Sampler counter to define when to update the plot
    samples=0;
    
    % Visualization sample rate 60Hz
    vsr=floor(1/(plotUpdateFreq*dt)); % Update freq
    
end

if(samples==vsr)
    
    % Update the robot link poses and additional markers. Currently set for
    % 20Hz, see vsr.
    
    %% Plot Robot
    
    % Updates the plot with the new joint positions
    % In this case, we are using just joints 1 2 and 3.
    % Then, the real q state for the visualization is:
    % q1 q2 q3 0 pi/2 0 (joint 4, 5 and 6 are fixed). pi/2 is needed to
    % point the ef in front of the robot.
    
    qreal=[Q(1:3); 0.0; Q(4); 0.0];
    
    % Changes the robot position
    iviz.Configuration=qreal;
    
    % Controls the plot view point from the simulink model
    if axis_view_control
        v=u(6:7);
        ax = fh.CurrentAxes;
        ax.View=v;
    end
    
    % Set the current time as the plot title
    ax.Title.String=sprintf('time = %2.2f s', t);
    
    %% Update additional CFs
    % Controls if the additional CF and EF are plotted, when activated the
    % simulation will be slower
    if u(8)
        
        % Update CFs poses
        
        %% Update Ellipsoid
        %We transform the unitary ellipsoid with the Rotation and translation
        % + scaling from the Jacobian SVD
        % NOTE: the unitary ellipsoid (Mellips) is defined in the S coordinate
        % frame, then we can apply the scalling factor S and later rotate to
        % the robot base (using U) and finally to the world c.f. (These last
        % transformations are embedded in Heax_W.
        Ma=Heax_W*S*Mellips;
        
        % To optimize the visualization, we do not delete the ellipsoid object
        % but we change its points, using the XData, YData, ZData vectors of
        % the object defined by the handler elipseHandler
        elipseHandler.XData=reshape(Ma(1,:),sxe(1),sxe(2));
        elipseHandler.YData=reshape(Ma(2,:),sxe(1),sxe(2));
        elipseHandler.ZData=reshape(Ma(3,:),sxe(1),sxe(2));
        
        
        % eye-cady stuff!
        rS=rank(S(1:3,1:3),0.01);
        [tag]=sprintf('%d-(%f)',rS,w);
        if(rS<3)
            coloR=colorS(2);
        else
            coloR=colorS(1);
        end
        
        % Again, to optimize visualization, we do not delete and create objects
        % we change its properties, in this case, the position, color, and
        % labels of each text.
        textHandler{1}.Position=[x4_W(1)+label_offset,x4_W(2)+label_offset,x4_W(3)+label_offset];
        textHandler{1}.String=tag;
        textHandler{1}.Color=coloR;
        
        [tag]=sprintf('q=[%0.2f, %0.2f, %0.2f %0.2f]deg',rad2deg(Q(1)),rad2deg(Q(2)),rad2deg(Q(3)),rad2deg(Q(4)));
        textHandler{2}.String=tag;
        
        [tag]=sprintf('S=[%0.2f, %0.2f, %0.2f]',S(1,1),S(2,2),S(3,3));
        textHandler{3}.String=tag;
        
        [tag]=sprintf('U_0=[%0.2f, %0.2f, %0.2f;',U(1,1),U(1,2),U(1,3));
        textHandler{4}.String=tag;
        
        [tag]=sprintf('       %0.2f, %0.2f, %0.2f;',U(2,1),U(2,2),U(2,3));
        textHandler{5}.String=tag;
        
        [tag]=sprintf('       %0.2f, %0.2f, %0.2f]',U(3,1),U(3,2),U(3,3));
        textHandler{6}.String=tag;
        
        [tag]=sprintf('a_w=[%0.2f, %0.2f, %0.2f;',EAxis_W(1,1),EAxis_W(1,2),EAxis_W(1,3));
        textHandler{7}.String=tag;
        
        [tag]=sprintf('       %0.2f, %0.2f, %0.2f;',EAxis_W(2,1),EAxis_W(2,2),EAxis_W(2,3));
        textHandler{8}.String=tag;
        
        [tag]=sprintf('       %0.2f, %0.2f, %0.2f]',EAxis_W(3,1),EAxis_W(3,2),EAxis_W(3,3));
        textHandler{9}.String=tag;
        
        
        %% Update the CF markers
        for i=1:number_links
            updateCF(cfMarker{i},HT_W{i},lengthEF);
        end
        
    else
        % When dissabled, the markers are still active. Then, we make them "invisible"
        for i=1:number_links
            updateCF(cfMarker{i},zeros(4),0);
        end
    end
    
    
    % Update figure
    %drawnow limitrate (low graphic resources, it will slow down also the scopes)
    drawnow
    
    %% Reset the visualizer sampler
    samples=0;
end


% increase the sampler
samples=samples+1;

%% Output: vector [Xef;Xefp;w_0;ws_0] (size 12X1)

Xout=[x4_W;x4p_w(1:3,1);w4p_w(1:3,1);w4_w(1:3,1)];


end



