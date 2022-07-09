%% get ros info

uri = getenv("ROS_MASTER_URI")
host = getenv('ROS_HOSTNAME')
ip = getenv("ROS_IP")
%% connect

setenv('ROS_MASTER_URI',"http://192.168.1.3:11311")
setenv('ROS_IP',"192.168.1.03" )
rosinit('NodeHost','192.168.1.5')
%%  ros shutdown
rosshutdown()
%% Receive Lidar Data
                                              .........................                                                                                                                   
scans = struct([]);

lidarSub = rossubscriber("/scan","sensor_msgs/LaserScan",'DataFormat','struct');
tic
% angle 값 degree to radian
% scansAndPoses로 [scans,pose] 받고 현재 포즈 추정
% pose graph optimization 설정해보기+


i=1;
while toc < 400
  
  scanMsg = receive(lidarSub);
  scan = rosReadLidarScan(scanMsg);
  scans{i} = scan;
  %rosPlot(scanMsg)
  
  i = i+1;
  
end


%% SLAM
scans_test = scans(1:455);
maxRange = 12; % meters
mapResolution = 10; % cells per meter

slamObj = lidarSLAM(mapResolution,maxRange);
slamObj.LoopClosureThreshold = 500;
slamObj.LoopClosureSearchRadius = 5;
 figure 
for i = 1 :5: numel(scans_test)

    addScan(slamObj,scans_test{i});
    [~,currpose] = scansAndPoses(slamObj)
    show(slamObj, 'Poses', 'off');
    hold on;
    show(slamObj.PoseGraph); 
    hold off;
    drawnow
end
%% 
 show(slamObj.PoseGraph)
%% Occupancy Map of Garage
[scansSLAM,poses] = scansAndPoses(slamObj);
occGrid = buildMap(scansSLAM,poses,mapResolution ,maxRange);
figure
show(occGrid)

title('Occupancy Map of Garage')

%% RRT

% Set start and goal poses.
start = [ 5.7926    0.4540   -0.0003];
goal = [-1.7551   -1.5147    1.6472];

% Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')

% Show start and goal headings.
r = 0.5;
plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
hold off

bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 1.0;
planner.MaxIterations = 30000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

rng default

[pthObj, solnInfo] = plan(planner,start,goal);

show(occGrid)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

% Interpolate and plot path.
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off

%% RRT*

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

exmap = occupancyMap(occGrid, 10);
sv.Map = exmap;
sv.ValidationDistance = 0.4;
ss.StateBounds = [exmap.XWorldLimits; exmap.YWorldLimits; [-pi pi]];

planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;

planner.MaxIterations = 10000;
planner.MaxConnectionDistance = 2.25;

start = [0    0    0];
goal = [ 9.4515    -11.2238  pi];
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
occGrid.show;
hold on;

plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
%% HybridAStar
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

exmap = occupancyMap(occGrid, 10);
sv.Map = exmap;
sv.ValidationDistance = 0.4;
ss.StateBounds = [exmap.XWorldLimits; exmap.YWorldLimits; [-pi pi]];

planner = plannerHybridAStar(sv,'MinTurningRadius',2.7,'MotionPrimitiveLength',2);
start = [ 0.6317    0.1192    0.0771];
goal = [ 9.4515    -11.2238  pi];
refpath = plan(planner,start,goal);
show(planner)

%% PATH FOLOWING - pure pursuit
 

% 초기 경로 설정 
% path=pthObj.States(:,1:2); % path planing 기반 
%  path=[0	0
%        -3.58054526505920	-5.09848155600576;
%        -1.99510397590693	-11.1389764286034;
%        -0.0701316209161487	-11.2695251973734;
%        6.17596976399654	-11.1710587265108;
%        10.5820000000000	-11.1016000000000];

% path = [0.631700000000000	0.119200000000000	0.0771000000000002;
% 1.62872926704205	0.196223637031674	0.0771000000000002;
% 2.62575853408411	0.273247274063347	0.0771000000000002;
% 3.62278780112616	0.350270911095021	0.0771000000000002;
% 4.61981706816821	0.427294548126694	0.0771000000000002;
% 5.61684633521026	0.504318185158368	0.0771000000000002;
% 6.61387560225232	0.581341822190041	0.0771000000000002;
% 7.61090486929437	0.658365459221715	0.0771000000000002;
% 8.60793413633642	0.735389096253388	0.0771000000000002;
% 9.59642611367260	0.628129749708006	-0.293270370370371;
% 10.4790680918819	0.170348038646770	-0.663640740740741;
% 11.1361620796196	-0.575874748433797	-1.03401111111111;
% 11.4824724883166	-1.53305685011595	-1.41328168696095;
% 11.5861151505948	-2.55173863757495	-1.47431265757781;
% 11.6847641539737	-3.57100648818400	-1.47431265757781;
% 11.7834131573527	-4.59027433879304	-1.47431265757781;
% 11.8820621607316	-5.60954218940209	-1.47431265757781;
% 11.9807111641105	-6.62881004001114	-1.47431265757781;
% 12.0793601674894	-7.64807789062018	-1.47431265757781;
% 12.1476059213826	-8.66875813425192	-1.62451035019045;
% 11.9023372791203	-9.65667096850169	-2.00378092604029;
% 11.3087308738441	-10.4835687315702	-2.38305150189012;
% 10.4511560470383	-11.0319243564903	-2.76232207773996;
% 9.45150000000000	-11.2238000000000	3.14159265358979];
% path=path(:,1:2);

% path=[
%    11.9944   -3.9858   -1.5498;
%    12.0538   -4.5258   -1.5190;
%    12.1320   -5.0275   -1.4806;
%    12.1886   -5.5374   -1.5393;
%    12.1574   -6.0727   -1.6796;
%    12.1517   -6.5744   -1.6419;
%    12.1734   -7.1661   -1.5925;
%    12.2166   -7.6863   -1.5471;
%    12.2964   -8.3115   -1.4981;
%    12.3623   -8.8330   -1.5110;
%    12.3746   -9.3755   -1.5997;
%    12.3446   -9.9111   -1.6613;
%    12.3975  -10.4305   -1.6964;
%    12.1243  -10.8548   -2.2175;
%    11.6687  -11.0536   -2.7430;
%    11.1065  -11.0459   -3.1398;
%    10.5820  -11.1016   -3.0880;
%     9.9817  -11.1925   -3.0453;
%     9.4515  -11.2238   -3.1324;
%     8.9178  -11.2658   -3.1133;
%     8.4121  -11.3123   -3.1152;
%     7.8856  -11.3051    3.0711;
%     7.3205  -11.2928    3.0711;
%     6.8103  -11.3066    3.1120;
%     6.2805  -11.3414   -3.1331;
%     5.9595  -11.3848   -3.0945;
%     5.3777  -11.4743   -3.0447;
%     4.8387  -11.5767   -3.0093;
%     4.3007  -11.6495   -3.0698;
%     3.7592  -11.6570    3.1151;
%     3.2319  -11.6613    3.0929;
%     2.7307  -11.6592    3.0804;
%     2.2822  -11.6554    3.0883;
%     1.7536  -11.6804    3.1255;
%     1.2267  -11.7273   -3.1127;
%     0.7557  -11.7893   -3.0797;
%     0.2244  -11.8566   -3.0814;
%    -0.2213  -11.7633    2.8612;
%    -0.5693  -11.3933    2.3196;
%    -0.6882  -10.8911    1.7980;
%    -0.7026  -10.2997    1.5819;
%    -0.7565   -9.7758    1.6065;
%    -0.8283   -9.2463    1.6459;
%    -0.9124   -8.7507    1.6745;
%    -1.0226   -8.2244    1.7048;
%    -1.1235   -7.6980    1.7138;
%    -1.1526   -7.1279    1.5768;
%    -1.1852   -6.4882    1.5707;
%    -1.2353   -5.9342    1.6017;
%    -1.2991   -5.4490    1.6390;
%    -1.3886   -4.8709    1.6683;
%    -1.4940   -4.3153    1.6863;
%    -1.5542   -3.7559    1.5928;
%    -1.5735   -3.1540    1.5540;
%    -1.6247   -2.5697    1.5869;
%    -1.6803   -2.0429    1.6179;
%    -1.7551   -1.5147    1.6472;
%    -1.8502   -1.0131    1.6877;
%    -1.9072   -0.4749    1.4891];
% path=path(:,1:2);

path=[
    0 0 0;
    1 0 0;
    2 0 0;
    3 0 0;
    3.5 0.5  0;
    4 1 0;
    ];
 path=path(:,1:2);
% pure_pursuit 초기화 
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation ]';

% pure pursuit 기계 설정 
robot = differentialDriveKinematics("TrackWidth", 0.25, "VehicleInputs", "VehicleSpeedHeadingRate","WheelRadius",0.051);


% controllerPurePursuit 설정
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.91;
controller.MaxAngularVelocity = 18;  % 원래 10 
controller.LookaheadDistance = 0.2; % 0.3

% 남은 목표거리 설정
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% ros 설정
robotCmd = rospublisher('/vesc/low_level/ackermann_cmd_mux/output','ackermann_msgs/AckermannDriveStamped');
velMsg = rosmessage(robotCmd);

% simulation loop  
sampleTime = 0.012;  % 0.2 -> 5Hz로 테스트 해보기 지금까지는 0.01이 best
vizRate = rateControl(1/sampleTime);

angles = struct([]);
 i=1;
% while( distanceToGoal > goalRadius )
%     
%     % 속도, 각속도 계산
%     [v, anglevel] = controller(robotCurrentPose);
%   
%     % Update the current pose
%     vel = derivative(robot, robotCurrentPose, [0.91 anglevel]);
%     robotCurrentPose = robotCurrentPose + vel*sampleTime;
%      
%     % Re-compute the distance to the goal
%     distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
%     
%     % 각속도 값 저장 
%     angles{i} = anglevel;
%     
%     i = i+1;
%     waitfor(vizRate);
%   
% end

%v 넣어서 테스트 해보기 
speeds = struct([]);
while( distanceToGoal > goalRadius )
    
    % 속도, 각속도 계산
    [v, anglevel] = controller(robotCurrentPose);
  
    % Update the current pose
    vel = derivative(robot, robotCurrentPose, [v anglevel]);
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
     
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % 각속도 값 저장 
    angles{i} = anglevel;
    speeds{i} = v;
    i = i+1;
    waitfor(vizRate);
  
end
speeds = cell2mat(speeds);
angles = cell2mat(angles);

% 왼쪽 코스에서는 왼쪽으로 많이 돌지 못함
% 오른쪽 코스에서는 오른쪽으로 많이 돌지 못함
for i=1:length(angles)
       angles(i) = angles(i)+(-0.0433);
end

%% 주행 

% 5hz 
reset(vizRate);
sampleTime = 0.012;
vizRate = rateControl(1/sampleTime);

 count = 0;
 alldistances = zeros([370 1]);
 allangles = zeros([370 1]);
 key = 0;
j=1;
% wecar 주행 
for i=1:length(angles)
    
    
    sendROSMessage(robotCmd,velMsg,angles(i),speeds(i));
 
%     while 1
%         [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
% %         alldistances = [alldistances distances]; 
% %         allangles = [allangles angles];
%         alldistances(j) = distances; 
%         allangles(j) = angles;
%     if count > 360 % 360*2.5
%         scans = lidarScan(alldistances, allangles);
%         break
%     end
%     count = count+1;
%     j=j+1;
%     end
%     waitfor(vizRate);
% end
% 0.02 6번
% 0.015 3번


%%
tic;
odomSub = rossubscriber("/vesc/odom");

% ros 속도 설정 best = 0.01
robotCmd = rospublisher('/vesc/low_level/ackermann_cmd_mux/output','ackermann_msgs/AckermannDriveStamped');
velMsg = rosmessage(robotCmd);
velMsg.Drive.Speed = 0.91;
velMsg.Drive.SteeringAngle = -0.0433;

i=1;

position = struct([]);
lidarSub = rossubscriber("/scan","sensor_msgs/LaserScan",'DataFormat','struct');
tic;
while toc <30 
   
     send(robotCmd,velMsg);
     pause(0.012);

end 

%% 
function position = reciveOdom(odomSub,robotCmd,velMsg)
    
   send(robotCmd,velMsg);
   odomMsg = receive(odomSub);
   position = odomMsg.Pose.Pose.Position;

end


function sendROSMessage(robotCmd,velMsg,angles,speeds)
    
   
    velMsg.Drive.SteeringAngleVelocity = angles;   
    velMsg.Drive.SteeringAngle = angles;
    velMsg.Drive.Speed = speeds;
    send(robotCmd,velMsg);
    
end

%% rrt hleper func
function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;

    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end
