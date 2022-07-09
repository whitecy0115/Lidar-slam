% 가로, 세로 넉넉히 0.6m -> l = 0.8485 = 0.4243 * 2
vfh = controllerVFH;
vfh.NumAngularSectors = 100;
vfh.UseLidarScan = true; % lidarScan data를 입력으로 사용
vfh.DistanceLimits = [0.3 0.7]; % 범위 값 : 최대/최소
vfh.RobotRadius = 0.4243; % 차량 반경(r)
vfh.SafetyDistance = 0.5; % 차량 안전거리
vfh.MinTurningRadius = 0.5; % 현재속도에서 최소 회전 반경(외부 점 기준 회전) *** 수정 필요
vfh.TargetDirectionWeight = 5; % 목표 방향에 대한 비용 함수 가중치90
vfh.CurrentDirectionWeight = 2; % 록 부드러운 경로 생성
vfh.HistogramThresholds
targetDir = 0; %값이 높을수록 효율적인 경로 생성
vfh.PreviousDirectionWeight = 3; % 값이 높을수

% h = figure;
% set(h,'Position',[50 50 800 400])
sampleTime = 0.54; 
vizRate = rateControl(1/sampleTime);

tic;
while 1
    tic;
    while toc < 0.5
	% Get laser scan data
 	scan = getScanData(pRPLIDAR);
	steerDir = vfh(scan, targetDir);

	if ~isnan(steerDir) % 직진해도 되는 경우 (객체 미검출)
 		disp('go')
        
	else % Stop and search for valid direction
 		disp('stop')
    
    end
    toc
    end
    waitfor(vizRate);
end

%% 
drawnow
%% 
    omega = -0.0043
    angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([0 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    %% 

vfh = controllerVFH
vfh.NumAngularSectors = 180;
% 속성 정의 하고 테스트 하기
vfh.DistanceLimits = [0.2 0.7];
vfh.UseLidarScan = true;
vfh.RobotRadius = 0.4243; % 차량 반경(r)
vfh.SafetyDistance = 0.5; % 차량 안전거리
vfh.MinTurningRadius = 0.5; % 현재속도에서 최소 회전 반경(외부 점 기준 회전) *** 수정 필요
vfh.TargetDirectionWeight = 5; % 목표 방향에 대한 비용 함수 가중치90
vfh.CurrentDirectionWeight = 2; % 록 부드러운 경로 생성
vfh.HistogramThresholds
targetDir = 0; %값이 높을수록 효율적인 경로 생성
vfh.PreviousDirectionWeight = 3; % 값이 높을수
robotInitialLocation = path(1,:);

robotGoal = path(end,:);



initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

% Create a Kinematic Robot Model
robot = differentialDriveKinematics("TrackWidth", 0.4, ...
    "VehicleInputs", "VehicleSpeedHeadingRate", "WheelRadius", 0.085);


% Define the Path Following Controller
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3; % 0.3
controller.MaxAngularVelocity = 10; %0.15
controller.LookaheadDistance = 0.5;

% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);


% Initialize the simulation loop
sampleTime = 0.54; 
vizRate = rateControl(1/sampleTime);


i=1;j=1;
flag=0;

while( distanceToGoal > goalRadius)
    [v, omega,~] = controller(robotCurrentPose);
    t0=tic;
     while toc(t0) < 0.45
         flag = getScanData(pRPLIDAR,targetDir);
% 
% 
%         steerDir = vfh(scan, targetDir);
% 
%         if isnan(steerDir) % 장애물 발견
% %             omega = -0.2; %-0.043
% %             disp('stop')
%                 flag=1;
%         end
    end

    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    %주행
    speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    

    i=i+1;
     waitfor(vizRate);
   
end
%%

clear;clc;
scout = canChannel('PEAK-System', 'PCAN_USBBUS1')

start(scout)

message = receive(scout, Inf, "OutputFormat", "timetable")
Table 3.5 Control Mode Setting Frame
TxMsg = canMessage(1057, false, 1);
TxMsg.Data = [1];
transmit(scout,TxMsg);

%%
function turn(scout)

   for i=1:42
   
    if i<=5
    speed_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(-0.4*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end
    if i>5 && i<=13
    speed_8 = typecast(swapbytes(int16(0.2*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end

    if i>13 && i<=18
    speed_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(0.4*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end

    if i>18 && i<=25
    speed_8 = typecast(swapbytes(int16(0.2*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end

    if i>25 && i<=29
    speed_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(0.4*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end

    if i>29 && i<=37
    speed_8 = typecast(swapbytes(int16(0.2*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end

    if i>37 && i<=42
    speed_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(-0.37*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    end
    pause(0.5);
   end
end
    %% 
    
     
    speed_8 = typecast(swapbytes(int16(0*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(a(k)*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
   %  waitfor(vizRate)
   
   