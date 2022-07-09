function pure_pursuit_scout(path,scout,detector)
x = char('1202');

cam = webcam(3);
cam.Resolution = '640x480';

robotInitialLocation = path(1,1:2);

robotGoal = path(end,1:2);

initialOrientation = path(1,3);

robotCurrentPose = [robotInitialLocation initialOrientation]';

% Create a Kinematic Robot Model
robot = differentialDriveKinematics("TrackWidth", 0.4, ...
    "VehicleInputs", "VehicleSpeedHeadingRate", "WheelRadius", 0.085);


% Define the Path Following Controller
controller = controllerPurePursuit;
controller.Waypoints = path(:,1:2);
controller.DesiredLinearVelocity = 0.2; % 0.3
controller.MaxAngularVelocity = 3; %0.15
controller.LookaheadDistance = 0.5;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

sampleTime = 0.54; 
vizRate = rateControl(1/sampleTime);


while( distanceToGoal > goalRadius)

    [v, omega,~] = controller(robotCurrentPose);
    
    text = getText(cam,detector);

    if contains(text,x) == 1
            break;
    end

    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    robotCurrentPose = robotCurrentPose + vel*sampleTime;

    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    speed_8 = typecast(swapbytes(int16(v*1000)),"uint8");
    angle_8 = typecast(swapbytes(int16(omega*1000)),"uint8");
    
    TxMsg = canMessage(273, false, 8);
    TxMsg.Data = ([speed_8 angle_8 0 0 0 0]);
    transmit(scout,TxMsg);
    
    waitfor(vizRate);
   
end

reset(vizRate);
object_trak(cam,scout,detector);

end