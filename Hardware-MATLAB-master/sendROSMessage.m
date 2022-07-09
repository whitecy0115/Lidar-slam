function sendROSMessage(robotCmd,velMsg,angles,speeds)
    
   
    velMsg.Drive.SteeringAngleVelocity = angles;   
    velMsg.Drive.SteeringAngle = angles;
    velMsg.Drive.Speed = speeds;
    send(robotCmd,velMsg);
    
end