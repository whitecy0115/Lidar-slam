function [path,planner] = HybridAStar(occGrid,start,goal)
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

exmap = occupancyMap(occGrid, 20);
sv.Map = exmap;
sv.ValidationDistance = 0.4;
ss.StateBounds = [exmap.XWorldLimits; exmap.YWorldLimits; [-pi pi]];

planner = plannerHybridAStar(sv,'MinTurningRadius',2.4,'MotionPrimitiveLength',3);
% start = [ 0    0    pi]; % pi: left 0: right pi/2: up -pi/2: down
% goal = [ -7.1    2.6  pi/2]; % 1: 3.5 12 pi (2.4, 3) 2: -4 11.8 3:-7.1 2.6 pi/2  
refpath = plan(planner,start,goal);
path = refpath.States;
end