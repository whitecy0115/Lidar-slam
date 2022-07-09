function [slamObj,resolution,maxRange] = SLAM(scans)
scans = scans(1:5:end);

maxRange = 12; % meters
resolution = 20; % cells per meter

slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

for i = 1:numel(scans)

    addScan(slamObj,scans{i});
    
end
end