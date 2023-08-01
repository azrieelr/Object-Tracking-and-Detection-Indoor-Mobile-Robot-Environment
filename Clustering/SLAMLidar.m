clc; clear; clf

load('FileLidarMO.mat');

maxLidarRange = 18;
mapResolution = 18;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 320;  
slamAlg.LoopClosureSearchRadius = 8;

for i=1:20
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

figure(1);
show(slamAlg); 
%title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
xlabel('X[meter]');
ylabel('Y[meter]');
saveas(gcf,'Map10Scan.png')

firstTimeLCDetected = false;

figure(2);
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
%title('First loop closure');
xlabel('X[meter]');
ylabel('Y[meter]');
saveas(gcf,'MapScanLoop.png')

figure(3)
show(slamAlg);
%title({'Final Built Map of the Environment', 'Trajectory of the Robot'});
xlabel('X[meter]');
ylabel('Y[meter]');
saveas(gcf,'FullScanMap.png')

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
figure(4); 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
%title('Occupancy Grid Map Built Using Lidar SLAM');
xlabel('X[meter]');
ylabel('Y[meter]');
saveas(gcf,'MapOGM.png')