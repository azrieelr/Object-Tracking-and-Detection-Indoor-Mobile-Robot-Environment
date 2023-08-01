clc; clear; clf; 

tic 
v = VideoWriter('KNN1.avi');
open(v);

load('FileLidarRB.mat');

save_percentage = 0.1; % Save 40% of frames
total_frames = length(scans);
frames_to_save = round(total_frames * save_percentage);
frame_interval = round(1 / save_percentage);

save_folder = 'saved_frames2';
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

xyzPoints = [];
for a = 1:length(scans)
    xyzPoints = [scans{a}.Cartesian zeros(length(scans{a}.Cartesian),1)];
    
    ptCloud = pointCloud(xyzPoints);
    point = [0,0,0];
    K = 100;
    [indices,dists] = findNearestNeighbors(ptCloud,point,K); 

    figure(1)
    pcshow(ptCloud)
    hold on
    plot3(point(1),point(2),point(3),'*r')
    plot3(ptCloud.Location(indices,1),ptCloud.Location(indices,2),ptCloud.Location(indices,3),'*')
    
    % Memilih indeks acak sebagai titik utama
    mainIndex = randi(length(indices));
    mainPoint = ptCloud.Location(indices(mainIndex), :);
    plot3(mainPoint(1), mainPoint(2), mainPoint(3), '*g');
    
    % Menghubungkan titik utama dengan indeks terdekat untuk setiap indeks
    for j = 1:length(indices)
        idx1 = indices(j);
        point1 = ptCloud.Location(idx1,:);
        
        % Cari indeks terdekat untuk setiap indeks
        nearestDist = inf;
        nearestIdx = -1;
        for k = 1:length(indices)
            if k == j
                continue;
            end
            idx2 = indices(k);
            point2 = ptCloud.Location(idx2,:);
            distance = norm(point1 - point2);
            
            if distance < nearestDist
                nearestDist = distance;
                nearestIdx = k;
            end
        end
        
        % Menggambarkan hubungan antara indeks dengan indeks terdekat
        idx2 = indices(nearestIdx);
        point2 = ptCloud.Location(idx2,:);
        
        x = [point1(1), point2(1)];
        y = [point1(2), point2(2)];
        z = [point1(3), point2(3)];

        plot3(x, y, z, 'Color', 'g');
    end
    
    legend('Point Cloud','Query Point','Nearest Neighbors','Location','southoutside','Color',[1 1 1])
%     legend('Point Cloud','Query Point','Nearest Neighbors','Location','southoutside','Color',[0 0 0])
    
    xlabel('X[meter]')
    ylabel('Y[meter]')
    zlabel('Z[meter]')
    
    ax = gca;
    ax.Color = 'k';
    ax.XColor = 'w';
    ax.YColor = 'w';
    ax.ZColor = 'w';
    ax.GridColor = 'w';
    
    % Saving 40% of the frames as images
    if mod(a, frame_interval) == 0
        save_file_name = fullfile(save_folder, sprintf('Frame_%d.png', a));
        print(gcf, save_file_name, '-dpng', '-r300');
    end
    
    if mod(a, frame_interval) == 0
        s = getframe(gcf);
        writeVideo(v, s);
    end
    
    hold off
    
    pause(0.1);
end

t = toc

tic;

save_folder = 'saved_frames3';
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

outputVideo = VideoWriter(fullfile(save_folder,'Euclidean.avi'));
outputVideo.FrameRate = 15;  % Optional - adjust frame rate as needed
open(outputVideo);

for frame = 1:length(scans)
    xyzPoints = scans{frame}.Cartesian;
    xyzPoints(:,3) = 0;
    pc = pointCloud(xyzPoints);
    
    minDistance = 1.3;
    [labels,numClusters] = pcsegdist(pc,minDistance);
    
    nearxy = zeros(numClusters,2);
    maxlevel = -inf;
    
    % Create a color map for the clusters
    cmap = hsv(numClusters);
    
    % Assign a color to each point based on its cluster label
    pointColors = zeros(size(xyzPoints, 1), 3);
    classification = cell(numClusters, 1);
    
    centroids = zeros(numClusters, 2);
    dimensions = zeros(numClusters, 2);
    
    for i = 1:numClusters
        c = find(labels == i);
        pointColors(c, :) = repmat(cmap(i, :), numel(c), 1);
        
        % XY coordinate extraction of obstacle
        xy = pc.Location(c, 1:2);
        
        % Compute the centroid of the cluster
        centroids(i, :) = mean(xy, 1);
    end
    
    % Update point colors based on the new cluster labels
    for i = 1:numClusters
        c = find(labels == i);
        pointColors(c, :) = repmat(cmap(i, :), numel(c), 1);
    end
    
    % Plot the clusters
    figure(2)
    pcshow(pc.Location, pointColors, 'MarkerSize', 15);
    xlabel('X[meter]')
    ylabel('Y[meter]')
    zlabel('Z[meter]')
    
    ay = gca;
    ay.Color = 'k';
    ay.XColor = 'w';
    ay.YColor = 'w';
    ay.ZColor = 'w';
    ay.GridColor = 'w';
    
    % Saving 40% of the frames as images
    if mod(frame, frame_interval) == 0
        save_file_name = fullfile(save_folder, sprintf('Frame_%d.png', frame));
        print(gcf, save_file_name, '-dpng', '-r300');
    end
    
    if mod(frame, frame_interval) == 0
        writeVideo(outputVideo, getframe(gcf));
    end
    
    hold off;
    
    pause(0.1);
end

t2 = toc
