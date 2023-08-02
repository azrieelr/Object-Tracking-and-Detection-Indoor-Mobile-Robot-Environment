clc; clear; clf

load('FileLidar.mat');

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
        % Compute the gradient for each point in the cluster
        if size(xy, 1) > 1
            gradient = diff(xy);
            % Normalize the gradient vectors
            gradient = gradient ./ vecnorm(gradient, 2, 2);
            % Compute the angles between consecutive gradient vectors
            angles = acos(dot(gradient(1:end-1, :), gradient(2:end, :), 2));
            % Classify the cluster as a wall or circle based on the sum of angles and point count
            total_angle = sum(angles);
            point_count = numel(c);
            if abs(total_angle - pi/2) < 0.1 || point_count >= 10
                classification{i} = 'Wall';
                dimensions(i, :) = [max(xy(:, 1)) - min(xy(:, 1)), max(xy(:, 2)) - min(xy(:, 2))];
            else
                classification{i} = 'Circle';
                radii = vecnorm(xy - repmat(centroids(i, :), size(xy, 1), 1), 2, 2);
                dimensions(i, 1) = mean(radii);
            end
        else
            classification{i} = ' ';
        end
    end
    
    % Merge undetected clusters into the nearest cluster
    for i = 1:numClusters
        if strcmp(classification{i}, 'Undetected')
            min_distance = inf;
            nearest_cluster = i;
            
            for j = 1:numClusters
                if i ~= j&& ~strcmp(classification{j}, 'Circle')
                    distance = norm(centroids(i, :) - centroids(j, :));
                    if distance < min_distance
                        min_distance = distance;
                        nearest_cluster = j;
                    end
                end
            end
            labels(labels == i) = nearest_cluster;
            classification{i} = classification{nearest_cluster};
            dimensions(i, :) = dimensions(nearest_cluster, :);
        end
    end
    
    % Define parameters
    numTracks = 0;  % initial number of tracks

    % Create multi-object tracker
    tracker = multiObjectTracker('FilterInitializationFcn', @initcvFilter, ...
        'AssignmentThreshold', 30, ...
        'ConfirmationParameters', [4 5], ...
        'DeletionParameters', [5 5]);

    % Iterate through scans
    for frame = 1:length(scans)
        % Your existing code here...

        % Obtain circle indices and initialize detections cell array
        circleIndex = strcmp(classification, 'Circle');
        detections = cell(sum(circleIndex), 1);

        % Define current time
        currentFrame = datetime('now');

        % Extract circle centroids and velocities (zeros in this case)
        circleCentroids = [centroids(circleIndex, :) zeros(sum(circleIndex), 2)];

        % Convert centroids to detections
        for i = 1:length(detections)
            detections{i} = objectDetection(frame, circleCentroids(i, 1:2), 'SensorIndex', 1);
        end

        % Update the tracker
        confirmedTracks = tracker(detections, currentFrame);

        % Update confirmed track count
        numTracks = numel(confirmedTracks);

        % Update tracks with the estimated state
        for i = 1:numTracks
            % Get the track's trajectory
            traj = confirmedTracks(i).Trajectory;

            % Plot the trajectory
            plot(traj(:, 1), traj(:, 2), 'k-', 'LineWidth', 2);
        end

        % Your existing code here...

        hold off;
        pause(0.1);
    end
    
    % Update point colors based on the new cluster labels
    for i = 1:numClusters
        c = find(labels == i);
        pointColors(c, :) = repmat(cmap(i, :), numel(c), 1);
    end

    % Plot the clusters
    figure(1)
    pcshow(pc.Location, pointColors, 'MarkerSize', 15);
    xlabel('X[meter]')
    ylabel('Y[meter]')
    zlabel('Z[meter]')
    
    % Label obstacles as straight line or circle based on angle comparison
    hold on
    for i = 1:numClusters
        c = find(labels == i);
        if ~isempty(c)
            centroid = mean(pc.Location(c, :), 1);
            text(centroid(1), centroid(2), centroid(3), [' ' classification{i} ' '], 'Color', cmap(i, :));
            if strcmp(classification{i}, 'Circle')
                dim_text = sprintf('%.2f', dimensions(i, 1));
                viscircles(centroids(i, :), dimensions(i, 1), 'Color', cmap(i, :), 'LineWidth', 0.5);
            else
                dim_text = '';
            end
            text(centroid(1), centroid(2), centroid(3) - 0.5, dim_text, 'Color', cmap(i, :));
        end
    end
    hold off;
    
    pause(0.1);
end
