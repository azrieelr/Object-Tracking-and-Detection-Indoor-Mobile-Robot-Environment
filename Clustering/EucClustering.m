
clc; clear; clf

% load('FileLidarRH.mat')
% load('FileLidarRB.mat')
load('FileLidarMO.mat')

save_percentage = 0.4; % Save 40% of frames
total_frames = length(scans);
frames_to_save = round(total_frames * save_percentage);
frame_interval = round(1 / save_percentage);

% save_folder = 'saved_framesRH';
% save_folder = 'saved_framesRB';
save_folder = 'saved_framesMO';
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

% Parameters for ground truth
r_circle_true = 0.3; % Circle radius
square_true = 0; % Square vertices
% square_true = [6, 6.5, 6.5, 6;...
%              3, 3, 3.5, 3.5];
%y_square1 = [3, 3, 3.5, 3.5];

% Initialize the accuracy results
accuracy_circle = [];
accuracy_square = [];

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
            % forms an angle close to 90 degrees
            num_right_angles = sum(abs(angles - pi/2) < 0.1);
            
                if num_right_angles >= 5
                    if point_count < 9     
                        classification{i} = 'Square';
                        dimensions(i, :) = [max(xy(:, 1)) - min(xy(:, 1)), max(xy(:, 2)) - min(xy(:, 2))];
                    else 
                        classification{i} = 'Wall';
                        dimensions(i, :) = [max(xy(:, 1)) - min(xy(:, 1)), max(xy(:, 2)) - min(xy(:, 2))];
                    end
                elseif abs(total_angle - pi/2) < 0.1 || point_count > 10
                    if point_count > 11
                        classification{i} = 'Wall';
                        dimensions(i, :) = [max(xy(:, 1)) - min(xy(:, 1)), max(xy(:, 2)) - min(xy(:, 2))];
                    else
                        classification{i} = 'Square';
                        dimensions(i, :) = [max(xy(:, 1)) - min(xy(:, 1)), max(xy(:, 2)) - min(xy(:, 2))];
                    end
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
        if strcmp(classification{i}, ' ')
            min_distance = inf;
            nearest_wall_cluster = i;

            for j = 1:numClusters
                if i ~= j && strcmp(classification{j}, 'Wall')
                    distance = norm(centroids(i, :) - centroids(j, :));
                    if distance < min_distance
                        min_distance = distance;
                        nearest_wall_cluster = j;
                    end
                end
            end
            labels(labels == i) = nearest_wall_cluster;
            classification{i} = classification{nearest_wall_cluster};
            dimensions(i, :) = dimensions(nearest_wall_cluster, :);
        end
    end
    
    % Update point colors based on the new cluster labels
    for i = 1:numClusters
        c = find(labels == i);
        pointColors(c, :) = repmat(cmap(i, :), numel(c), 1);
    end
    
    % Compute the accuracy for each object
    for i = 1:numClusters
        if strcmp(classification{i}, 'Square')
            % Compute the corners of the detected square
            width = dimensions(i, 1);
            height = dimensions(i, 2);
            lower_left = centroids(i, :) - [width height]/2;
            upper_right = lower_left + [width height];
            square_detected = [lower_left(1), upper_right(1), upper_right(1), lower_left(1); ...
                               lower_left(2), lower_left(2), upper_right(2), upper_right(2)];
            
            % Compute the accuracy for square (sum of absolute differences)
            accuracy_square = [accuracy_square, sum(sum(abs(square_detected - square_true)))];
        elseif strcmp(classification{i}, 'Circle')
            % Compute the accuracy for circle (absolute difference of radii)
            accuracy_circle = [accuracy_circle, abs(dimensions(i, 1) - r_circle_true)];
        end
    end
    
    % Plot the clusters
    figure(1)
    pcshow(pc.Location, pointColors, 'MarkerSize', 15);
    xlabel('X[meter]')
    ylabel('Y[meter]')
    zlabel('Z[meter]')
    
    ax = gca;
    ax.Color = 'k';
    ax.XColor = 'w';
    ax.YColor = 'w';
    ax.ZColor = 'w';
    ax.GridColor = 'w';
    
    % Label obstacles as straight line or circle based on angle comparison
    
    hold on;
    
    for i = 1:numClusters
        c = find(labels == i);
        if ~isempty(c)
            centroid = mean(pc.Location(c, :), 1);
            text(centroid(1), centroid(2), centroid(3), [' ' classification{i} ' '], 'Color', cmap(i, :));

            if strcmp(classification{i}, 'Square')
                width = dimensions(i, 1);
                height = dimensions(i, 2);
                lower_left = centroid(1:2) - [width height]/2;

                rectangle('Position', [lower_left, width, height], 'EdgeColor', cmap(i, :), 'LineWidth', 0.5);

                dim_text = sprintf('%.2f', width);
            elseif strcmp(classification{i}, 'Circle')
                dim_text = sprintf('%.2f', dimensions(i, 1));
                viscircles(centroid(1:2), dimensions(i, 1), 'Color', cmap(i, :), 'LineWidth', 0.5);
            else
                dim_text = '';
            end

            text(centroid(1), centroid(2), centroid(3) - 0.5, dim_text, 'Color', cmap(i, :));
        end
    end
    
    % Saving 40% of the frames as images
    if mod(frame, frame_interval) == 0
        save_file_name = fullfile(save_folder, sprintf('Frame_%d.png', frame));
        print(gcf, save_file_name, '-dpng', '-r300');
    end
    
    hold off;
    
    pause(0.1);
end

% Performa hasil deteksi objek

% Display the accuracy results
disp(['Average absolute error for circles: ', num2str(mean(accuracy_circle))]);
disp(['Average absolute error for squares: ', num2str(mean(accuracy_square))]);

% Set up figure (2)
figure(2);

% Subplot for the circle
% subplot(2,1,1); % Divide plot into 2 rows, 1 column, accessing first element
plot(1:length(accuracy_circle), accuracy_circle, 'r');
hold on
plot(1:length(r_circle_true), r_circle_true, 'g');
hold off
% title('Accuracy of Circle Measurements');
xlabel('Frame Number');
ylabel('Radius');
legend('Output Radius', 'Input Radius');

% Set up figure (3)
figure(3);

% Subplot for the square
% subplot(2,1,2); % Divide plot into 2 rows, 1 column, accessing second element
if ~isempty(accuracy_square)
    plot(1:length(accuracy_square), accuracy_square, 'b');
    hold on
    plot(1:length(square_true), square_true, 'g');
    hold off
%     title('Accuracy of Square Measurements');
    xlabel('Frame Number');
    ylabel('Length');
    legend('Output Length', 'Input Length');
else
    title('No Square Measurements Available');
end

% Calculate the measurement errors
% Subtract measured sizes from true sizes
% For the circle
error_measurements_circle = r_circle_true - accuracy_circle;

% For the square
%if ~isempty(accuracy_square)
error_measurements_square = square_true - accuracy_square;
%end


% Calculate MAE and MSE for circle
MAE_circle = mean(abs(error_measurements_circle));
MSE_circle = mean(error_measurements_circle.^2);

% Calculate MAE and MSE for square if there is any square measurements
%if ~isempty(error_measurements_square)
MAE_square = mean(abs(error_measurements_square));
MSE_square = mean(error_measurements_square.^2);
% end

% Set up figure
figure(4);

% Subplot for the circle
% subplot(2,1,1); % Divide plot into 2 rows, 1 column, accessing first element
plot(1:length(accuracy_circle), accuracy_circle, 'r');
hold on;
plot(1:length(r_circle_true), r_circle_true, 'g');
hold off;
title(sprintf('Accuracy of Circle Measurements\nMAE: %.2f, MSE: %.2f', MAE_circle, MSE_circle));
xlabel('Frame Number');
ylabel('Radius');
legend('Output Radius', 'Input Radius');

% Set up figure
figure(5);

% Subplot for the square
% subplot(2,1,2); % Divide plot into 2 rows, 1 column, accessing second element
if ~isempty(accuracy_square)
    plot(1:length(accuracy_square), accuracy_square, 'b');
    hold on;
    plot(1:length(square_true), square_true, 'g');
    hold off;
    title(sprintf('Accuracy of Square Measurements\nMAE: %.2f, MSE: %.2f', MAE_square, MSE_square));
    xlabel('Frame Number');
    ylabel('Side Length');
    legend('Output Side Length', 'Input Side Length');
else
    title('No Square Measurements Available');
end
