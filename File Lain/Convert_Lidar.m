clc; clear;

% Load LIDAR data
load('FileLidar.mat');

% Initializations
im_size = [1024, 1024]; % desired image size

% Specify the resolution of the grid (size of each cell in meters)
gridResolution = 0.5;

% Specify the size of the grid in meters
gridSize = [20 20];

for frame = 1:length(scans)
    % Initialize an image for this frame
    img = zeros(im_size);

    xyzPoints = scans{frame}.Cartesian;

    % Normalize the points to fit within the image dimensions
    normalizedPoints = bsxfun(@minus, xyzPoints, min(xyzPoints));
    normalizedPoints = bsxfun(@rdivide, normalizedPoints, max(normalizedPoints));
    pixelPoints = round(bsxfun(@times, normalizedPoints(:,1:2), im_size-1) + 1);

    % Add points to image
    for i = 1:size(pixelPoints, 1)
        img(pixelPoints(i, 2), pixelPoints(i, 1)) = 1; % set pixel to some value
    end

    % Display the image
    figure(1);
    imshow(img);
    title(sprintf('Frame %d - Image', frame));

    % Initialize the occupancy grid
    occupancyGrid = robotics.OccupancyGrid(gridSize(1), gridSize(2), 1/gridResolution);

    % Convert points to grid coordinates
    gridPoints = ceil(xyzPoints(:,1:2) / gridResolution);

    % Update occupancy grid
    for i = 1:size(gridPoints, 1)
        % Make sure grid points are within the grid size
        if all(gridPoints(i,:) > 0) && all(gridPoints(i,:) <= gridSize/gridResolution)
            occupancyGrid.setOccupancy(gridPoints(i,:), 1);
        end
    end

    % Display the occupancy grid
    figure(2);
    show(occupancyGrid);
    title(sprintf('Frame %d - Occupancy Grid', frame));
end
