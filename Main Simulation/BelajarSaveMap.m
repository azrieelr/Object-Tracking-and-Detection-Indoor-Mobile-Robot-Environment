clc; clear; 

load robotSimulatorExampleMaps.mat
peta2 = binaryOccupancyMap(borderMap, 3.35);
lingkungan.mapName = 'peta2';

x_square = [8, 8.5, 8.5, 8];
y_square = [11, 11, 11.5, 11.5];
x_square1 = [6, 6.5, 6.5, 6];
y_square1 = [3, 3, 3.5, 3.5];

setOccupancy(peta2, [x_square' y_square'], ones(length(x_square), 1), 'world');
setOccupancy(peta2, [x_square1' y_square1'], ones(length(x_square1), 1), 'world');
inflate(peta2, 0.1)

% assuming 'map' is your binaryOccupancyMap
mapMatrix = occupancyMatrix(peta2); % Convert to matrix form

figure;
imshow(mapMatrix, 'InitialMagnification', 'fit'); % Display the map
map = [1 1 1; 
       0 0 0];
colormap(map); % Optional: use grayscale
axis equal;

% Save the figure
% set(gcf, 'PaperPositionMode', 'auto'); % Ensure saved figure has the same aspect ratio
% print('mase4.tiff', '-dtiff'); % Save as PNG
