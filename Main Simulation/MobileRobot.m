clc; clear all; close all

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:25;         % Time array

initPose = [2;2;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Path planning
% Load map and inflate it by a safety distance
close all
load robotSimulatorExampleMaps.mat
peta = binaryOccupancyMap(borderMap, 3.35);
inflate(peta,0.59);

% Create a Probabilistic Road Map (PRM)
%planner = mobileRobotPRM(map);
%planner.NumNodes = 75;
%planner.ConnectionDistance = 5;

% Find a path from the start point to a specified goal point
startPoint = initPose(1:2)'; %titik awal
goalPoint  = [11, 11]; %titik akhir
%waypoints = findpath(planner,startPoint,goalPoint);
waypoints = [2 2; 11 11]; %kalo mau ganti titik-titik path planning
%show(planner);

%% Pure Pursuit Controller
controller = controllerPurePursuit; %controller untuk path following
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Create visualizer 
load robotSimulatorExampleMaps.mat % Reload original (uninflated) map for visualization
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'peta';

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,200);
lidar.maxRange = 20; %atur range lidar

% % Create objects [x,y,label]
% objects = [4, 8, 1; ...
%            8, 10, 2; ...
%            10, 8.5, 3; ...
%            12, 9, 4];

% Create object Detector sensor
% detector = ObjectDetector;
% detector.fieldOfView = pi/4;

% Create visualizer
% attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1;1 1 0];
viz.objectMarkers = 'so^k';
attachLidarSensor(viz,lidar);

%% Simulation loop
r = rateControl(1/sampleTime);
range = [];
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    ranges = lidar(pose(:,idx));    % Nampilin Lidar untuk akhir iterasi
    range = [range ranges];         % Nampilin loop append data Lidar 
    distance = [zeros(200,1) range]; % Menambah nilai nol pada detik awal
%     detections = detector(pose(:,idx),objects);
    viz(pose(:,idx),waypoints,ranges)
    
    % Display object detections every 10th iteration
    %if mod(idx,10) == 0
    %    if ~isempty(detections)
    %        nearestLabel = detections(1,3);
    %        disp(['Nearest object is of label ' num2str(nearestLabel)]); 
    %    else
    %        disp('No objects detected'); 
    %    end
    %end  
    waitfor(r);
end

%% Visualisasi 
TimeStamp = [tVec; distance];   % Gabung nilai waktu dan jarak
figure(2)
title("Actual Path Mobile Robot")
plot(pose(1,:),pose(2,:));      % Plot posisi robot 
