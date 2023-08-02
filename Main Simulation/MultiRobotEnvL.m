%% class MultiRobotEnv utk Skenario 5
classdef MultiRobotEnvL < matlab.System
    % MULTIROBOTENV 2D Multi-Robot Environment
    %
    % Displays the pose (position and orientation) of multiple objects in a 
    % 2D environment. Additionally has the option to display a map as a 
    % robotics.OccupancyGrid or robotics.BinaryOccupancyGrid, object
    % trajectories, waypoints, lidar scans, and/or objects.
    %
    % For more information, see <a href="matlab:edit mrsDocMultiRobotEnv">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        numRobots = 1;              % Number of robots
        robotRadius = 0;            % Robot radii [m]
        showTrajectory = false;     % Show trajectory
        mapName = '';               % Map
        hasWaypoints = false;       % Accept waypoints
        hasLidar = false;           % Accept lidar inputs
        hasObjDetector = false;     % Accept object detections
        hasRobotDetector = false;   % Accept robot detections
        plotSensorLines = true;     % Plot sensor lines
        showRobotIds = true;        % Show robot IDs
%         title = '';                 % edit judul visualisasi
    end
    properties
       % Lidar
       sensorOffset = {[0 0]};          % Lidar sensor offset (x,y) [m] 
       scanAngles = {[-pi/4,0,pi/4]};   % Scan angles [rad]
       % Object detectors
       objDetectorOffset = {[0 0]};     % Object detector offset (x,y) [m] 
       objDetectorAngle = 0;            % Object detector angle [rad]
       objDetectorFOV = pi/4;           % Object detector field of view [rad] 
       objDetectorMaxRange = 5;         % Object detector maximum range [m]
       objectColors = [1 0 0;0 1 0;0 0 1]; % Object label colors [RGB rows]
       % Robot detectors
       robotDetectorOffset = {[0 0]};   % Robot detector offset (x,y) [m] 
       robotDetectorAngle = 0;          % Robot detector angle [rad]
       robotDetectorFOV = pi/4;         % Robot detector field of view [rad] 
       robotDetectorMaxRange = 5;       % Robot detector maximum range [m]
       % Other
       Poses; % Robot poses (x,y,theta) [m,m,rad] 
    end

    % Private properties
    properties(Access = private)
        map;                        % Occupancy grid representing the map
        fig;                        % Figure window
        ax;                         % Axes for plotting
        RobotHandle;                % Handle to robot body marker or circle
        OrientationHandle;          % Handle to robot orientation line
        LidarHandles;               % Handle array to lidar lines
        TrajHandle;                 % Handle to trajectory plot
        trajX;                      % X Trajectory points
        trajY;                      % Y Trajectory points
        WaypointHandle;             % Handle to waypoints
        ObjectHandle;               % Handle to objects
        ObjDetectorHandles = {};    % Handle array to object detector lines
        RobotDetectorHandles = {};  % Handle array to robot detector lines
        IdHandles;                  % Handle array to robot IDs
    end

    %% METHODS
    methods    
        % Constructor: Takes number of robots as mandatory argument
        function obj = MultiRobotEnv(N)
            obj.numRobots = N;
        end        
        
        % Constructor 2: jml robot dan judul visualisasi
%         function obj = MultiRobotEnv(N, judul)
%             MultiRobotEnv(N);
%             obj.title = judul;
%         end  
    end
        
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)                 
            % Setup the visualization
            setupVisualization(obj);                
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,robotIndices,poses,varargin)

            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end            
            
            % Unpack the optional arguments
            idx = 1;
            % Waypoints
            if obj.hasWaypoints
                waypoints = varargin{idx};
                idx = idx + 1;
            else
                waypoints = [];
            end
            % Lidar ranges
            if any(obj.hasLidar) 
                ranges = varargin{idx};
                idx = idx + 1;
            else
                ranges = cell(1,obj.numRobots);
            end
            % Objects
            if any(obj.hasObjDetector) 
                objects = varargin{idx};
            else
                objects = [];
            end
            
            % Draw the waypoints and objects
            drawWaypointsAndObjects(obj,waypoints,objects);
           
            % Draw the robots
            if ~isempty(robotIndices)
                drawRobots(obj,robotIndices,poses,ranges);
            end
            
            % Update the figure
            drawnow('limitrate')           
        end    
        
    end
    
    methods (Access = public) 
        
        % Performs all the visualization setup. It is separate from the
        % setup method, since it can be called independently as well.
        function setupVisualization(obj)
            warna = ['r.-','g.-','b.-', 'y.-', 'm.-'];
            warna2 = ['rO', 'gO', 'bO', 'yO', 'mO'];
            warna3 = ['r', 'g', 'b', 'y', 'm', 'c', 'k', 'k','b','y'];
            
            % Convert scalar flags to arrays based on number of robots
            if numel(obj.robotRadius) ~= obj.numRobots
                obj.robotRadius = repmat(obj.robotRadius,[1,obj.numRobots]);
            end
            if numel(obj.showTrajectory) ~= obj.numRobots
                obj.showTrajectory = repmat(obj.showTrajectory,[1,obj.numRobots]);
            end
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar = repmat(obj.hasLidar,[1,obj.numRobots]);
            end
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector = repmat(obj.hasObjDetector,[1,obj.numRobots]);
            end
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector = repmat(obj.hasRobotDetector,[1,obj.numRobots]);
            end
            
            % Initialize poses
            obj.Poses = nan(3,obj.numRobots);
            
            % Create figure
            FigureName = 'Demo Penghindaran Rintangan';
            FigureTag = 'MultiRobotEnvironment';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            
            % Create global axes
            obj.ax = axes('parent',obj.fig);
            hold(obj.ax,'on');

            % Show the map
            obj.map = internal.createMapFromName(obj.mapName);
            if ~isempty(obj.map)
                show(obj.map,'Parent',obj.ax);
            end
            
            % Initialize robot plot
            obj.OrientationHandle = cell(obj.numRobots,1);
            obj.RobotHandle = cell(obj.numRobots,1);
            
            for rIdx = 1:obj.numRobots
                obj.OrientationHandle{rIdx} = plot(obj.ax,0,0,'r','LineWidth',1.5); %menggambar (plot) garis orientasi warna merah ('r') dengan ketebalan 1.5 pixel
                if obj.robotRadius(rIdx) > 0
                    [x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),5);
                    obj.RobotHandle{rIdx} = plot(obj.ax,x,y,warna3(rIdx),'LineWidth',1.5);
                     if rIdx < 9
                         % Finite size robot, utk robot dgn ukuran yg ditentukan
                         [x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),5);
                         obj.RobotHandle{rIdx} = plot(obj.ax,x,y,warna3(rIdx),'LineWidth',1.5);
                     elseif rIdx == 9
                         obj.RobotHandle{rIdx} = plot(obj.ax,0,0,'bx', ...
                         'MarkerSize',obj.robotRadius(rIdx),'LineWidth',0.5);  %% manusia statis
                     elseif rIdx == 10
                         obj.RobotHandle{rIdx} = plot(obj.ax,0,0,'yx', ...
                         'MarkerSize',obj.robotRadius(rIdx),'LineWidth',0.5);  %% manusia statis
                     end
                    %obj.RobotHandle{rIdx} = plot(obj.ax,x,y,'b','LineWidth',1.5); %menggambar (plot) lingkaran biru ('b') dengan ketebalan garis 1.5
                    %obj.RobotHandle{rIdx} = plot(obj.ax,x,y,'r','LineWidth',1.5); %menggambar (plot) lingkaran merah('r') dengan ketebalan garis 1.5
                else
                    % Point robot, utk robot jenis titik
                    obj.RobotHandle{rIdx} = plot(obj.ax,0,0,'bo', ...
                         'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
                end
                % Initialize robot IDs, if enabled
                if obj.showRobotIds
                    %aslinya
% % %                     obj.IdHandles{rIdx} = text(0,0,num2str(rIdx), ... 
% % %                                'Color','b','FontWeight','bold');
                    %diubah senin 10 feb 2020, 16:30
                    if rIdx == 1
                            obj.IdHandles{rIdx} = text(0,0,'robot', ... 
                               'Color','b','FontWeight','bold');                            
                    elseif rIdx == 2
                            obj.IdHandles{rIdx} = text(0,0,'Target', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 3
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 4
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 5
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 6
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 7
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 8
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 9
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    elseif rIdx == 10
                            obj.IdHandles{rIdx} = text(0,0,'obstacle', ... 
                               'Color','b','FontWeight','bold');
                    end
                end
            end
            
            % Initialize trajectory
            obj.TrajHandle = cell(obj.numRobots,1);
            obj.trajX = cell(obj.numRobots,1);
            obj.trajY = cell(obj.numRobots,1);
            
            for rIdx = 1:obj.numRobots
                if obj.showTrajectory(rIdx)
                    %membuat lintasan/trajektori warna biru 'b.-' bentuk garis
                     obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'b.-');
                     obj.TrajHandle{rIdx} = plot(obj.ax,0,0,warna(rIdx));
                    if rIdx==1
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'bo', ...
                        'MarkerSize',obj.robotRadius(rIdx)*5, ...
                        'LineWidth',1);  %% ego-robot     
                    
                         theta = obj.Poses(3,rIdx);
                         x1 = 0 + obj.robotRadius(rIdx)*60*cos(theta);
                         y1 = 0 - obj.robotRadius(rIdx)*60*sin(theta);
                         obj.TrajHandle{rIdx} = plot(obj.ax,x1,y1,'r-', ...
                         'MarkerSize',obj.robotRadius(rIdx)*60, ...
                         'LineWidth',1);  %% ego-robot     
                     
                         x2 = 0 - obj.robotRadius(rIdx)*60*cos(theta);
                         y2 = 0 + obj.robotRadius(rIdx)*60*sin(theta);
                         obj.TrajHandle{rIdx} = plot(obj.ax,x2,y2,'r-', ...
                         'MarkerSize',obj.robotRadius(rIdx)*60, ...
                         'LineWidth',1);  %% ego-robot
                    
                    elseif rIdx==2
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'g-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1);  %% target#1
                    elseif rIdx==3
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'b-.', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',0.5);  %% manusia tunggal                  
                    elseif rIdx==4
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'y-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1);  %% target#2                  
                    elseif rIdx==5
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'m:', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',0.5);  %% grup manusia                  
                    elseif rIdx==6
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'c-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1);  %% target#3
                    elseif rIdx==7
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'k-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',0.5);  %% manusia statis                  
                    elseif rIdx==8
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'k-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1);  %% target#4
                    elseif rIdx==9
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'b--', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',0.5);  %% robot transpor beroda                  
                    elseif rIdx==10
                        obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'y-', ...
                        'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1);  %% target#5
                    end
                    %membuat lintasan/trajektori warna merah 'ro' bentuk lingkaran
                    obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'rO');
                    %membuat garis lintasan/trajektori warna hijau 'g.-'
                     obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'gO', ...
                         'MarkerSize',obj.robotRadius(rIdx)*30,'LineWidth',1');                    
                    %membuat garis lintasan/trajektori spt obj robot
                    [x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),17);
                    obj.TrajHandle{rIdx} = plot(obj.ax,x,y,'b','LineWidth',1.5);
                    obj.trajX{rIdx} = [];
                    obj.trajY{rIdx} = [];
                end
            end
            
            % Initialize waypoints
            if obj.hasWaypoints
                obj.WaypointHandle = plot(obj.ax,0,0, ...
                    'rx','MarkerSize',10,'LineWidth',2);
%                   obj.WaypointHandle = plot(obj.ax,0,0,'ys', ...
%                         'MarkerSize',obj.robotRadius(rIdx),'LineWidth',25,...
%                         'MarkerFaceColor','y','MarkerEdgeColor','y');  %% Sabtu15Mei2021
            end
            
            % Initialize lidar lines
            obj.LidarHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasLidar(rIdx)
                    for idx = 1:numel(obj.scanAngles{rIdx})
                        obj.LidarHandles{rIdx}(idx) = plot(obj.ax,0,0,'b--');
                    end
                end
            end
            
            % Initialize objects and object detector lines
            obj.ObjectHandle = scatter(obj.ax,[],[],75,'s','filled', ...
                    'LineWidth',20);
            obj.ObjDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasObjDetector(rIdx)
                    objDetectorFormat = 'g-.';
                    obj.ObjDetectorHandles{rIdx}(1) = plot(obj.ax,0,0,objDetectorFormat); % Left line
                    obj.ObjDetectorHandles{rIdx}(2) = plot(obj.ax,0,0,objDetectorFormat); % Right line
                end
            end

            % Initialize robot detector lines
            obj.RobotDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                    robotDetectorFormat = 'm:';
                    obj.RobotDetectorHandles{rIdx} = [ plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5), ... % Left line
                                                       plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5) ];    % Right line
            end
            
            % Final setup
            % title(obj.ax,'Multi-Robot Visualization');
            % title(obj.ax,'Visualisasi Multi-Robot');
            %title(obj.ax,'Demo Penghindaran Rintangan by Muhammad Fuad');
            title(obj.ax,'Collision Avoidance');
            hold(obj.ax,'off');
            axis equal
            
            % inisialisasi legenda bisa pakai RobotHandle
            % kode di bawah ini dipakai utk objek berjumlah 0
            % hanya ada mobile robot dan targetnya saja
             leg = legend( ...
                 [ ...
                     obj.RobotHandle{1}, obj.RobotHandle{2}, ...
                     obj.TrajHandle{1}, obj.TrajHandle{2}, ...
                 ], ...                
                 'Ego-Robot','Target#1', ...
                 'Jarak ke Target','Jarak ke Wall', ...
                 'Location','northeastOutside');
            % kode di bawah ini dipakai utk objek berjumlah 2 
%             leg = legend( ...
%                  [ ...
%                      obj.RobotHandle{1}, obj.RobotHandle{2}, ...
%                      obj.RobotHandle{3}, ...
%                      obj.TrajHandle{1}, obj.TrajHandle{2}, ...
%                      obj.TrajHandle{3} ...
%                  ], ...                
%                  'Ego-Robot','Target#1', ...
%                  'Obstacle#1', ...
%                  'Jarak ke Target','Jarak ke Obst#1', ...
%                  'Jarak ke Obst#2', ...
%                  'Location','northeastOutside');
            % kode di bawah ini dipakai utk objek berjumlah 10
%              leg = legend( ...
%                  [ ...
%                      obj.RobotHandle{1}, obj.RobotHandle{2}, ...
%                      obj.RobotHandle{3}, obj.RobotHandle{4}, ...
%                      obj.RobotHandle{5}, obj.RobotHandle{6}, ...
%                      obj.RobotHandle{7}, obj.RobotHandle{8}, ...
%                      obj.RobotHandle{9}, obj.RobotHandle{10}, ...
%                      obj.TrajHandle{1}, obj.TrajHandle{2}, ...
%                      obj.TrajHandle{3}, obj.TrajHandle{4}, ...
%                      obj.TrajHandle{5}, obj.TrajHandle{6}, ...
%                      obj.TrajHandle{7}, obj.TrajHandle{8}, ...
%                      obj.TrajHandle{9}, obj.TrajHandle{10} ...
%                  ], ...                
%                  'Ego-Robot','Target#1', ...
%                  'Obstacle#1','Target#2',...
%                  'Obstacle#2','Target#3', ...                
%                  'Obstacle#3','Target#4', ...
%                  'Obstacle#4','Target#5', ...
%                  'Jarak ke Target','Jarak ke Obst#1', ...
%                  'Jarak ke Obst#2','Jarak ke Obst#3', ...
%                  'Jarak ke Obst#4', ...
%                  'Jarak Obs#1 ke Tar#2', ...
%                  'Jarak Obs#2 ke Tar#3', ...
%                  'Jarak Obs#3 ke Tar#4', ...
%                  'Jarak Obs#4 ke Tar#5', 'Status Benturan', ...
%                  'Location','northeastOutside');
            %title(leg, 'Waktu Sampai Target: ');
            title(leg, 'Time to Target: ');
            % inisialisasi legenda juga bisa pakai TrajHandle
%               legend([obj.TrajHandle{1}, obj.TrajHandle{2}, ...
%                  obj.TrajHandle{3}, obj.TrajHandle{4}, ...
%                  obj.TrajHandle{5}], ...
%                  'Ego-Robot','Target', ...
%                  'Obstacle#1','Obstacle#2','Obstacle#3', ...
%                  'Location','northeastOutside')
        end
        
        % Helper method to draw the waypoints and objects,
        % which are independent of the robot
        function drawWaypointsAndObjects(obj,waypoints,objects)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update waypoints
            if obj.hasWaypoints && (numel(waypoints) > 1)
                set(obj.WaypointHandle,'xdata',waypoints(:,1), ...
                                       'ydata',waypoints(:,2));
            else
                set(obj.WaypointHandle,'xdata',[], ...
                                       'ydata',[]);
            end
            
            % Update the objects
            if numel(objects) <= 1
                set(obj.ObjectHandle,'xdata',[],'ydata',[],'cdata',[]);
            else
                % Plot the objects with their corresponding colors
                if size(obj.objectColors,1) == 1
                    colorData = obj.objectColors; % Use the single color specified
                else
                    %% objectColors = [1 0 0;0 1 0;0 0 1]; % Object label colors [RGB rows] %%
                    colorData = obj.objectColors(objects(:,3),:); % Use the color based on labels
                end
                set(obj.ObjectHandle,'xdata',objects(:,1),...
                    'ydata',objects(:,2),'cdata',colorData);
            end
        end
        
        % Helper method to draw all robots (calls drawRobot)
        function drawRobots(obj,robotIndices,poses,ranges)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end

            % Draw each robot and its sensors              
            % Single-robot case
            if numel(robotIndices) == 1
               obj.Poses(:,robotIndices) = poses;
               drawRobot(obj,robotIndices,poses,ranges); 
            % Multi-robot case
            else
                for rIdx = robotIndices
                    pose = poses(:,rIdx);
                    obj.Poses(:,rIdx) = pose;
                    drawRobot(obj,rIdx,pose,ranges{rIdx}); 
                end
            end
        end
        
        % Helper method to draw the robot and its sensors at each step
        function drawRobot(obj,rIdx,pose,ranges)
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Update the trajectory
            if obj.showTrajectory(rIdx)
                %ini aslinya gak ada yg pas di bawah ini
                %[x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),5);
                obj.trajX{rIdx}(end+1) = x;
                obj.trajY{rIdx}(end+1) = y;
                set(obj.TrajHandle{rIdx},'xdata',obj.trajX{rIdx}, ...
                    'ydata',obj.trajY{rIdx});
            end
            
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
            r = obj.robotRadius(rIdx);
            if r > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,r,17);
                set(obj.RobotHandle{rIdx},'xdata',xc,'ydata',yc);
                len = max(lineLength,2*r); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle{rIdx},'xdata',x,'ydata',y);
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            end
            % Show robot IDs, if enabled
            if obj.showRobotIds                
                set(obj.IdHandles{rIdx},'Position',[x y] - max(1.25*r,lineLength*0.5));
            end
            
            % Update lidar lines
            if obj.hasLidar(rIdx) && obj.plotSensorLines
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.sensorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                scanAngs = obj.scanAngles{rIdx};
                for idx = 1:numel(ranges)
                    if ~isnan(ranges(idx))
                        % If there is a single sensor offset, use that value
                        if size(sensorLoc,1) == 1
                            sensorPos = sensorLoc;
                            % Else, use the specific index's sensor location
                        else
                            sensorPos = sensorLoc(idx,:);
                        end
                        
                        alpha = theta + scanAngs(idx);
                        lidarX = sensorPos(1) + [0, ranges(idx)*cos(alpha)];
                        lidarY = sensorPos(2) + [0, ranges(idx)*sin(alpha)];
                        set(obj.LidarHandles{rIdx}(idx),'xdata',lidarX, ...
                            'ydata',lidarY);
                    else
                        set(obj.LidarHandles{rIdx}(idx),'xdata',[],'ydata',[]);
                    end
                end
            end
            
            % Update object and object detector lines  
            if obj.hasObjDetector(rIdx) && obj.plotSensorLines
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.objDetectorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                % Plot the object detector lines
                maxRange = obj.objDetectorMaxRange(rIdx);
                % Left line
                alphaLeft = theta + obj.objDetectorAngle(rIdx) + obj.objDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsLeft = rayIntersection(obj.map,[sensorLoc alphaLeft],0,maxRange);
                else
                    intPtsLeft = NaN;
                end
                if ~isnan(intPtsLeft)
                    objLeftX = [sensorLoc(1) intPtsLeft(1)];
                    objLeftY = [sensorLoc(2) intPtsLeft(2)];
                else
                    objLeftX = sensorLoc(1) + [0, maxRange*cos(alphaLeft)];
                    objLeftY = sensorLoc(2) + [0, maxRange*sin(alphaLeft)];
                end
                set(obj.ObjDetectorHandles{rIdx}(1), ...
                    'xdata',objLeftX,'ydata',objLeftY);
                % Right line
                alphaRight = theta + obj.objDetectorAngle(rIdx) - obj.objDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsRight = rayIntersection(obj.map,[sensorLoc alphaRight],0,maxRange);
                else
                    intPtsRight = NaN;
                end
                if ~isnan(intPtsRight)
                    objRightX = [sensorLoc(1) intPtsRight(1)];
                    objRightY = [sensorLoc(2) intPtsRight(2)];
                else
                    objRightX = sensorLoc(1) + [0, maxRange*cos(alphaRight)];
                    objRightY = sensorLoc(2) + [0, maxRange*sin(alphaRight)];
                end
                set(obj.ObjDetectorHandles{rIdx}(2), ...
                    'xdata',objRightX,'ydata',objRightY);
            end
            
            % Update robot detector lines
            if obj.hasRobotDetector(rIdx) && obj.plotSensorLines
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.robotDetectorOffset{rIdx}';
                sensorLoc = [x,y] + offsetVec';
                
                % Plot the robot detector lines
                maxRange = obj.robotDetectorMaxRange(rIdx);
                % Left line
                alphaLeft = theta + obj.robotDetectorAngle(rIdx) + obj.robotDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsLeft = rayIntersection(obj.map,[sensorLoc alphaLeft],0,maxRange);
                else
                    intPtsLeft = NaN;
                end
                if ~isnan(intPtsLeft)
                    objLeftX = [sensorLoc(1) intPtsLeft(1)];
                    objLeftY = [sensorLoc(2) intPtsLeft(2)];
                else
                    objLeftX = sensorLoc(1) + [0, maxRange*cos(alphaLeft)];
                    objLeftY = sensorLoc(2) + [0, maxRange*sin(alphaLeft)];
                end
                set(obj.RobotDetectorHandles{rIdx}(1), ...
                    'xdata',objLeftX,'ydata',objLeftY);
                % Right line
                alphaRight = theta + obj.robotDetectorAngle(rIdx) - obj.robotDetectorFOV(rIdx)/2;
                if ~isempty(obj.map)
                    intPtsRight = rayIntersection(obj.map,[sensorLoc alphaRight],0,maxRange);
                else
                    intPtsRight = NaN;
                end
                if ~isnan(intPtsRight)
                    objRightX = [sensorLoc(1) intPtsRight(1)];
                    objRightY = [sensorLoc(2) intPtsRight(2)];
                else
                    objRightX = sensorLoc(1) + [0, maxRange*cos(alphaRight)];
                    objRightY = sensorLoc(2) + [0, maxRange*sin(alphaRight)];
                end
                set(obj.RobotDetectorHandles{rIdx}(2), ...
                    'xdata',objRightX,'ydata',objRightY);
            end            
           
        end    
           
        % Attaches all properties associated with a LidarSensor object
        function attachLidarSensor(obj,rIdx,lidar)
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar = repmat(obj.hasLidar,[1,obj.numRobots]);
            end
            obj.hasLidar(rIdx) = true;
            obj.sensorOffset{rIdx} = lidar.sensorOffset;
            obj.scanAngles{rIdx} = lidar.scanAngles;
            
            % Ensure to use the same map as the visualizer
            release(lidar);
            lidar.mapName = obj.mapName;
            lidar.isMultiRobot = true;
            lidar.robotIdx = rIdx;
            setEnvironment(lidar,obj);
        end
        
        % Attaches all properties associated with an ObjectDetector object
        function attachObjectDetector(obj,rIdx,detector)
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector = repmat(obj.hasObjDetector,[1,obj.numRobots]);
            end 
            obj.hasObjDetector(rIdx) = true;
            obj.objDetectorOffset{rIdx} = detector.sensorOffset;
            obj.objDetectorAngle(rIdx) = detector.sensorAngle;
            obj.objDetectorFOV(rIdx) = detector.fieldOfView;
            obj.objDetectorMaxRange(rIdx) = detector.maxRange;
            
            % Ensure to use the same map as the visualizer
            release(detector);
            detector.mapName = obj.mapName;
        end

        % Attaches all properties associated with a RobotDetector object
        function attachRobotDetector(obj,detector)
            release(obj);
            
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector = repmat(obj.hasRobotDetector,[1,obj.numRobots]);
            end
            
            rIdx = detector.robotIdx;
            obj.hasRobotDetector(rIdx) = true;
            obj.robotDetectorOffset{rIdx} = detector.sensorOffset;
            obj.robotDetectorAngle(rIdx) = detector.sensorAngle;
            obj.robotDetectorFOV(rIdx) = detector.fieldOfView;
            obj.robotDetectorMaxRange(rIdx) = detector.maxRange;
        end
        
    end
        
end