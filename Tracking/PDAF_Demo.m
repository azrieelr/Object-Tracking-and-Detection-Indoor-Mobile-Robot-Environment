
clc; clear; clf; 

% Masukan data x,y,theta sebagai input awal 
% load('FilePoseRB.mat')
% load('FilePoseMO.mat')
load('FilePoseRH.mat')

% Number of targets to be tracked (e.g. n)
TrackNum = size(PoseXObs, 2);

% Generate measurements (including clutter) from ground truth
DataList = gen_obs_cluttered_multi(TrackNum, PoseXObs, PoseYObs);
%DataList = gen_obs_cluttered_multi(TrackNum, PoseXObs, PoseYObs, PoseORObs);

% Create variable to store RMSE for each track
RMSE_enkf = zeros(2, TrackNum);

% Number of simulation runs
SimNum = 1;

% Plotting variables
ShowPlots = 1; % Enable/Disable plots
SkipFrames = 5; % Skip n frames between plots

% for each simulation run
for sim = 1:SimNum

    %% Initiate KF parameters
    n = 4;      %number of state
    q = 0.03;    %std of process 
    r = 0.125;    %std of measurement
    s.Q = [1^3/3, 0, 1^2/2, 0;  0, 1^3/3, 0, 1^2/2; 1^2/2, 0, 1, 0; 0, 1^2/2, 0, 1]*10*q^2; % covariance of process
    s.R = r^2*eye(n/2);        % covariance of measurement  
    s.sys = (@(x)[x(1)+ x(3); x(2)+x(4); x(3); x(4)]);     % assuming measurements arrive 1 per sec
    s.obs = @(x)[x(1);x(2)];                          % measurement equation
    st = [PoseXObs(1,:); PoseYObs(1,:)];   % initial state
    %st = [PoseXObs(1,:); PoseYObs(1,:); PoseORObs(1,:)];   % initial state
    s.x_init = [];
    s.P_init = [];

    %% Perform Track Initiations
    
    % 1) Two-point difference initiation
    %for i = 1:TrackNum
    %    s.x_init(:,i)=[DataList(1,i,2); DataList(2,i,2); DataList(1,i,2)-DataList(1,i,1); DataList(2,i,2)-DataList(2,i,1)]; %initial state
    %end
    %s.P_init = [q^2, 0, q^2, 0;
    %            0, q^2, 0, q^2;
    %            q^2, 0, 2*q^2, 0;
    %            0, q^2, 0, 2*q^2];     % initial state covraiance

    % 2)Single-point initiation
    Vmax = 0.5; % Max velocity = 0.4 m/s
    for i = 1:TrackNum
        s.x_init(:,i)=[DataList(1,i,2); DataList(2,i,2); 0; 0]; %initial state
    end
    s.P_init = diag([q^2, q^2, (Vmax^2/3), (Vmax^2/3)]);

    N=size(DataList,3) ;   % total dynamic steps

    %% Instantiate vars to store output
    Logs = [];
    Log.xV_enkf = zeros(n,N);    % mean estimates
    Log.PV_enkf = zeros(n,n,N);  % covariance estimates    
    Log.sV_enkf = zeros(n/2,N);  % ground truth
    Log.zV_enkf = zeros(n/2,N);  % measurements
    Log.eV_enkf = zeros(n/2,N);  % RMSE
    
    %% Process and Observation handles and covariances
    TrackObj.sys        = s.sys;
    TrackObj.obs        = s.obs;
    TrackObj.Q          = s.Q;
    TrackObj.R          = s.R;
    TrackObj.P          = s.P_init;

    %% Initiate Tracklist and Log for each target
    TrackList = [];
    for i=1:TrackNum
        
        TrackObj.x          = s.x_init(:,i);
        TrackList{i}.TrackObj = TrackObj;
        
        Logs{i}             = Log;
        Logs{i}.xV_enkf(:,1) = s.x_init(:,i);
        Logs{i}.PV_enkf(:,:,1) = s.P_init;
        Logs{i}.sV_enkf(:,1) = st(:,i);
    end

    %% Create enkf instance to perform Track prediction in Observation Association function 
    enkf = EnKalmanFilter(TrackObj, 0.5, 1, 2);

    %% Prepare background image from plot
    %img = imread('maze3.png');

    % set the range of the axes
    % The image will be stretched to this.
    min_x = 0;
    max_x = 15;
    min_y = 0;
    max_y = 15;

    % make data to plot - just a line.
    x = min_x:max_x;
    y = (6/8)*x;

    figure()
    
    %% Simulation
    for i=1:N

        fprintf('Iteration = %d/%d\n',i,N);
        %tempDataList = DataList(:,:,i);
        %tempDataList( :, ~any(tempDataList,1) ) = []; 
        
        % Run Observation Association to perform track update for all targets and generate the Validation matrix.
        [TrackList, ValidationMatrix, bettaNTFA] = Observation_Association(TrackList, DataList(:,:,i), enkf);
        
        % JPDAF_enkf_Update provides JPDAF updates for all targets, using a enkf.
        TrackList = JPDAF_ENKF_Update(TrackList, DataList(:,:,i), ValidationMatrix', bettaNTFA);
        %TrackList = Track_InitConfDel(TrackList,tempDataList,ValidationMatrix',bettaNTFA, betta);
        
        % Update Logs
        for j=1:TrackNum
            Logs{j}.xV_enkf(:,i) = TrackList{j}.TrackObj.x;
            Logs{j}.PV_enkf(:,:,i) = TrackList{j}.TrackObj.P;
            st = [PoseXObs(i,j); PoseYObs(i,j)];
            %st = [PoseXObs(i,j); PoseYObs(i,j); PoseORObs(i,j)];
            Logs{j}.sV_enkf(:,i)= st;
            % Compute squared error
            Logs{j}.eV_enkf(:,i) = (TrackList{j}.TrackObj.x(1:2,1) - st).*(TrackList{j}.TrackObj.x(1:2,1) - st);
        end
        
        % Visualise the process (Set ShowPlots=0 to disable)
        if (ShowPlots)
            % Plot data
            
            if(i==1 || rem(i,SkipFrames+1)==0)
                clf;
                 
                 % Flip the image upside down before showing it
%                 imagesc([min_x max_x], [min_y max_y], flipud(img)); %with map bacground
                imagesc([min_x max_x], [min_y max_y]); %without map background
                hold on
                
                % Draw ground truth for all targets
                for j=1:TrackNum
                    h2 = plot(Logs{j}.sV_enkf(1,1:i),Logs{j}.sV_enkf(2,1:i),'b.-','LineWidth',1);
                    if j~=1
                        set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                    end
                    h2 = plot(Logs{j}.sV_enkf(1,i),Logs{j}.sV_enkf(2,i),'bo','MarkerSize', 10);
                    set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Exclude line from legend
                end
                
                % Draw all measurements
                h2 = plot(DataList(1,:,i),DataList(2,:,i),'k*','MarkerSize', 10);
                
                % Draw estimated position mean and covariance
                for j=1:TrackNum
                    colour = 'r';
                    if(j==2)
                        colour = 'c';
                    elseif (j==3)
                        colour = 'm';
                    elseif (j==4)
                        colour = 'k';
                    elseif (j==5)
                        colour = [0.4940 0.1840 0.5560];
                    elseif (j==6)
                        colour = [0.9290 0.6940 0.1250];
                    elseif (j==7)
                        colour = 'y';
                    elseif (j==9)
                        colour = 'g';
                    elseif (j==10)
                        colour = [0 0.4470 0.7410];
                    elseif (j==11)
                        colour = [0.8500 0.3250 0.0980];
                    end
                    h4 = plot(Logs{j}.xV_enkf(1,1:i),Logs{j}.xV_enkf(2,1:i),strcat(colour,'.-'),'LineWidth',1);
                    %h4 = plot(Logs{j}.xV_enkf(1,i),Logs{j}.xV_enkf(2,i),strcat(colour,'o'),'MarkerSize', 10);
                    set(get(get(h4,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
                    h2=plot_gaussian_ellipsoid(Logs{j}.xV_enkf(1:2,i), Logs{j}.PV_enkf(1:2,1:2,i));
                    set(h2,'color',colour);
                    set(h2,'LineWidth',1);
                end
                
                % set the y-axis back to normal.
                set(gca,'ydir','normal');
                str = sprintf('Estimated state x_{1,k} vs. x_{2,k}');
                title(str);
                xlabel('X position (m)');
                ylabel('Y position (m)');
                
                if TrackNum==1
                    h_legend = legend('Real', 'Meas', 'Target 1');
                elseif TrackNum == 2
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2');
                elseif TrackNum == 3
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3');
                elseif TrackNum == 4
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4');
                elseif TrackNum == 5
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5');
                elseif TrackNum == 6
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6');
                elseif TrackNum == 7
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7');
                elseif TrackNum == 8
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8');
                elseif TrackNum == 9
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9');
                elseif TrackNum == 10
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9', 'Target 10');
                elseif TrackNum == 11
                    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9', 'Target 10', 'Target 11');
                end
                
                set(h_legend,'FontSize',9, 'Orientation', 'vertical', 'Location', 'southwest');
                axis([0 15 0 15]);
                pause(0.01)
            end
        end
    end

    % Compute & Print RMSE
    RMSE = zeros(2, TrackNum);
    for i=1:TrackNum
        RMSE(:,i) = sqrt(sum(Logs{i}.eV_enkf,2)/N);
    end
    RMSE_enkf = RMSE_enkf + RMSE;

end
RMSE_enkf = RMSE_enkf/SimNum

% Average RMSE over all simulations
RMSE_enkf_avg = RMSE_enkf / SimNum;

% Plot RMSE
figure()
for i=1:TrackNum
    plot(1:N, sqrt(Logs{i}.eV_enkf(1,:)), 'b-', 1:N, sqrt(Logs{i}.eV_enkf(2,:)), 'r-');
    hold on
end
grid on
title('RMSE of Position Estimates')
xlabel('Time step')
ylabel('RMSE')
legend('X Position', 'Y Position')

% Plot average RMSE
figure()
plot(1:TrackNum, RMSE_enkf_avg(1,:), 'bo-', 1:TrackNum, RMSE_enkf_avg(2,:), 'ro-');
grid on
title('Average RMSE of Position Estimates')
xlabel('Track')
ylabel('Average RMSE')
legend('X Position', 'Y Position')

% figure(4)
for i=1:TrackNum
    % Calculate error between true position and enkf prediction for each track
    error_x(1,:,i) = Logs{i}.sV_enkf(1,:) - Logs{i}.xV_enkf(1,:);
    error_y(1,:,i) = Logs{i}.sV_enkf(2,:) - Logs{i}.xV_enkf(2,:);
    
%     subplot(TrackNum, 2, 2*i-1);  % plot for x error
%     plot(error_x, 'r');
%     title(sprintf('Error Position X for Track %d', i));
%     xlabel('Time Step');
%     ylabel('Error');
% 
%     subplot(TrackNum, 2, 2*i);  % plot for y error
%     plot(error_y, 'b');
%     title(sprintf('Error Position Y for Track %d', i));
%     xlabel('Time Step');
%     ylabel('Error');
end

% Kode di bawah ini sampai 310 digunakan untuk plot hasil error pada skenario terakhir  
figure()
title(sprintf('Error Position X'))

for z = 1:5
    subplot(3,2,z)
    plot(squeeze(error_x(1,:,z)),'r');
    xlim([1 145])
    title(sprintf('Error Position X for track %d',z));
    xlabel('Time Step');
    ylabel('Error');
end

figure()
for z = 1:6
    subplot(3,2,z)
    plot(squeeze(error_x(1,:,z+5)),'r');
    xlim([1 145])
    title(sprintf('Error Position X for track %d', z));
    xlabel('Time Step');
    ylabel('Error');
end

figure()
for z = 1:5
    subplot(3,2,z)
    plot(squeeze(error_y(1,:,z)),'b');
    xlim([1 145])
    title(sprintf('Error Position Y for track %d',z));
    xlabel('Time Step');
    ylabel('Error');
end

figure()
for z = 1:6
    subplot(3,2,z)
    plot(squeeze(error_y(1,:,z+5)),'b');
    xlim([1 145])
    title(sprintf('Error Position Y for track %d',z));
    xlabel('Time Step');
    ylabel('Error');
end

% Plot for position error
figure()
for i=1:TrackNum
    % Create a subplot for each track
    subplot(TrackNum, 1, i); 
    
    % Calculate position error and plot
    error_position = sqrt(Logs{i}.eV_enkf);
    plot(error_position, 'g');

    % Add title and labels to each subplot
    title(sprintf('Filter Position Error %d', i));
    xlabel('Time Step');
    ylabel('Position Error');
end

% Draw history/end plot
figure()
% Flip the image upside down before showing it
% imagesc([min_x max_x], [min_y max_y], flipud(img)); %with map bacground
imagesc([min_x max_x], [min_y max_y]); %without map background

hold on;
 for j=1:TrackNum
    h2 = plot(Logs{j}.sV_enkf(1,:),Logs{j}.sV_enkf(2,:),'b.-','LineWidth',1);
    if j>=5
        set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
    h2 = plot(Logs{j}.sV_enkf(1,end),Logs{j}.sV_enkf(2,end),'bo','MarkerSize', 10);
    set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Exclude line from legend
 end

h2 = plot(DataList(1,:,end),DataList(2,:,end),'k*','MarkerSize', 10);
for j=1:TrackNum
    colour = 'r';
    if(j==2)
        colour = 'c';
    elseif (j==3)
        colour = 'm';
    elseif (j==4)
        colour = 'k';
    elseif (j==5)
        colour = [0.4940 0.1840 0.5560];
    elseif (j==6)
        colour = [0.9290 0.6940 0.1250];
    elseif (j==7)
        colour = 'y';
    elseif (j==9)
        colour = 'g';
    elseif (j==10)
        colour = [0 0.4470 0.7410];
    elseif (j==11)
        colour = [0.8500 0.3250 0.0980];
    end
    h4 = plot(Logs{j}.xV_enkf(1,:),Logs{j}.xV_enkf(2,:),strcat(colour,'.-'),'LineWidth',1);
    %h4 = plot(Logs{j}.xV_enkf(1,end),Logs{j}.xV_enkf(2,end),strcat(colour,'o'),'MarkerSize', 10);
    set(get(get(h4,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    h2=plot_gaussian_ellipsoid(Logs{j}.xV_enkf(1:2,end), Logs{j}.PV_enkf(1:2,1:2,end));
    set(h2,'color',colour);
    set(h2,'LineWidth',1);
end

% set the y-axis back to normal.
set(gca,'ydir','normal');
str = sprintf('Estimated state x_{1,k} vs. x_{2,k}');
title(str);
xlabel('X position (m)');
ylabel('Y position (m)');
if TrackNum==1
    h_legend = legend('Real', 'Meas', 'Target 1');
elseif TrackNum == 2
    h_legend = legend('Real', 'Meas', 'Obstacle 1', 'Obstacle 2');
elseif TrackNum == 3
    h_legend = legend('Real', 'Meas', 'Obstacle 1', 'Obstacle 2', 'Target 1');
elseif TrackNum == 4
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4');
elseif TrackNum == 5
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5');
elseif TrackNum == 6
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6');
elseif TrackNum == 7
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7');
elseif TrackNum == 8
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8');
elseif TrackNum == 9
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9');
elseif TrackNum == 10
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9', 'Target 10');
elseif TrackNum == 11
    h_legend = legend('Real', 'Meas', 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'Target 6', 'Target 7', 'Target 8', 'Target 9', 'Target 10', 'Target 11');
end
set(h_legend,'FontSize',9, 'Orientation', 'vertical', 'Location', 'southwest');
axis([0 15 0 15]);
