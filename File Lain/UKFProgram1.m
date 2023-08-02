%unscented kalman filter for radar and lidar measurement data fusion

clear all; clc;

dt = 0.1;
N=20; % Number of time steps for filter
% x(:,1) = [0; 0; 50; 50];


%%  Read measurements from a file 
%Data = hasilLidar;
%% Read Measurements from a csv file (or other formats) 
Data = csvread('Radar_Lidar_Data1.csv',1,1);
%Data = csvread('Radar_Lidar_Data2.csv',1,1);
Radar_Measurement = [];
Lidar_Measurement = [];
UKF_Path = [];
if (Data(1,1) == 1)
    x = [Data(1,2); Data(1,3); 0; 0];
else
    x = [Data(1,2); Data(1,3); Data(1,4); 0];
end
% Step 1:  Define UT Scaling parameters and weight vectors
L = 4;          % Size of state vector
alpha = 1;      % Primary scaling parameter 
beta = 2;       % Secondary scaling parameter (Gaussian assumption)
kappa = 0;      % Tertiary scaling parameter 
lambda = alpha^2*(L+kappa) - L;
% wm = ones(2*L + 1,1)*1/(2*(L+lambda)); 
wm=1/(2*(L+lambda));
wc = wm; 
wm(1) = lambda/(lambda+L);
wc(1) = lambda/(lambda+L) + 1 - alpha^2 + beta;
% Step 2:  Define noise assumptions
% Q = diag([0 0 4 4]); 
% R = diag([1 1]);
R = [[0.09, 0, 0];
      [0, 0.005, 0];
      [0, 0, 0.09]];
Q = [(dt^2)/4 0 (dt^3)/2 0;
     0 (dt^2)/4 0 (dt^3)/2;
     (dt^3/2) 0 (dt^2) 0;
     0 (dt^3)/2 0 (dt^2)];
P0 = eye(4,4);
F = [[1, 0, dt, 0];
     [0, 1, 0, dt];
     [0, 0, 1, 0];
     [0, 0, 0, 1]];%matrix transition
 P = P0;     % Set first value of P to the initial P0 
 sP = chol(P,'lower');
chi_p = [x, x*ones(1,L)+sqrt(L+lambda)*sP, ...        
        x*ones(1,L)-sqrt(L+lambda)*sP];
 %% Assuming that the velocity is constant (zero acceleration )
u = 0;
B = [(dt^2)/2 (dt^2)/2 dt dt]';
% H=[1 0 0 0;0 1 0 0];%
  H= [[0, 0, 0, 0];
                     [0,0, 0, 0];
                     [0, 0, 1, 0]];%MEASUREMENT MATRIX
               

for k = 2:N 
for n = 1:length(Data)
    
    if (Data(n,1) == 2)
        chi_m = F*chi_p; % Propagate each sigma-point through prediction
         x_m = chi_m*wm;   % Calculate mean of predicted state    
         P_m = Q;% Calculate covariance of predicted state 
          for i = 1:2*L+1    
        P_m = P_m + wc*(chi_m(:,i) - x_m(:,i))*(chi_m (:,i)- x_m(:,i))';   
          end 
           %measurement update
        Z = Data(n,2:4);
        X = Z(1)*cos(Z(2));
        Y = Z(1)*sin(Z(2));
        VX = Z(3)*cos(Z(2));
        VY = Z(3)*sin(Z(2));
        Z_Car = [X; Y; VX; VY];
        y = Z' - (H * Z_Car);
%    psi_m =[1 0 0 0;0 1 0 0]*chi_m ;
y_m = y*wm; % Calculate mean of predicted output 
% Calculate covariance of predicted output  
      % and cross-covariance between state and output
  Pyy = R;   
      Pxy = zeros(L,1);   
      for i = 1:2*L+1     
          Pyy = Pyy + wc*(y - y_m)*(y- y_m)';     
          Pxy =  wc*(chi_m(:,i) - x_m(:,i))*(y- y_m)';   
      end 
%        K = Pxy/Pyy;% Calculate Kalman gain 
%        x(:,k)=x_m(:,4)+K*y;% Update state estimate
%        P = P_m - K*Pyy*K';% Update covariance estimate
%        UKF_Path = [UKF_Path;[x(1),x(2)]];
%        Radar_Measurement = [Radar_Measurement; Data(n,2:4)];
    else
         chi_m = F*chi_p;
         x_m = chi_m*wm;  
         P_m = Q;
          for i = 1:2*L+1    
        P_m = P_m + wc*(chi_m(:,i) - x_m(:,i))*(chi_m(:,i) - x_m(:,i))';   
          end 
           %measurement update
        Z = Data(n,2:4);
        X = Z(1)*cos(Z(2));
        Y = Z(1)*sin(Z(2));
        VX = Z(3)*cos(Z(2));
        VY = Z(3)*sin(Z(2));
        Z_Car = [X; Y; VX; VY];
        y = transpose(Z) - (H * Z_Car);
    psi_m =[1 0 0 0;0 1 0 0]*chi_m ;
y_m = y*wm; 
  Pyy = R;   
      Pxy = zeros(L,2);   
      for i = 1:2*L+1     
           Pyy = Pyy + wc*(y - y_m)*(y - y_m)';     
          Pxy =  wc*(chi_m(:,i) - x_m(:,i))*(y- y_m)';   
      end 
        K = Pxy/Pyy;
%         x(:,k) =X+K*y;

         x(:,k) =x_m(:,4)+K*y;
        P = P_m - K*Pyy*K';
         UKF_Path = [UKF_Path;[x(1),x(2)]];
        Lidar_Measurement = [Lidar_Measurement; Data(n,2:3)];
    end
end
end
%for i = 1:length(Radar_Measurement)
%    Radar_Measurement_Cart(i,:) = [[Radar_Measurement(i,1),0];[0, Radar_Measurement(i,1)]]*[cos(Radar_Measurement(i,2));sin(Radar_Measurement(i,2))];
%end

hold on;

plot(Data(:,6),Data(:,7),'linewidth', 2);
scatter(UKF_Path(:,1),UKF_Path(:,2),25,'filled','r');
scatter(Lidar_Measurement(:,1),Lidar_Measurement(:,2),5,'filled','blue');
%scatter(Radar_Measurement_Cart(:,1),Radar_Measurement_Cart(:,2),5,'filled','g');

legend('Grundtruth','UKF ','Lidar Measurement','Radar Measurement','Location','northwest');
axis square;
hold off;
axis square;
hold off;
RMSE_Px=rmse(UKF_Path(1,:),Lidar_Measurement(1,:));
RMSE_Py=rmse(UKF_Path(2,:),Radar_Measurement_Cart(2,:))