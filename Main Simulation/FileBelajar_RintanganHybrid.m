clc; clear; clf;

tic

%% Difinisi Plant dan Simulasi
jmlObjek = 12;   %menentukan jumlah objek
lingkungan = MultiRobotEnv(jmlObjek);
lingkungan.robotRadius = [0.3 0.3 0.3 0.3 0.3 0.4 0.5 0.3 0.4 0.5 0.5 0.5];
lingkungan.showTrajectory = true;

%robot diff drive
d = 0.6;    %diameter body robot (m)
R = 0.15;   %radius roda (m)
L = 0.15;   %radius roda (m)
dd = MoBotDiffSteer_Generasi2(d,R,L);  %mobile robot diff drive

%objek target 1
d_t = 0.1;    %diameter body robot (m)
R_t = 0.15;   %radius roda (m)
L_t = 0.15;   %radius roda (m)
t = MoBotDiffSteer_Generasi2(d_t,R_t,L_t);  %mobile robot diff drive

%objek target 2
d_t2 = 0.1;    %diameter body robot (m)
R_t2 = 0.15;   %radius roda (m)
L_t2 = 0.15;   %radius roda (m)
t2 = MoBotDiffSteer_Generasi2(d_t2,R_t2,L_t2);  %mobile robot diff drive

%objek rintangan 1
d_o1 = 0.6;    %diameter body robot (m)
R_o1 = 0.15;   %radius roda (m)
L_o1 = 0.15;   %radius roda (m)
o1 = MoBotDiffSteer_Generasi2(d_o1,R_o1,L_o1);  %mobile robot diff drive

%objek rintangan 2
d_o2 = 0.6;    %diameter body robot (m)
R_o2 = 0.15;   %radius roda (m)
L_o2 = 0.15;   %radius roda (m)
o2 = MoBotDiffSteer_Generasi2(d_o2,R_o2,L_o2);  %mobile robot diff drive

%objek rintangan 3
d_o3 = 0.6;    %diameter body robot (m)
R_o3 = 0.15;   %radius roda (m)
L_o3 = 0.15;   %radius roda (m)
o3 = MoBotDiffSteer_Generasi2(d_o3,R_o3,L_o3);  %mobile robot diff drive

%objek rintangan 4
d_o4 = 0.6;    %diameter body robot (m)
R_o4 = 0.15;   %radius roda (m)
L_o4 = 0.15;   %radius roda (m)
o4 = MoBotDiffSteer_Generasi2(d_o4,R_o4,L_o4);  %mobile robot diff drive

%objek rintangan 5
d_o5 = 0.6;    %diameter body robot (m)
R_o5 = 0.15;   %radius roda (m)
L_o5 = 0.15;   %radius roda (m)
o5 = MoBotDiffSteer_Generasi2(d_o5,R_o5,L_o5);  %mobile robot diff drive

%objek rintangan 6
d_o6 = 0.6;    %diameter body robot (m)
R_o6 = 0.15;   %radius roda (m)
L_o6 = 0.15;   %radius roda (m)
o6 = MoBotDiffSteer_Generasi2(d_o6,R_o6,L_o6);  %mobile robot diff drive

%objek rintangan 7
d_o7 = 0.6;    %diameter body robot (m)
R_o7 = 0.15;   %radius roda (m)
L_o7 = 0.15;   %radius roda (m)
o7 = MoBotDiffSteer_Generasi2(d_o7,R_o7,L_o7);  %mobile robot diff drive

%objek rintangan 6
d_o8 = 0.6;    %diameter body robot (m)
R_o8 = 0.15;   %radius roda (m)
L_o8 = 0.15;   %radius roda (m)
o8 = MoBotDiffSteer_Generasi2(d_o8,R_o8,L_o8);  %mobile robot diff drive

%objek rintangan 7
d_o9 = 0.6;    %diameter body robot (m)
R_o9 = 0.15;   %radius roda (m)
L_o9 = 0.15;   %radius roda (m)
o9 = MoBotDiffSteer_Generasi2(d_o9,R_o9,L_o9);  %mobile robot diff drive

load robotSimulatorExampleMaps.mat
%peta = robotics.BinaryOccupancyGrid(borderMap, 3.35);
%peta2 = robotics.BinaryOccupancyGrid(borderMap, 3.35);
peta = binaryOccupancyMap(borderMap, 3.35);
peta2 = binaryOccupancyMap(borderMap, 3.35);
petaDgnJarakAman = copy(peta);
%inflate(petaDgnJarakAman,lingkungan.robotRadius(1));
%lingkungan.mapName = 'peta';
lingkungan.mapName = 'peta2';

WaktuSample = 0.1;
tVec = 0:WaktuSample:50;

%% Menetapkan Pose Robot dan Objek 
poses(:,1) = [3; 3; pi/2];
jenisAgen = "mobileRobot";
poses(:,2) = [13.5; 13.5; pi/2];
jenisAgen = [jenisAgen, "target"];
poses(:,3) = [13.5; 3; pi/2];
jenisAgen = [jenisAgen, "target"];
poses(:,4) = [5.5; 4; -pi];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,5) = [6; 10; -pi/2];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,6) = [9.5; 8.5; pi];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,7) = [12; 10; -pi];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,8) = [8; 2; 0];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,9) = [10; 12; -pi/2];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,10) = [7; 11; -pi];
jenisAgen = [jenisAgen, "nonLinearObjects"];
poses(:,11) = [2; 13.5; 0];
jenisAgen = [jenisAgen, "target"];
poses(:,12) = [7; 7; 0];
jenisAgen = [jenisAgen, "objectsStatis"];

posesTarget1 = poses(:,2);
posesTarget2 = poses(:,3);
posesTarget3 = poses(:,11);

% Define the vertices of the square
x_square = [8, 8.5, 8.5, 8];
y_square = [11, 11, 11.5, 11.5];
x_square1 = [6, 6.5, 6.5, 6];
y_square1 = [3, 3, 3.5, 3.5];
xx = [poses(1,3); poses(1,4); poses(1,5); poses(1,6); poses(1,7); poses(1,8); poses(1,9); poses(1,10); poses(1,11); poses(1,12)];
yy = [poses(2,3); poses(2,4); poses(2,5); poses(2,6); poses(2,7); poses(2,8); poses(2,9); poses(2,10); poses(2,11); poses(2,12)];

% Set the occupancy of the square
setOccupancy(peta, [x_square' y_square'], ones(length(x_square), 1), 'world');
setOccupancy(peta, [x_square1' y_square1'], ones(length(x_square1), 1), 'world');
setOccupancy(peta, [xx yy], ones(1,1), 'world')
inflate(peta, 0.59)

% Set the occupancy of the square 
% setOccupancy(peta2, [x_square' y_square'], ones(length(x_square), 1), 'world');
% setOccupancy(peta2, [x_square1' y_square1'], ones(length(x_square1), 1), 'world');
% inflate(peta2, 0.1)

%% Menentukan Rencana Lintasan 
[a,b] = Belajar_CloverLeaf(0.01, 40, 3, poses(1,4), poses(2,4));
figPerencana = gcf;
set(figPerencana,'Name',sprintf('Path_Planning'));
pos1 = get(figPerencana,'Position'); % get position of Figure(1) 
set(gcf,'Position', pos1 - [400,200,410,-200]); % Shift position of Figure(1), ini aslinya sdh pas utk rekaman
graAxes2 = gca;
judulAxes2 = graAxes2.Title;
set(judulAxes2,'String',sprintf('Obstacle#1 CLOVERLEAF'));
set(graAxes2,'Position',[0.13   0.11   0.3   0.3]);
titikLintasan2 = cell(length(a), 2);
titikLintasan2 = [a(1)   b(1);];
xlim([0 15]);
ylim([0 15]);
for titik = 2:length(a)
    titikLintasan2 = [titikLintasan2; a(titik)   b(titik);];
end

pathPlanning1 = robotics.PRM(peta);
pathPlanning1.NumNodes = 100;
pathPlanning1.ConnectionDistance = 5;
titikAwal1 = poses(1:2,1)';
titikAkhir1 = posesTarget1(1:2)';
titikLintasan1 = findpath(pathPlanning1, titikAwal1, titikAkhir1);
graAxes1 = axes('Position',[0.13   0.5   0.3   0.3]);
show(pathPlanning1);
judulAxes1 = graAxes1.Title;
set(judulAxes1,'String',sprintf('Ego-Robot'));

pathPlanning2 = robotics.PRM(petaDgnJarakAman);
pathPlanning2.NumNodes = 100;
pathPlanning2.ConnectionDistance = 5;
titikAwal2 = poses(1:2,8)';
titikAkhir2 = posesTarget2(1:2)';
titikLintasan6 = findpath(pathPlanning2, titikAwal2, titikAkhir2);

pathPlanning3 = robotics.PRM(petaDgnJarakAman);
pathPlanning3.NumNodes = 100;
pathPlanning3.ConnectionDistance = 5;
titikAwal3 = poses(1:2,9)';
titikAkhir3 = posesTarget3(1:2)';
titikLintasan7 = findpath(pathPlanning3, titikAwal3, titikAkhir3);

pathPlanning4 = robotics.PRM(petaDgnJarakAman);
pathPlanning4.NumNodes = 100;
pathPlanning4.ConnectionDistance = 5;
titikAwal4 = poses(1:2,10)';
titikAkhir4 = posesTarget3(1:2)';
titikLintasan8 = findpath(pathPlanning4, titikAwal4, titikAkhir4);

graAxes3 = axes('Position',[0.5   0.11   0.3   0.3]);
[c,d] = Belajar_CloverLeaf(0.01, 40, 3, poses(1,5), poses(2,5));
judulAxes3 = graAxes3.Title;
set(judulAxes3,'String',sprintf('Obstacle#2 CLOVERLEAF'));
titikLintasan3 = cell(length(c), 2);
titikLintasan3 = [c(1)   d(1);];
xlim([0 15]);
ylim([0 15]);
for titik = 2:length(c)
    titikLintasan3 = [titikLintasan3; c(titik)   d(titik);];
end

graAxes3 = axes('Position',[0.5   0.5   0.3   0.3]);
[e,f] = Belajar_Circle(0.01, 40, 2, poses(1,6), poses(2,6));
judulAxes4 = graAxes3.Title;
set(judulAxes4,'String',sprintf('Obstacle#3 CIRCLE'));
titikLintasan4 = cell(length(e), 2);
titikLintasan4 = [e(1)   f(1);];
xlim([0 15]);
ylim([0 15]);
for titik = 2:length(e)
   titikLintasan4 = [titikLintasan4; e(titik)   f(titik);];
end

graAxes3 = axes('Position',[0.5   0.5   0.3   0.3]);
[g,h] = Belajar_Circle(0.01, 40, 2, poses(1,7), poses(2,7));
judulAxes5 = graAxes3.Title;
set(judulAxes5,'String',sprintf('Obstacle#4 CIRCLE'));
titikLintasan5 = cell(length(g), 2);
titikLintasan5 = [g(1)   h(1);];
xlim([0 15]);
ylim([0 15]);
for titik = 2:length(g)
   titikLintasan5 = [titikLintasan5; g(titik)   h(titik);];
end

%% Kontroller Menuju target tertentu
%kontroller robot diff drive 
kontroler = robotics.PurePursuit;
kontroler.Waypoints = titikLintasan1;
kontroler.LookaheadDistance = 5; 
kontroler.DesiredLinearVelocity = 1; % Kalo bisa diperlambat
kontroler.MaxAngularVelocity = 0.5;

%kontroler objek 1
kontroler1 = robotics.PurePursuit;
kontroler1.Waypoints = titikLintasan2;
kontroler1.LookaheadDistance = 0.5; 
kontroler1.DesiredLinearVelocity = 0.5; % Harus dinaikin
kontroler1.MaxAngularVelocity = 2.5;

%kontroler objek 2
kontroler2 = robotics.PurePursuit;
kontroler2.Waypoints = titikLintasan3;
kontroler2.LookaheadDistance = 0.5; 
kontroler2.DesiredLinearVelocity = 0.5; % Harus dinaikin
kontroler2.MaxAngularVelocity = 2.5;

%kontroler objek 3
kontroler3 = robotics.PurePursuit;
kontroler3.Waypoints = titikLintasan4;
kontroler3.LookaheadDistance = 0.5; 
kontroler3.DesiredLinearVelocity = 1; % Harus dinaikin
kontroler3.MaxAngularVelocity = 2.5;

%kontroler objek 4
kontroler4 = robotics.PurePursuit;
kontroler4.Waypoints = titikLintasan5;
kontroler4.LookaheadDistance = 0.5; 
kontroler4.DesiredLinearVelocity = 0.5;
kontroler4.MaxAngularVelocity = 2.5;

%kontroler objek 5
kontroler5 = robotics.PurePursuit;
kontroler5.Waypoints = titikLintasan6;
kontroler5.LookaheadDistance = 0.5; 
kontroler5.DesiredLinearVelocity = 0.5; % Harus dinaikin
kontroler5.MaxAngularVelocity = 2.5;

%kontroler objek 6
kontroler6 = robotics.PurePursuit;
kontroler6.Waypoints = titikLintasan7;
kontroler6.LookaheadDistance = 0.5; 
kontroler6.DesiredLinearVelocity = 0.5; % Harus dinaikin
kontroler6.MaxAngularVelocity = 2.5;

%kontroler objek 7
kontroler7 = robotics.PurePursuit;
kontroler7.Waypoints = titikLintasan8;
kontroler7.LookaheadDistance = 0.5; 
kontroler7.DesiredLinearVelocity = 0.5;
kontroler7.MaxAngularVelocity = 2.5;

%% Masukan Lidar 
lidars = cell(1, jmlObjek);
for robotIdx = 1:jmlObjek    
        lidars{robotIdx} = LidarSensor;
        lidars{robotIdx}.sensorOffset = [0,0];
        if robotIdx == 1 
            lidars{robotIdx}.scanAngles = linspace(-pi, pi,200);
            %lidars{robotIdx}.scanAngles = linspace(-pi/2, pi/2,19);
        elseif robotIdx == 2 || robotIdx == 3 || robotIdx == 4 || robotIdx == 5 || robotIdx == 6 || robotIdx == 7 || robotIdx == 8 || robotIdx == 9 || robotIdx == 10 || robotIdx == 11 || robotIdx == 12
            lidars{robotIdx}.scanAngles = linspace(-pi, pi,20);
        end
        lidars{robotIdx}.maxRange = 20;    
    if robotIdx == 1 
        attachLidarSensor(lingkungan,robotIdx,lidars{robotIdx});
    end
end

%% Simulasi Loop
algovo = Belajar_HVO3_Majemuk;

kecepatan = zeros(1, jmlObjek);
posesAgen1Sebelumnya = poses(:,1);
posesAgen3Sebelumnya = poses(:,4);
posesAgen4Sebelumnya = poses(:,5);
posesAgen5Sebelumnya = poses(:,6);
posesAgen6Sebelumnya = poses(:,7);
posesAgen7Sebelumnya = poses(:,8);
posesAgen8Sebelumnya = poses(:,9);
posesAgen9Sebelumnya = poses(:,10);
posesAgen10Sebelumnya = poses(:,12);

% kode mapping utk figure performa
dataPosisiXEgoRobot = poses(1,1);
dataPosisiYEgoRobot = poses(2,1);
dataOrientasiEgoRobot = poses(3,1);

% kode utk simpan data Target
dataPosisiXTarget1 = poses(1,2);
dataPosisiYTarget1 = poses(2,2);
dataOrientasiTarget1 = poses(3,2);

% kode utk simpan data Target
dataPosisiXTarget2 = poses(1,3);
dataPosisiYTarget2 = poses(2,3);
dataOrientasiTarget2 = poses(3,3);

% kode utk simpan data Target
dataPosisiXTarget3 = poses(1,11);
dataPosisiYTarget3 = poses(2,11);
dataOrientasiTarget3 = poses(3,11);

% kode utk simpan data obstacle
dataPosisiXObstacle1 = poses(1,4);
dataPosisiYObstacle1 = poses(2,4);
dataOrientasiObstacle1 = poses(3,4);

% kode utk simpan data obstacle
dataPosisiXObstacle2 = poses(1,5);
dataPosisiYObstacle2 = poses(2,5);
dataOrientasiObstacle2 = poses(3,5);

% kode utk simpan data obstacle
dataPosisiXObstacle3 = poses(1,6);
dataPosisiYObstacle3 = poses(2,6);
dataOrientasiObstacle3 = poses(3,6);

% kode utk simpan data obstacle
dataPosisiXObstacle4 = poses(1,7);
dataPosisiYObstacle4 = poses(2,7);
dataOrientasiObstacle4 = poses(3,7);

% kode utk simpan data obstacle
dataPosisiXObstacle5 = poses(1,8);
dataPosisiYObstacle5 = poses(2,8);
dataOrientasiObstacle5 = poses(3,8);

% kode utk simpan data obstacle
dataPosisiXObstacle6 = poses(1,9);
dataPosisiYObstacle6 = poses(2,9);
dataOrientasiObstacle6 = poses(3,9);

% kode utk simpan data obstacle
dataPosisiXObstacle7 = poses(1,10);
dataPosisiYObstacle7 = poses(2,10);
dataOrientasiObstacle7 = poses(3,10);

% kode utk simpan data obstacle
dataPosisiXObstacle8 = poses(1,12);
dataPosisiYObstacle8 = poses(2,12);
dataOrientasiObstacle8 = poses(3,12);

%% Data Awal Simulasi
ranges = cell(1, jmlObjek);%utk skenario 1 ego dgn 1 obst
angles = cell(1, jmlObjek);%utk skenario 1 ego dgn 1 obst
for robotIdx = 1:jmlObjek
      if robotIdx ~= 2 %utk skenario 1 ego, 1 obs, 1 target
        ranges{robotIdx} = lidars{robotIdx}(poses(:,robotIdx));
        angles{robotIdx} = lidars{robotIdx}.scanAngles;
      end
end

poses = [poses(:, 1), poses(:, 2), poses(:, 3), poses(:, 4), poses(:, 5), poses(:, 6), poses(:,7), poses(:,8), poses(:,9), poses(:,10), poses(:,11), poses(:,12)]; %ini dengan objek target
lingkungan([],[],poses,{},[]);

figSimulasi = gcf;
pos2 = get(figSimulasi,'Position');  % get position of Figure(2) 
set(figSimulasi,'Position', pos2 - [250,200,-520,-200]) % Shift position of Figure(2), ini aslinya sdh pas utk rekaman
set(figSimulasi,'Name',sprintf('Collision Avoidance Demo'));
axesSimulasi = gca;
judulSimulasi = axesSimulasi.Title;
set(judulSimulasi,'String',sprintf('Scenario 1\nAvoiding Multi Modal Moving Obstacle with Loop Motion'));
set(judulSimulasi,'HorizontalAlignment','left');

figPerforma = figure('Name', 'Algorithm Performance');
figSimulasi = gcf;
pos3 = get(figSimulasi,'Position');  % get position of Figure(2) 
set(figPerforma, 'Position', pos1 - [-820,200,420,-200]);
axesPerforma = axes('Position', [0.1 0.1 0.15 0.2]);%ego-robot, linear vel
axesPerforma2 = axes('Position', [0.1 0.4 0.15 0.2]);%ego-robot, angul vel
axesPerforma3 = axes('Position', [0.3 0.1 0.15 0.2]);%ego Vref
axesPerforma4 = axes('Position', [0.3 0.4 0.15 0.2]);%ego WRef
axesPerforma5 = axes('Position', [0.5 0.1 0.15 0.2]);%ego WL
axesPerforma6 = axes('Position', [0.5 0.4 0.15 0.2]);%ego WR
axesPerforma7 = axes('Position', [0.1 0.7 0.15 0.2]);%jarak ego ke objek lain
axesPerforma8 = axes('Position', [0.3 0.7 0.15 0.2]);%ego posisi vs refer
axesPerforma9 = axes('Position', [0.5 0.7 0.15 0.2]);%ego orientasi vs ref
axesPerforma10 = axes('Position', [0.7 0.7 0.15 0.2]);%ego bentur vs time
axesPerforma11 = axes('Position', [0.7 0.4 0.15 0.2]);%jarak aman/nyaman vs aktual obst#1 manusia tunggal
axesPerforma12 = axes('Position', [0.7 0.1 0.15 0.2]);%jarak aman/nyaman vs aktual obst#2 manusia grup

%% Algoritma Menjalakan Robot
%tambah besar angka Rate akan menambah cepat simulasi dalam setiap stepnya
rate = robotics.Rate(50);
indek = 1;

radiusTarget = 1.2; %ikut usulan proposal kualifikasi berdasar riset Edward T Hall Chapter X Distance in Man

jarakKeTarget1 = norm(titikAwal1 - titikAkhir1);
jarakKeTarget2 = norm(titikAwal2 - titikAkhir2);
jarakKeTarget3 = norm(titikAwal3 - titikAkhir3);
jarakKeObstacle1 = norm(poses(1:2,1) - poses(1:2,4));
jarakKeObstacle2 = norm(poses(1:2,1) - poses(1:2,5));
jarakKeObstacle3 = norm(poses(1:2,1) - poses(1:2,6));
jarakKeObstacle4 = norm(poses(1:2,1) - poses(1:2,7));
jarakKeObstacle5 = norm(poses(1:2,1) - poses(1:2,8));
jarakKeObstacle6 = norm(poses(1:2,1) - poses(1:2,9));
jarakKeObstacle7 = norm(poses(1:2,1) - poses(1:2,10));
jarakKeObstacle8 = norm(poses(1:2,1) - poses(1:2,12));

%% kode ditambahkan utk kelengkapan performa
ambangJarakAman = 0.35;
dataJarakAman = ambangJarakAman/0.5;
dataJarakNyaman = ambangJarakAman/1.2;
dataJarakKeTarget1DgnAmbang = ambangJarakAman/jarakKeTarget1;
dataJarakKeTarget2DgnAmbang = ambangJarakAman/jarakKeTarget2;
dataJarakKeTarget3DgnAmbang = ambangJarakAman/jarakKeTarget3;
dataJarakKeObstacle1DgnAmbang = ambangJarakAman/jarakKeObstacle1;
dataJarakKeObstacle2DgnAmbang = ambangJarakAman/jarakKeObstacle2;
dataJarakKeObstacle3DgnAmbang = ambangJarakAman/jarakKeObstacle3;
dataJarakKeObstacle4DgnAmbang = ambangJarakAman/jarakKeObstacle4;
dataJarakKeObstacle5DgnAmbang = ambangJarakAman/jarakKeObstacle5;
dataJarakKeObstacle6DgnAmbang = ambangJarakAman/jarakKeObstacle6;
dataJarakKeObstacle7DgnAmbang = ambangJarakAman/jarakKeObstacle7;
dataJarakKeObstacle8DgnAmbang = ambangJarakAman/jarakKeObstacle8;

dataJarakKeTarget1 = jarakKeTarget1;
dataJarakKeTarget2 = jarakKeTarget2;
dataJarakKeTarget3 = jarakKeTarget3;
dataJarakKeObstacle1 = jarakKeObstacle1;
dataJarakKeObstacle2 = jarakKeObstacle2;
dataJarakKeObstacle3 = jarakKeObstacle3;
dataJarakKeObstacle4 = jarakKeObstacle4;
dataJarakKeObstacle5 = jarakKeObstacle5;
dataJarakKeObstacle6 = jarakKeObstacle6;
dataJarakKeObstacle7 = jarakKeObstacle7;
dataJarakKeObstacle8 = jarakKeObstacle8;

%Mengatasi ERROR Undefined function or variable
[vRef, wRef] = kontroler1(posesAgen1Sebelumnya);
[wR, wL] = dd.velocityKeWheelSpeed(vRef, wRef);
[v, w] = dd.wheelSpeedKeVelocity(wR, wL);
kecepatan_frameBody1 = [v; 0; w];

%kode utk mapping Skenario5 bagian kecepatan ke Skenario2
velos(:, 1) =  [v; 0; w]; %ego
velos(:, 2) =  [0; 0; 0]; %target#1

dataKecVRefEgoRob = vRef;
dataKecWRefEgoRob = wRef;
dataKecWLEgoRob = wL;
dataKecWREgoRob = wR;
dataKecLinearEgoRob = v;
dataKecAngularEgoRob = w;
dataKecVRefObsEgoRob = vRef;
dataKecWRefObsEgoRob = wRef;

%Mengatasi ERROR Undefined function or variable
[vRef6, wRef6] = kontroler5(posesAgen7Sebelumnya);
[wR6, wL6] = o5.velocityKeWheelSpeed(vRef6, wRef6);
[v6, w6] = o5.wheelSpeedKeVelocity(wR6, wL6);
kecepatan_frameBody7 = [v6; 0; w6];

%kode utk mapping Skenario5 bagian kecepatan ke Skenario2
velos(:, 8) =  [v6; 0; w6]; %ego
velos(:, 3) =  [0; 0; 0]; %target#2

%Mengatasi ERROR Undefined function or variable
[vRef7, wRef7] = kontroler6(posesAgen8Sebelumnya);
[wR7, wL7] = o6.velocityKeWheelSpeed(vRef7, wRef7);
[v7, w7] = o6.wheelSpeedKeVelocity(wR7, wL7);
kecepatan_frameBody8 = [v7; 0; w7];

%kode utk mapping Skenario5 bagian kecepatan ke Skenario2
velos(:, 9) =  [v; 0; w]; %ego
velos(:, 11) =  [0; 0; 0]; %target#2

%Mengatasi ERROR Undefined function or variable
[vRef8, wRef8] = kontroler7(posesAgen9Sebelumnya);
[wR8, wL8] = o7.velocityKeWheelSpeed(vRef8, wRef8);
[v8, w8] = o7.wheelSpeedKeVelocity(wR8, wL8);
kecepatan_frameBody9 = [v8; 0; w8];

%kode utk mapping Skenario5 bagian kecepatan ke Skenario2
velos(:, 10) =  [v; 0; w]; %ego
velos(:, 11) =  [0; 0; 0]; %target#2

dataHeadingVector =[0; 0; 0];

k = 1;
waktu = k;

jmlBenturan = 0;
dataBenturan = NaN;

loopPP = 0;
loopVO = 0;

[vRef2, wRef2] = kontroler1(posesAgen3Sebelumnya);
[wR2, wL2] = o1.velocityKeWheelSpeed(vRef2, wRef2);
[v2, w2] = o1.wheelSpeedKeVelocity(wR2, wL2);
kecepatan_frameBody3 = [v2; 0; w2];

velos(:, 4) = [v2; 0; w2]; %obst#1

[vRef3, wRef3] = kontroler2(posesAgen4Sebelumnya);
[wR3, wL3] = o2.velocityKeWheelSpeed(vRef3, wRef3);
[v3, w3] = o2.wheelSpeedKeVelocity(wR3, wL3);
%kecepatan_frameBody4 = [v3; 0; w3];

velos(:, 5) =  [v3; 0; w3];%obst#2

[vRef4, wRef4] = kontroler3(posesAgen5Sebelumnya);
[wR4, wL4] = o3.velocityKeWheelSpeed(vRef4, wRef4);
[v4, w4] = o3.wheelSpeedKeVelocity(wR4, wL4);
kecepatan_frameBody5 = [v4; 0; w4];

velos(:, 6) =  [v4; 0; w4];%obst#3

[vRef5, wRef5] = kontroler4(posesAgen6Sebelumnya);
[wR5, wL5] = o4.velocityKeWheelSpeed(vRef5, wRef5);
[v5, w5] = o4.wheelSpeedKeVelocity(wR5, wL5);
kecepatan_frameBody6 = [v5; 0; w5];

velos(:, 7) =  [v5; 0; w5];%obst#4

jarakAktif = 2;

i = 1;
while(jarakKeTarget1 > radiusTarget)
   for robotIdx = 1:jmlObjek
       if robotIdx ~= 2 %ego=1, target=2, obst=3
        ranges{robotIdx} = lidars{robotIdx} (poses(:,robotIdx));
        angles{robotIdx} = lidars{robotIdx}.scanAngles;
       end
   end
  
   Ranges = [ranges{1}]; %Record data lidar range dan angles selama waktu simulasi
   Angles = [angles{1}]'; 
   Count = 50;
   
   % menggabungkan keempat data tersebut menjadi satu struct MATLAB
   scan.Ranges = Ranges;
   scan.Angles = Angles;
   scans{i} = lidarScan(Ranges, Angles);
   i = i+1;
   
   lingkungan(1:jmlObjek, poses, ranges, []);
   xlim([0 15]);
   ylim([0 15]);
   
   %update kecepatan dari semua agen selain agen no 2
   for pedestrianIndek = 1:jmlObjek
       if pedestrianIndek == 1            
            %% cek jarak jika aman gerakkan agen 1 menuju target
            jarak = ranges{1}';
            jmlDataRangesValid = size(find(ranges{1} < 2)); %% jika jarak            
            statusKontroler = '';

            if (~isnan(ranges{1}(1)) || ~isnan(ranges{1}(2)) || ~isnan(ranges{1}(3)) ...
             || ~isnan(ranges{1}(4)) || ~isnan(ranges{1}(5)) || ~isnan(ranges{1}(6)) ...
             || ~isnan(ranges{1}(7)) || ~isnan(ranges{1}(8)) || ~isnan(ranges{1}(9))) && ...
             (ranges{1}(1) < jarakAktif || ranges{1}(2) < jarakAktif ...
             || ranges{1}(3) < jarakAktif || ranges{1}(4) < jarakAktif ...
             || ranges{1}(5) < jarakAktif || ranges{1}(6) < jarakAktif ...
             || ranges{1}(7) < jarakAktif || ranges{1}(8) < jarakAktif ...
             || ranges{1}(9) < jarakAktif)
             
                namaKontroler = algovo.dapatkanNamaMetode();
                statusKontroler = sprintf('%s%d', namaKontroler, loopVO);
                loopVO = loopVO + 1;

                algovo.menghapusDataIDRadius();
                algovo.memperbaruiModalitas(jenisAgen);
                algovo.memperbaruiDataLIDAR(ranges{1}, angles{1}');
                for indekAgen = 1:jmlObjek
                    algovo.memperbaruiDataRadiusPoseVeloAgen( ...
                            indekAgen, ...
                            lingkungan.robotRadius(indekAgen), ...
                            poses(1,indekAgen), poses(2,indekAgen), poses(3,indekAgen), ...
                            velos(1,indekAgen), velos(2,indekAgen), velos(3,indekAgen) ...
                          );
                end
                
                [vRefObs,wRefObs] = algovo.menghitungKecepatanBebasBenturan(algovo.v_i); 
                dataKecVRefObsEgoRob = vRefObs;
                dataKecWRefObsEgoRob = wRefObs;
                                
                [wRobs, wLobs] = dd.velocityKeWheelSpeed(vRefObs, wRefObs);
                [v, w] = dd.wheelSpeedKeVelocity(wRobs, wLobs);
          else 
            statusKontroler = sprintf('PP%d', loopPP);
            loopPP = loopPP + 1;
                [vRef, wRef] = kontroler(posesAgen1Sebelumnya);
                [wR, wL] = dd.velocityKeWheelSpeed(vRef, wRef);
                [v, w] = dd.wheelSpeedKeVelocity(wR, wL);
               
                dataKecVRefObsEgoRob = vRef;
                dataKecWRefObsEgoRob = wRef;
            end
            kecepatan_lokal_robot = [v;0;w]; % kecepatan body mobile robot [vx;vy;w]
            statusBenturan = '';
            if jarakKeObstacle1 < 0.5 || jarakKeObstacle2 < 0.5 || jarakKeObstacle3 < 0.5 || jarakKeObstacle4 < 0.5 || jarakKeObstacle5 < 0.5 || jarakKeObstacle6 < 0.5 || jarakKeObstacle7 < 0.5 || jarakKeObstacle8 < 0.5
                dataBenturan = [dataBenturan    jarakKeTarget1];
                jmlBenturan = jmlBenturan + 1;                
                statusBenturan = 'Collide';                
            else
                dataBenturan = [dataBenturan    NaN];                
                statusBenturan = 'Safe';                
            end            
            
            dataKecVRefEgoRob = [dataKecVRefEgoRob  vRef];
            dataKecWRefEgoRob = [dataKecWRefEgoRob  wRef];
            dataKecWLEgoRob = [dataKecWLEgoRob  wL];
            dataKecWREgoRob = [dataKecWREgoRob  wR];
            dataKecLinearEgoRob = [dataKecLinearEgoRob   v];
            dataKecAngularEgoRob = [dataKecAngularEgoRob   w];
            
            dataJarakAman = [dataJarakAman  ambangJarakAman/0.5];
            dataJarakNyaman = [dataJarakNyaman  ambangJarakAman/1.2];            
            dataJarakKeObstacle1DgnAmbang = [dataJarakKeObstacle1DgnAmbang  ambangJarakAman/jarakKeObstacle1];
            dataJarakKeObstacle2DgnAmbang = [dataJarakKeObstacle2DgnAmbang  ambangJarakAman/jarakKeObstacle2];
            dataJarakKeObstacle3DgnAmbang = [dataJarakKeObstacle3DgnAmbang  ambangJarakAman/jarakKeObstacle3];
            dataJarakKeObstacle4DgnAmbang = [dataJarakKeObstacle4DgnAmbang  ambangJarakAman/jarakKeObstacle4];
            dataJarakKeObstacle5DgnAmbang = [dataJarakKeObstacle5DgnAmbang  ambangJarakAman/jarakKeObstacle5];
            dataJarakKeObstacle6DgnAmbang = [dataJarakKeObstacle6DgnAmbang  ambangJarakAman/jarakKeObstacle6];
            dataJarakKeObstacle7DgnAmbang = [dataJarakKeObstacle7DgnAmbang  ambangJarakAman/jarakKeObstacle7];
            dataJarakKeObstacle8DgnAmbang = [dataJarakKeObstacle8DgnAmbang  ambangJarakAman/jarakKeObstacle8];
            dataJarakKeTarget1DgnAmbang = [dataJarakKeTarget1DgnAmbang  ambangJarakAman/jarakKeTarget1];
            dataJarakKeTarget2DgnAmbang = [dataJarakKeTarget2DgnAmbang  ambangJarakAman/jarakKeTarget2];
            dataJarakKeTarget3DgnAmbang = [dataJarakKeTarget3DgnAmbang  ambangJarakAman/jarakKeTarget3];
            
            dataJarakKeTarget1 = [dataJarakKeTarget1  jarakKeTarget1];
            dataJarakKeTarget2 = [dataJarakKeTarget2  jarakKeTarget2];
            dataJarakKeTarget3 = [dataJarakKeTarget3  jarakKeTarget3];
            dataJarakKeObstacle1 = [dataJarakKeObstacle1    jarakKeObstacle1];
            dataJarakKeObstacle2 = [dataJarakKeObstacle2    jarakKeObstacle2];
            dataJarakKeObstacle3 = [dataJarakKeObstacle3    jarakKeObstacle3];
            dataJarakKeObstacle4 = [dataJarakKeObstacle4    jarakKeObstacle4];
            dataJarakKeObstacle5 = [dataJarakKeObstacle5    jarakKeObstacle5];
            dataJarakKeObstacle6 = [dataJarakKeObstacle6    jarakKeObstacle6];
            dataJarakKeObstacle7 = [dataJarakKeObstacle7    jarakKeObstacle7];
            dataJarakKeObstacle8 = [dataJarakKeObstacle8    jarakKeObstacle8];
            
            dataPosisiXEgoRobot = [dataPosisiXEgoRobot    poses(1,1)];
            dataPosisiYEgoRobot = [dataPosisiYEgoRobot    poses(2,1)];
            dataOrientasiEgoRobot = [dataOrientasiEgoRobot  poses(3,1)];
            
            dataPosisiXObstacle1 = [dataPosisiXObstacle1    poses(1,4)];
            dataPosisiYObstacle1 = [dataPosisiYObstacle1    poses(2,4)];
            dataOrientasiObstacle1 = [dataOrientasiObstacle1    poses(3,4)];
            
            dataPosisiXObstacle2 = [dataPosisiXObstacle2    poses(1,5)];
            dataPosisiYObstacle2 = [dataPosisiYObstacle2    poses(2,5)];
            dataOrientasiObstacle2 = [dataOrientasiObstacle2    poses(3,5)];
            
            dataPosisiXObstacle3 = [dataPosisiXObstacle3    poses(1,6)];
            dataPosisiYObstacle3 = [dataPosisiYObstacle3    poses(2,6)];
            dataOrientasiObstacle3 = [dataOrientasiObstacle3    poses(3,6)];
            
            dataPosisiXObstacle4 = [dataPosisiXObstacle4    poses(1,7)];
            dataPosisiYObstacle4 = [dataPosisiYObstacle4    poses(2,7)];
            dataOrientasiObstacle4 = [dataOrientasiObstacle4    poses(3,7)];

            dataPosisiXObstacle5 = [dataPosisiXObstacle5    poses(1,8)];
            dataPosisiYObstacle5 = [dataPosisiYObstacle5    poses(2,8)];
            dataOrientasiObstacle5 = [dataOrientasiObstacle5    poses(3,8)];
            
            dataPosisiXObstacle6 = [dataPosisiXObstacle6    poses(1,9)];
            dataPosisiYObstacle6 = [dataPosisiYObstacle6    poses(2,9)];
            dataOrientasiObstacle6 = [dataOrientasiObstacle6    poses(3,9)];
            
            dataPosisiXObstacle7 = [dataPosisiXObstacle7    poses(1,10)];
            dataPosisiYObstacle7 = [dataPosisiYObstacle7    poses(2,10)];
            dataOrientasiObstacle7 = [dataOrientasiObstacle7    poses(3,10)];
            
            dataPosisiXObstacle8 = [dataPosisiXObstacle8    poses(1,12)];
            dataPosisiYObstacle8 = [dataPosisiYObstacle8    poses(2,12)];
            dataOrientasiObstacle8 = [dataOrientasiObstacle8    poses(3,12)];
            
            dataPosisiXTarget1 = [dataPosisiXTarget1   poses(1,2)];
            dataPosisiYTarget1 = [dataPosisiYTarget1   poses(2,2)];
            dataOrientasiTarget1 = [dataOrientasiTarget1   poses(3,2)];

            dataPosisiXTarget2 = [dataPosisiXTarget2   poses(1,3)];
            dataPosisiYTarget2 = [dataPosisiYTarget2   poses(2,3)];
            dataOrientasiTarget2 = [dataOrientasiTarget2   poses(3,3)];
            
            dataPosisiXTarget3 = [dataPosisiXTarget3   poses(1,11)];
            dataPosisiYTarget3 = [dataPosisiYTarget3   poses(2,11)];
            dataOrientasiTarget3 = [dataOrientasiTarget3   poses(3,11)];
            
            k = k + 1;
            waktu = [waktu   k];
            str1 = sprintf('EgoBot|k:%s|vd:%0.1f|wd:%0.1f|v:%0.1f|w:%0.1f', ...
                statusKontroler,dataKecVRefObsEgoRob,dataKecWRefObsEgoRob,v,w);            
            str2 = sprintf('Heading:%0.1f|%0.1f|%0.1f|Lidar:%0.1f|%0.1f|%0.1f|%0.1f[%0.1f]%0.1f|%0.1f|%0.1f|%0.1f', ...
                rad2deg(dataHeadingVector(1)),rad2deg(dataHeadingVector(2)),rad2deg(dataHeadingVector(3)),ranges{1}(1),ranges{1}(2),ranges{1}(3),ranges{1}(4), ...
                ranges{1}(5),ranges{1}(6),ranges{1}(7),ranges{1}(8),ranges{1}(9));
            str3 = sprintf('O#1|v:%0.2f,w:%0.2f, O#2|v:%0.2f,w:%0.2f, O#3|v:%0.2f,w:%0.2f, O#4|v:%0.2f,w:%0.2f, O#5|v:%0.2f,w:%0.2f, O#6|v:%0.2f,w:%0.2f, O#7|v:%0.2f,w:%0.2f', ...
                            v2,w2,v3,w3,v4,w4,v5,w5,v6,w6,v7,w7,v8,w8); %manusiaTunggal            
            str4 = sprintf('DistTar#1:%0.2f|DistTar#2:%0.2f|DistTar#3:%0.2f|DistOb#1:%0.2f|DistOb#2:%0.2f|DistOb#3:%0.2f|DistOb#4:%0.2f|DistOb#5:%0.2f|DistOb#6:%0.2f|DistOb#7:%0.2f|DistOb#8:%0.2f', ...
                            jarakKeTarget1, jarakKeTarget2, jarakKeTarget3, jarakKeObstacle1, jarakKeObstacle2, jarakKeObstacle3, jarakKeObstacle4, jarakKeObstacle5, jarakKeObstacle6, jarakKeObstacle7, jarakKeObstacle8);
            str5 = sprintf('Controller1(PP):%d|Controller2(%s):%d', loopPP, algovo.dapatkanNamaMetode(), loopVO);           
            str6 = sprintf('Collision Status: %s | No of Collision: %d', statusBenturan, jmlBenturan);
                        
            % kode mapping ke figure performa
            %legSimulasi = axesSimulasi.Legend;
            %set(legSimulasi, 'String',{str1,str2,str3,str4,str5,str6});

            kecepatan_global = dd.kecepatanLokalKeGlobal(kecepatan_lokal_robot,posesAgen1Sebelumnya);
            poses(:, 1) = posesAgen1Sebelumnya + kecepatan_global*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
            posesAgen1Sebelumnya = poses(:,1);
       elseif pedestrianIndek == 2 %%pose objek Target selalu tetap
            poses(:, 2) = poses(:, 2);
       elseif pedestrianIndek == 3 %%pose objek Target selalu tetap
            poses(:, 3) = poses(:, 3);
       elseif pedestrianIndek == 11 %%pose objek Target selalu tetap
            poses(:, 11) = poses(:, 11);
       elseif pedestrianIndek == 4 %%kontroler gerakan  melingkar dari obstacle           
           %%AWAL persiapan utk menentukan gerak obstacle CLOVER LEAF
            [vRef2, wRef2] = kontroler1(posesAgen3Sebelumnya);
            [wR2, wL2] = o1.velocityKeWheelSpeed(vRef2, wRef2);
            [v2, w2] = o1.wheelSpeedKeVelocity(wR2, wL2);

            kecepatan_lokal_o1 = [v2; 0; w2]; %nonlinear object 1            
            kecepatan_global_o1 = o1.kecepatanLokalKeGlobal(kecepatan_lokal_o1,posesAgen3Sebelumnya);
            poses(:,4) = posesAgen3Sebelumnya + kecepatan_global_o1*WaktuSample; % terapkan integrasi eulerian utk update data pose terkini                        
            posesAgen3Sebelumnya = poses(:,4);
       elseif pedestrianIndek == 5 %%kontroler gerakan  melingkar dari obstacle           
           %%AWAL persiapan utk menentukan gerak obstacle CLOVER LEAF
            [vRef3, wRef3] = kontroler2(posesAgen4Sebelumnya);     
            [wR3, wL3] = o2.velocityKeWheelSpeed(vRef3, wRef3);
            [v3, w3] = o2.wheelSpeedKeVelocity(wR3, wL3);
                        
            kecepatan_lokal_o2 = [v3; 0; w3];%nonlinear object 1            
            kecepatan_global_o2 = o2.kecepatanLokalKeGlobal(kecepatan_lokal_o2,posesAgen4Sebelumnya);
            poses(:, 5) = posesAgen4Sebelumnya + kecepatan_global_o2*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
            posesAgen4Sebelumnya = poses(:,5);
       elseif pedestrianIndek == 6 %%kontroler gerakan  melingkar dari obstacle           
           %AWAL persiapan utk menentukan gerak obstacle LINGKARAN
           [vRef4, wRef4] = kontroler3(posesAgen5Sebelumnya);
            
           [wR4, wL4] = o3.velocityKeWheelSpeed(vRef4, wRef4);
           [v4, w4] = o3.wheelSpeedKeVelocity(wR4, wL4);
            
           kecepatan_lokal_o3 = [v4; 0; w4];%nonlinear object 3
           kecepatan_global_o3 = o3.kecepatanLokalKeGlobal(kecepatan_lokal_o3,posesAgen5Sebelumnya);
           poses(:, 6) = posesAgen5Sebelumnya + kecepatan_global_o3*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
           posesAgen5Sebelumnya = poses(:,6);
       elseif pedestrianIndek == 7 %%kontroler gerakan  melingkar dari obstacle           
           %AWAL persiapan utk menentukan gerak obstacle LINGKARAN
           [vRef5, wRef5] = kontroler4(posesAgen6Sebelumnya);
            
           [wR5, wL5] = o4.velocityKeWheelSpeed(vRef5, wRef5);
           [v5, w5] = o4.wheelSpeedKeVelocity(wR5, wL5);
            
           kecepatan_lokal_o4 = [v5; 0; w5];%nonlinear object 3
           kecepatan_global_o4 = o4.kecepatanLokalKeGlobal(kecepatan_lokal_o4,posesAgen6Sebelumnya);
           poses(:, 7) = posesAgen6Sebelumnya + kecepatan_global_o4*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
           posesAgen6Sebelumnya = poses(:,7);
       elseif pedestrianIndek == 8 %%kontroler gerakan  melingkar dari obstacle           
           %AWAL persiapan utk menentukan gerak obstacle LINGKARAN
           [vRef6, wRef6] = kontroler5(posesAgen7Sebelumnya);
            
           [wR6, wL6] = o5.velocityKeWheelSpeed(vRef6, wRef6);
           [v6, w6] = o5.wheelSpeedKeVelocity(wR6, wL6);
            
           kecepatan_lokal_o5 = [v6; 0; w6];%nonlinear object 3
           kecepatan_global_o5 = o5.kecepatanLokalKeGlobal(kecepatan_lokal_o5,posesAgen7Sebelumnya);
           poses(:, 8) = posesAgen7Sebelumnya + kecepatan_global_o5*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
           posesAgen7Sebelumnya = poses(:,8);
       elseif pedestrianIndek == 9 %%kontroler gerakan  melingkar dari obstacle           
           %AWAL persiapan utk menentukan gerak obstacle LINGKARAN
           [vRef7, wRef7] = kontroler6(posesAgen8Sebelumnya);
            
           [wR7, wL7] = o6.velocityKeWheelSpeed(vRef7, wRef7);
           [v7, w7] = o6.wheelSpeedKeVelocity(wR7, wL7);
            
           kecepatan_lokal_o6 = [v5; 0; w5];%nonlinear object 3
           kecepatan_global_o6 = o5.kecepatanLokalKeGlobal(kecepatan_lokal_o6,posesAgen8Sebelumnya);
           poses(:, 9) = posesAgen8Sebelumnya + kecepatan_global_o6*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
           posesAgen8Sebelumnya = poses(:,9);
       elseif pedestrianIndek == 10 %%kontroler gerakan  melingkar dari obstacle           
           %AWAL persiapan utk menentukan gerak obstacle LINGKARAN
           [vRef8, wRef8] = kontroler7(posesAgen9Sebelumnya);
            
           [wR8, wL8] = o7.velocityKeWheelSpeed(vRef8, wRef8);
           [v8, w8] = o7.wheelSpeedKeVelocity(wR8, wL8);
            
           kecepatan_lokal_o7 = [v8; 0; w8];%nonlinear object 3
           kecepatan_global_o7 = o7.kecepatanLokalKeGlobal(kecepatan_lokal_o7,posesAgen9Sebelumnya);
           poses(:, 10) = posesAgen9Sebelumnya + kecepatan_global_o7*WaktuSample;% terapkan integrasi eulerian utk update data pose terkini                        
           posesAgen9Sebelumnya = poses(:,10);
       elseif pedestrianIndek == 12 %%pose objek Target selalu tetap
            poses(:, 12) = poses(:, 12);
       else %%kontroler gerakan dari obstacle lain, jika ada dlm skenario
            deteksi = step(detektors{pedestrianIndek}); 
            kecepatan(:,pedestrianIndek) = kontrolerKerumunanPedestrian(poses, pedestrianIndek, deteksi, 1);
            poses = poses + kecepatan*WaktuSample;
       end
   end
   
   poses = [poses(:, 1), poses(:, 2), poses(:, 3), poses(:, 4), poses(:, 5), poses(:, 6), poses(:, 7), poses(:, 8), poses(:, 9), poses(:, 10), poses(:, 11), poses(:, 12)]; %ini dgn objek target
   
   %%bagian ini di-uncomment hanya utk memeriksa nilai titik awal dan akhir
   titikAwal1 = poses(1:2, indek);
   titikAwal2 = poses(1:2, indek);
   titikAwal3 = poses(1:2, indek);
   
   % Re-compute the distance to the goal
   jarakKeTarget1 = norm(titikAwal1' - titikAkhir1);
   jarakKeTarget2 = norm(titikAwal2' - titikAkhir2);
   jarakKeTarget3 = norm(titikAwal3' - titikAkhir3);
   jarakKeObstacle1 = norm(poses(1:2,1) - poses(1:2,4));
   jarakKeObstacle2 = norm(poses(1:2,1) - poses(1:2,5));
   jarakKeObstacle3 = norm(poses(1:2,1) - poses(1:2,6));
   jarakKeObstacle4 = norm(poses(1:2,1) - poses(1:2,7));
   jarakKeObstacle5 = norm(poses(1:2,1) - poses(1:2,8));
   jarakKeObstacle6 = norm(poses(1:2,1) - poses(1:2,9));
   jarakKeObstacle7 = norm(poses(1:2,1) - poses(1:2,10));
   jarakKeObstacle8 = norm(poses(1:2,1) - poses(1:2,12));
      
    waitfor(rate);
end

% posesr = [dataPosisiXEgoRobot; dataPosisiYEgoRobot; dataOrientasiEgoRobot]';           % Gabungkan data pose
% PoseXObs = [dataPosisiXObstacle1; dataPosisiXObstacle2; dataPosisiXObstacle3; dataPosisiXObstacle4; dataPosisiXObstacle5; dataPosisiXObstacle6; dataPosisiXObstacle7; dataPosisiXObstacle8; dataPosisiXTarget1; dataPosisiXTarget2; dataPosisiXTarget3]';           % Gabungan pose x obstacle
% PoseYObs = [dataPosisiYObstacle1; dataPosisiYObstacle2; dataPosisiYObstacle3; dataPosisiYObstacle4; dataPosisiYObstacle5; dataPosisiYObstacle6; dataPosisiYObstacle7; dataPosisiYObstacle8; dataPosisiYTarget1; dataPosisiYTarget2; dataPosisiYTarget3]';           % Gabungan pose Y obstacle
% PoseORObs = [dataOrientasiObstacle1; dataOrientasiObstacle2; dataOrientasiObstacle3; dataOrientasiObstacle4; dataOrientasiObstacle5; dataOrientasiObstacle6; dataOrientasiObstacle7; dataOrientasiObstacle8; dataOrientasiTarget1; dataOrientasiTarget2; dataOrientasiTarget3]';    % Gabungan orientasi obstacle
% save("FilePoseRH.mat","PoseXObs","PoseYObs","PoseORObs")
% %lidarPlot = cell2mat(RangesAll);
% save("FileBelajarRH.mat") %save data output lidar
% save("FileLidarRH.mat","scans")

waktuProses = toc

%judulLegenda = legSimulasi.Title;
strMenit = split(sprintf('%0.1f',waktuProses/60),".");
strDetik = split(sprintf('%0.1f',mod(waktuProses,60)),".");
str21 = sprintf('Time to Target: %s minutes %s seconds', strMenit{1}, strDetik{1});
%set(judulLegenda, 'String',str21);

figPerforma.CurrentAxes = axesPerforma;
plot(waktu, dataKecLinearEgoRob);
xlabel('time [s]');
ylabel('v [m/s]');
title('Mobile Robot v');

figPerforma.CurrentAxes = axesPerforma2;
plot(waktu, dataKecAngularEgoRob);
xlabel('time [s]');
ylabel('w [rad/s]');
title('Mobile Robot w');

figPerforma.CurrentAxes = axesPerforma3;
plot(waktu, dataKecVRefEgoRob);%aslinya
xlabel('time [s]');
ylabel('VRef [m/s]');
title('Mobile Robot VRef');

figPerforma.CurrentAxes = axesPerforma4;
plot(waktu, dataKecWRefEgoRob);%aslinya
xlabel('time [s]');
ylabel('WRef [rad/s]');
title('Mobile Robot WRef');

figPerforma.CurrentAxes = axesPerforma5;
plot(waktu, dataKecWLEgoRob);
xlabel('time [s]');
ylabel('WL [rad/s]');
title('Mobile Robot WL');

figPerforma.CurrentAxes = axesPerforma6;
plot(waktu, dataKecWREgoRob);
xlabel('time [s]');
ylabel('WR [rad/s]');
title('Mobile Robot WR');

figPerforma.CurrentAxes = axesPerforma7;
plot(waktu, dataJarakKeTarget1, ...
    waktu, dataJarakKeObstacle1);
xlabel('time [s]');
ylabel('distance[m]');
title('Distance to Target/Obstacle');
legend(axesPerforma7, {'Target', 'Obstacle1'});

figPerforma.CurrentAxes = axesPerforma8;
plot(titikLintasan1(:,1),titikLintasan1(:,2),dataPosisiXEgoRobot, dataPosisiYEgoRobot);
xlim([0 15])
ylim([0 15])
xlabel('time [s]');
ylabel('distance[m]');
title('Mobile Robot Reference vs Actual Trajectory');

figPerforma.CurrentAxes = axesPerforma9;
plot(waktu, dataOrientasiEgoRobot);
xlabel('time [s]');
ylabel('degree[deg]');
title('Mobile Robot Orientation');

figPerforma.CurrentAxes = axesPerforma10;
plot(waktu, dataBenturan, 'rX');
xlabel('time [s]');
ylabel('Distance to target[m]');
title('Collision');

figPerforma.CurrentAxes = axesPerforma11;
plot( ...
    waktu, dataJarakAman, 'r-', ...
    waktu, dataJarakKeObstacle1DgnAmbang, 'b-' ...
);
xlabel('time [s]');
ylabel('Proximity Index w/ Obstacle#1');
title('Safety w/ Obstacle#1');
legend(axesPerforma11, {'Safe', 'Actual'});

figPerforma.CurrentAxes = axesPerforma12;
plot( ...
    waktu, dataJarakAman, 'r-', ...
    waktu, dataJarakKeTarget1DgnAmbang, 'b-' ...
);
xlabel('time [s]');
ylabel('Proximity Index w/ Target');
title('Safety w/ Target');
legend(axesPerforma12, {'Safe', 'Actual'});
