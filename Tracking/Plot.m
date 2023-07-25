% Inisialisasi data
lidar_data = hasilLidar; % Data LiDAR dengan ukuran 40x4321
pose_data = posesr; % Data pose robot dengan ukuran 3x1508

% Inisialisasi figure dan axis untuk menampilkan animasi
figure;
ax = gca;
hold(ax,'on');
xlim(ax,[-5 5]); % Menentukan batas sumbu x dari plot
ylim(ax,[-5 5]); % Menentukan batas sumbu y dari plot

% Lakukan loop untuk setiap frame animasi
for i = 1:size(pose_data,2)
    % Ambil data LiDAR pada frame i
    lidar_frame = lidar_data(:,(i-1)*10+1:i*10);
    
    % Transformasikan data LiDAR ke sistem koordinat global menggunakan pose robot pada frame i
    transformed_lidar_frame = transform_lidar(lidar_frame, pose_data(:,i));
    
    % Plot data LiDAR pada frame i
    plot(ax, transformed_lidar_frame(1,:), transformed_lidar_frame(2,:), 'k.');
    
    % Plot pose robot pada frame i
    plot(ax, pose_data(1,i), pose_data(2,i), 'ro');
    
    % Tampilkan judul pada plot
    title(ax, ['Frame ' num2str(i)]);
    
    % Tunggu sebentar sebelum menampilkan frame berikutnya
    pause(0.01);
    
    % Hapus plot sebelumnya dari axis
    cla(ax);
end

% Fungsi untuk mentransformasikan data LiDAR ke sistem koordinat global menggunakan pose robot
function transformed_lidar = transform_lidar(lidar_data, pose)
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    rotation_matrix = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    translation_vector = repmat([x; y], 1, size(lidar_data,2));
    
    transformed_lidar = rotation_matrix .* lidar_data + translation_vector;
end