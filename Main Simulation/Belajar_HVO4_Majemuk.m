%% C:\kerjaan\BukuDisertasi\PresentasiSenin3Mei2021\RevisiPakDjokoBuTuti\SimPRinRoMoDaV10\RintanganStatis\kode
%% @Jumat,21Mei2021,15.58PM 

classdef Belajar_HVO4_Majemuk < matlab.System    
    %BELAJAR_VO_MAJEMUK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       % parameter penghindaran
       epsilon = 1E-5;                     % toleransi sudut
       neighbourDist = 15;                 % jarak dengan objects 
       maxNeighbours = 10;                 % jml maximum objects
       kepadatan = 15;                  % kepadatan objects dlm lingkungan
       % parameter VO
%        maxSpeed = 0.5;%%Jumat28Mei2021,10.28AM
       maxSpeed = 2;
%        maxSpeed = 1.5;
       feasabliltyMatrix; % matrix kecepatan yg dpt dicapai robot
        
       poseEgoX; poseEgoY; poseEgoPsi; p_i; % p_i = Pr = [xr; yr; ?r]
       veloEgoVx; veloEgoVy; veloEgoW; v_i; % v_i = Vr = [Vxr; Vyr; ?r]
       r_i; 
       
       poseObsX; poseObsY; poseObsPsi; p_j; % p_j = Po = [xo; yo; ?o]
       veloObsVx; veloObsVy; veloObsW; v_j; % v_j = Vo = [Vxo; Vyo; ?o]
       r_j;
       
       desiredVelocity; % v_i = Vr
              
       id; % struktur data utk menyimpan id, radius, pose dan velocity
       radius;
       poses;
       velos;
       
       himpunanVO; %himpunan kecepatan yg tak boleh dipilih karena sebabkan benturan (collision velocity)
       himpunanFV; %himpunan kecepatan yg dpt dilaksanakan/diwujudkan (fisibel) robot mobil
       himpunanEV; %himpunan kecepatan FV yg bebas benturan
       himpunanAV; %himpunan kecepatan EV yg terdekat dgn desiredVelocity (v_i)
       himpunanHeadingVector; %himpunan sudut haluan robot mobil
       
       jmlAgen;
       alpharo; phiro; dro; rro;
       modalitas;%jenis agen {VO,RVO,NLVO}
       dataLIDAR_jarak; dataLIDAR_sudut;
       indekAgenSaatIni;
       
       %% Selasa8Jun2021,20.09PM
       idAgen;
       haluan; %unrecognized property 'haluan',Selasa8Jun2021,21.00PM
       indekHaluan;
       jarakPadaIndekHaluan;
    end
    
    methods
        %konstruktor
        function obj = Belajar_HVO4_Majemuk(varargin)
%               setProperties(obj,nargin,varargin{:});
            obj.idAgen = varargin{1};
        end
        %menghasilkan output berupa nama metode
        function [namaMetode] = dapatkanNamaMetode(obj,~)
           namaMetode = 'HVO'; 
        end
        %mengenolkan struktur data id dan radius sebelum memperbarui data
        function menghapusDataIDRadius(obj,~)
            obj.id = []; %yg nyebabin salah sebelumnya obj.id = [obj.id varargin{1}];
            obj.radius = [];   %% radius robot 
        end
        %memperbarui data modalitas dari objek bergerak
        function memperbaruiModalitas(obj,jenisAgen)
           obj.modalitas = jenisAgen; 
        end
        %memperbarui data LIDAR terdiri dari jarak dan sudut
        function memperbaruiDataLIDAR(obj, jarak, sudut)
            obj.dataLIDAR_jarak = jarak;
            obj.dataLIDAR_sudut = sudut;
        end %akhir function memperbaruiDataLIDAR(obj, jarak, sudut)
        %menghitung steering dari sudut yg setara dg jarak bacaan LIDAR
        
        %menghitung steering dari sudut V2 yg setara dg jarak bacaan LIDAR
        function [sudutHaluan] = menghitungHaluanBebasBenturan_V2(obj,~)
            %haluan dihitung dgn 1 kriteria yaitu jarak terjauh saja
            dataJarak = obj.dataLIDAR_jarak;
            dataSudut = obj.dataLIDAR_sudut;            
            indekHaluanBenturan = [];
            indekHaluanAman = [];
            dataJarakAman = [];
            dataSudutAman = [];            
            jarakAman = 1.0;
            for indek=1:length(dataJarak)                
                if dataJarak(indek) < jarakAman 
                    indekHaluanBenturan = [indekHaluanBenturan, indek];                    
                else
                    indekHaluanAman = [indekHaluanAman, indek];
                    dataJarakAman = [dataJarakAman, dataJarak(indek)];
                    dataSudutAman = [dataSudutAman, dataSudut(indek)];
                end
            end                        
            sudutHaluan = median(dataSudutAman);
        end
        %memperbarui data radius,poses, velocities setiap id agen
        function memperbaruiDataRadiusPoseVeloAgen(obj,varargin)
            idnyaAgen = varargin{1};
            obj.id = [obj.id idnyaAgen]; %yg nyebabin salah sebelumnya obj.id = [obj.id varargin{1}];
            obj.radius = [obj.radius varargin{2}];   %% radius robot 
            obj.poses(1,idnyaAgen) = varargin{3};%yg nyebabib salah sebelumnya obj.poses(1,obj.id) = varargin{3};
            obj.poses(2,idnyaAgen) = varargin{4};
            obj.poses(3,idnyaAgen) = varargin{5};
            obj.velos(1,idnyaAgen) = varargin{6};
            obj.velos(2,idnyaAgen) = varargin{7};
            obj.velos(3,idnyaAgen) = varargin{8};
            
            if idnyaAgen == obj.idAgen
                obj.r_i = obj.radius(idnyaAgen);%iki seng garakno salah obj.r_i = obj.radius(obj.id);
                obj.poseEgoX = obj.poses(1,idnyaAgen);
                obj.poseEgoY = obj.poses(2,idnyaAgen);
                obj.poseEgoPsi = obj.poses(3,idnyaAgen);
                obj.p_i = [obj.poseEgoX; obj.poseEgoY; obj.poseEgoPsi];
                obj.veloEgoVx = obj.velos(1,idnyaAgen);
                obj.veloEgoVy = obj.velos(2,idnyaAgen);
                obj.veloEgoW = obj.velos(3,idnyaAgen);
                obj.v_i = [obj.veloEgoVx; obj.veloEgoVy; obj.veloEgoW];
                obj.desiredVelocity = obj.v_i;%Rabu 19 Februari 2020,11:25AM
            else
                obj.r_j = obj.radius(idnyaAgen);
                obj.poseObsX = obj.poses(1,idnyaAgen);
                obj.poseObsY = obj.poses(2,idnyaAgen);
                obj.poseObsPsi = obj.poses(3,idnyaAgen);
                obj.p_j = [obj.poseObsX; obj.poseObsY; obj.poseObsPsi];
                obj.veloObsVx = obj.velos(1,idnyaAgen);
                obj.veloObsVy = obj.velos(2,idnyaAgen);
                obj.veloObsW = obj.velos(3,idnyaAgen);
                obj.v_j = [obj.veloObsVx; obj.veloObsVy; obj.veloObsW];
            end
        end
        %menyiapkan FV
        %function [FV] = menyiapkanFV(obj,minVelo,maxVelo,kepadatan)
        function [FV] = menyiapkanFV(obj,~)%Rabu 19 Februari 2020, 10:41AM
            minVelo = -ones(3,1)*obj.maxSpeed;
            maxVelo = ones(3,1)*obj.maxSpeed;
            kepadatan = obj.kepadatan;
            jmlDataVelo = numel(minVelo);%jml data velo = 3, yaitu [Vx; Vy; w]
            jmlData = kepadatan^jmlDataVelo; %misal: 2^3 = 8, utk kepadatan = 2 dan jmlDataVelo = 3
            jmlPengali = zeros(jmlDataVelo,1); %[0; 0; 0]
            jmlDistribusi = zeros(jmlDataVelo,kepadatan);%3x8, 3baris 8kolom
            % \kol  1    2   
            %================
            %bar|   0    0   |
            %bar|   0    0   |
            %bar|   0    0   |
            for dataKe=1:jmlDataVelo
                %linspace(-4,4,2) ==> -4    4
                jmlDistribusi(dataKe,:) = linspace(minVelo(dataKe), maxVelo(dataKe), kepadatan );
                jmlPengali(dataKe) = jmlData/kepadatan^dataKe;%misal: 8/(2^1) = 4,nilai selanjutnya 2 dan 1
            end %akhir for dataKe=1:jmlDataVelo
            jmlPengali = fliplr(jmlPengali); %[4;2;1]
            FV = zeros(jmlDataVelo, size(jmlDistribusi,2)^jmlDataVelo);%FV seukuran dgn 3 x dimensi jmlDistribusi=3 baris x jmlData kolom
            for dataKe=1:jmlDataVelo %outer loop, misal: dari 1 s/d 3, sbg loop baris = 3
                dataYgDisisipkan = [];
                for dataSisipanKe=1:kepadatan %inner loop, misal dari 1 s/d 2, sbg loop kolom = 2
                    iterasiDistribusi = repmat(jmlDistribusi(dataKe,dataSisipanKe),1,jmlPengali(dataKe));%menyalin jmlDistribusi ke iterasiDistribusi
                    dataYgDisisipkan = horzcat(dataYgDisisipkan, iterasiDistribusi);%menyusun iterasiDistribusi secara baris atau horizontal
                end %akhir for dataSisipanKe=1:kepadatan %inner loop
                jmlSalinan = jmlData/size(dataYgDisisipkan,2);
                FV(dataKe,:) = repmat(dataYgDisisipkan,1,jmlSalinan);%menyalin dataYgDisisipkan ke FV dgn dimensi 1 baris x jmlSalinan kolom
            end %akhir for dataKe=1:jmlDataVelo, outer loop
        end %akhir function [FV] = menyiapkanFV(minVelo,maxVelo,kepadatan)
        
        %% <@RumahTalon,Jumat21Mei2021,17.27PM: upgrade fungsi>
        function [sudutHaluan] = menghitungHaluanBebasBenturan(obj, ...
                himpunanJarak,jarakAktif, fov, resolusi ...
            )
            %haluan dihitung dgn 1 kriteria yaitu jarak terjauh saja
            dataJarak = obj.dataLIDAR_jarak;
            dataSudut = obj.dataLIDAR_sudut;
            [datanya,indeknya] = sort(dataJarak(:), 'descend')
            %sudutHaluan = dataSudut(indeknya);
            %sudutHaluan = sudutHaluan(1);
            %haluan dihitung dgn 2 kriteria yaitu jarak di sebelah kanan
            %dan kiri dari kandidat haluan harus tidak nol
            indekKandidatHaluan = -1;
            jarakAman = jarakAktif;
            if (fov == 180 || resolusi == 19)
                for indek=1:length(dataJarak)
                    if indeknya(indek) > 5 && indeknya(indek) < 14 %19 titik
    %                 if indeknya(indek) > 3 && indeknya(indek) < 6 %9 titik
                        if dataJarak(indeknya(indek)-1) > jarakAman && ...
                           dataJarak(indeknya(indek)+1) > jarakAman && ...
                           dataJarak(indeknya(indek)-2) > jarakAman && ...
                           dataJarak(indeknya(indek)+2) > jarakAman && ...
                           dataJarak(indeknya(indek)-3) > jarakAman && ...
                           dataJarak(indeknya(indek)+3) > jarakAman && ...
                           dataJarak(indeknya(indek)-4) > jarakAman && ...
                           dataJarak(indeknya(indek)+4) > jarakAman && ...
                           dataJarak(indeknya(indek)-5) > jarakAman && ...
                           dataJarak(indeknya(indek)+5) > jarakAman
                                indekKandidatHaluan = indeknya(indek);
                                break;
                        end %akhir if loop dlm
                    end %akhir if loop luar
                end %akhir for
            elseif (fov == 360 || resolusi == 29)
                for indek=1:length(dataJarak)
                    if indeknya(indek) > 3 && indeknya(indek) < 26 %29 titik: gagal
                    %if indeknya(indek) > 5 && indeknya(indek) < 25 %29 titik: gagal
                    %if indeknya(indek) > 5 && indeknya(indek) < 14 %19 titik
                    %if indeknya(indek) > 3 && indeknya(indek) < 6 %9 titik
                        if dataJarak(indeknya(indek)-1) > jarakAman && ...
                           dataJarak(indeknya(indek)+1) > jarakAman && ...
                           dataJarak(indeknya(indek)-2) > jarakAman && ...
                           dataJarak(indeknya(indek)+2) > jarakAman && ...
                           dataJarak(indeknya(indek)-3) > jarakAman && ...
                           dataJarak(indeknya(indek)+3) > jarakAman %%&& ...
    %                        dataJarak(indeknya(indek)-4) > jarakAman && ...
    %                        dataJarak(indeknya(indek)+4) > jarakAman && ...
    %                        dataJarak(indeknya(indek)-5) > jarakAman && ...
    %                        dataJarak(indeknya(indek)+5) > jarakAman
                                indekKandidatHaluan = indeknya(indek);
                                break;
                        end %akhir if loop dlm
                    end %akhir if loop luar
                end %akhir for
            end % end if
            sudutHaluan = dataSudut(indekKandidatHaluan);
        end 
        
        function [sudutHaluan] = menghitungHaluanBebasBenturan_ASLINYA(obj,~) %%Sabtu01Ags2020
            %Sabtu 31 Okt 2020, 14.53 PM
            %hitung sudut dari robot ke target            
%             orientasiRobot = obj.poses(3,1);
%             penanda = orientasiRobot/norm(orientasiRobot);
%             sudutAntaraRobotDanTarget = obj.menghitungSudutAntarRobotDanTarget2(obj.poses(:,1),obj.poses(:,2));
%             sudutYgDidekati = penanda*sudutAntaraRobotDanTarget;
            %% bagian yg di atas ini bisa dikembangkan nanti agar robot
            %% dpt memilih arah hindar yg dekat dgn arah target berada
            
            %haluan dihitung dgn 1 kriteria yaitu jarak terjauh saja
            dataJarak = obj.dataLIDAR_jarak;
            dataSudut = obj.dataLIDAR_sudut;
            dataJarak(isnan(dataJarak)) = -Inf;%agar NaN tdk jadi no 1
            [datanya,indeknya] = sort(dataJarak(:), 'descend')
            %sudutHaluan = dataSudut(indeknya);
            %sudutHaluan = sudutHaluan(1);
            %haluan dihitung dgn 2 kriteria yaitu jarak di sebelah kanan
            %dan kiri dari kandidat haluan harus tidak nol
            indekKandidatHaluan = -1;
            %jarakAman = 0.5;%kemepeten
            %jarakAman = 1.0;%percobaan0m
            jarakAman = 2.5;%disesuaikan dengan ukuran robot
            for indek=1:length(dataJarak)
                if indeknya(indek) > 3 && indeknya(indek) < 26 %29 titik: gagal
                %if indeknya(indek) > 5 && indeknya(indek) < 25 %29 titik: gagal
                %if indeknya(indek) > 5 && indeknya(indek) < 14 %19 titik
                %if indeknya(indek) > 3 && indeknya(indek) < 6 %9 titik
                    if dataJarak(indeknya(indek)-1) > jarakAman && ...
                       dataJarak(indeknya(indek)+1) > jarakAman && ...
                       dataJarak(indeknya(indek)-2) > jarakAman && ...
                       dataJarak(indeknya(indek)+2) > jarakAman && ...
                       dataJarak(indeknya(indek)-3) > jarakAman && ...
                       dataJarak(indeknya(indek)+3) > jarakAman %%&& ...
%                        dataJarak(indeknya(indek)-4) > jarakAman && ...
%                        dataJarak(indeknya(indek)+4) > jarakAman && ...
%                        dataJarak(indeknya(indek)-5) > jarakAman && ...
%                        dataJarak(indeknya(indek)+5) > jarakAman
                            indekKandidatHaluan = indeknya(indek);
                            break;
                    end %akhir if loop dlm
                end %akhir if loop luar
            end %akhir for
            obj.haluan = dataSudut(indekKandidatHaluan);
            obj.indekHaluan =  indekKandidatHaluan;
            obj.jarakPadaIndekHaluan = dataJarak(indekKandidatHaluan);
            sudutHaluan = dataSudut(indekKandidatHaluan);%% aslinya
            
            %% Sabtu 31 Okt 2020, 15.58 PM, semoga lebih efektif efisien
            %sudutHaluan = penanda*((sudutHaluan + sudutAntaraRobotDanTarget)/2);
            
            %sudutHaluan = dataSudut(indekKandidatHaluan-1);
        end %akhir function
        %menghitung steering dari sudut V2 yg setara dg jarak bacaan LIDAR
        
        function [kecepatanLinear, kecepatanAngular] = menghitungKecepatanBebasBenturan(obj,vel, ...
            himpunanJarak,jarakAktif, fov, resolusi ...
        )
            obj.himpunanFV = obj.menyiapkanFV();
            obj.himpunanVO = obj.menghitungHVOmajemuk(); %Ahad 16 Februari 2020 01:49AM
            obj.himpunanEV = obj.menghitungEV(obj.himpunanFV, obj.himpunanVO);
            obj.himpunanAV = obj.menghitungAV(obj.himpunanEV);
            kecepatanLinear = obj.himpunanAV(1);
            kecepatanLinear = norm(kecepatanLinear);
            kelajuan = norm(kecepatanLinear);
            haluan = kecepatanLinear/kelajuan;            
            obj.himpunanHeadingVector = haluan;            
            kecepatanAngular = obj.menghitungHaluanBebasBenturan( ...
                himpunanJarak,jarakAktif, fov, resolusi ...
            );
        end
        %% </@RumahTalon,Jumat21Mei2021,17.27PM>
        
        %menghitung kecepatan bebas benturan
        %function [kecepatanLinear, kecepatanAngular] = menghitungKecepatanBebasBenturan(obj,vel,jenisAgen)
        function [kecepatanLinear, kecepatanAngular] = menghitungKecepatanBebasBenturan_ASLINYA(obj,vel)
            %hitung FV
            %obj.himpunanFV = obj.menyiapkanFV(obj.maxSpeed,obj.maxSpeed,obj.kepadatan);
            obj.himpunanFV = obj.menyiapkanFV();
            
            %hitung VO majemuk
            %obj.himpunanVO = obj.menghitungVOmajemuk();
            obj.himpunanVO = obj.menghitungHVOmajemuk(); %Ahad 16 Februari 2020 01:49AM
            
            %hitung EV = FV - VO
            obj.himpunanEV = obj.menghitungEV(obj.himpunanFV, obj.himpunanVO);
            
            %hitung AV = EV yg optimal: paling dekat, atau kriteria lain
            %obj.himpunanAV = menghitungAV(obj.desiredVelocity, obj.himpunanEV);
            %obj.himpunanAV = obj.menghitungAV(vel, obj.himpunanEV);
            obj.himpunanAV = obj.menghitungAV(obj.himpunanEV);
% % % %             kecepatanLinear = obj.himpunanAV(1);
            kecepatanLinear = sqrt(obj.himpunanAV(1)^2+obj.himpunanAV(2)^2);
% % % %             kecepatanAngular = obj.himpunanAV(3);
%             kecepatanLinear = norm(kecepatanLinear);
% % %             % APPLY SPEED CONSTRAINT
% % %             kelajuanLinear = norm(kecepatanLinear);
% % %             if kelajuanLinear > obj.maxSpeed
% % %                 arahYgDisarankan = kecepatanLinear/kelajuanLinear;
% % %                 kecepatanLinear = arahYgDisarankan(1)*obj.maxSpeed;
% % %             end
% % % %             
% % %             kelajuan = norm(kecepatanLinear);
% % %             haluan = kecepatanLinear/kelajuan;            
% % %             obj.himpunanHeadingVector = haluan;            
% % %             if isnan(haluan)
% % %                 haluan = [1;0;0];  % jika heading nol maka pertahankan nilai sebelumnya
% % %             end
% % %             kecepatanAngular = haluan(1);
            %kecepatanAngular = -0.5*haluan(1);            
            %kecepatanAngular = exampleHelperComputeAngularVelocity(-0.1*haluan(1),haluan(1));
            %kecepatanAngular = exampleHelperComputeAngularVelocity(obj.menghitungHaluanBebasBenturan()/5,obj.menghitungHaluanBebasBenturan()/2); %ditambahkan pada Rabu 05 Februari 2020 14:12
            %%ditambahkan pada Rabu 05 Februari 2020 14:41, selamat tapi
            %%osilasi
            %kecepatanAngular = 0.3*obj.menghitungHaluanBebasBenturan();
            %kecepatanAngular = 0.5*obj.menghitungHaluanBebasBenturan();
            %rabu 05 feb 2020
            kecepatanAngular = obj.menghitungHaluanBebasBenturan_ASLINYA();
            %Senin 10 feb 2020, 20:24
            %kecepatanAngular = obj.menghitungHaluanBebasBenturan_V2();
        end %akhir function [kecepatanLinear, kecepatanAngular] = menghitungKecepatanBebasBenturan(obj,vel)
        %menghitung HVO dari beberapa objek
        function [HVOmajemuk] = menghitungHVOmajemuk(obj,~)
            %VOmajemuk = zeros(1, obj.jmlAgen-2);
            %VOmajemuk = cell(1, obj.jmlAgen-2);
            HVOmajemuk = [];
            obj.jmlAgen = length(obj.id);
            %agen objects dimulai dari indek = 3, indek 1 utk robot, indek 2 utk target
            indekVO = 1;
            for indekAgen=3:obj.jmlAgen 
                %membaca data objek agen
                %obj.jenis(indekAgen)
                obj.p_j = obj.poses(:,indekAgen);
                obj.v_j = obj.velos(:,indekAgen);
                obj.r_j = obj.radius(indekAgen);                
                %VO_i = obj.menghitungVOtunggal(obj.p_i,obj.v_i,obj.r_i,obj.p_j,obj.v_j,obj.r_j);
                %VO_i = obj.menghitungHVOtunggal(obj.p_i,obj.v_i,obj.r_i,obj.p_j,obj.v_j,obj.r_j);
                obj.indekAgenSaatIni = indekAgen;
                VO_i = obj.menghitungHVOtunggal(obj.p_i,obj.p_j);
                VO_i.objectID = indekAgen;                
                % VOmajemuk = [VOmajemuk,VO_i];% simpan VO dari setiap objects
                %VOmajemuk(indekVO) = VO_i;% simpan VO dari setiap objects
                HVOmajemuk = [HVOmajemuk, VO_i];
                indekVO = indekVO + 1;
            end %akhir for indekAgen=3:obj.jmlAgen 
        end %akhir function [HVOmajemuk] = menghitungHVOmajemuk(obj,~)
        %menghitung kecepatan yg menyebabkan benturan utk dihimpun dalam HVO
        %function [HVO] = menghitungHVOtunggal(obj,pr,vr,rr,po,vo,ro)
        function [HVO] = menghitungHVOtunggal(obj,pr,po)
            indekAgen = obj.indekAgenSaatIni
            jenisObjects = obj.modalitas(indekAgen)
            if jenisObjects == "mobileRobot"
                %HVO = obj.menghitungNLVOtunggal(pr,vr,rr,po,vo,ro);
                HVO = obj.menghitungNLVOtunggal(pr,po);
            elseif jenisObjects == "target"
                %HVO = obj.menghitungVOtunggal(pr,vr,rr,po,vo,ro);
                HVO = obj.menghitungVOtunggal(pr,po);
            elseif jenisObjects == "statisObjects"
                %HVO = obj.menghitungVOtunggal(pr,vr,rr,po,vo,ro);
                HVO = obj.menghitungVOtunggal(pr,po);
            elseif jenisObjects == "robotLain"
                %HVO = obj.menghitungRVOtunggal(pr,vr,rr,po,vo,ro);
                HVO = obj.menghitungRVOtunggal(pr,po);
            elseif jenisObjects == "nonLinearObjects"
                %HVO = obj.menghitungNLVOtunggal(pr,vr,rr,po,vo,ro);
                HVO = obj.menghitungNLVOtunggal(pr,po);
            end %akhir if jenisObjects == "mobileRobot"
        end %akhir function [HVO] = menghitungHVOtunggal(pr,po)
        %menghitung VO dari beberapa objek
        function [VOmajemuk] = menghitungVOmajemuk(obj,~)
            %VOmajemuk = zeros(1, obj.jmlAgen-2);
            %VOmajemuk = cell(1, obj.jmlAgen-2);
            VOmajemuk = [];
            obj.jmlAgen = length(obj.id);
            %agen objects dimulai dari indek = 3, indek 1 utk robot, indek 2 utk target
            indekVO = 1;
            for indekAgen=3:obj.jmlAgen 
                %membaca data objek agen
                %obj.jenis(indekAgen)
                obj.p_j = obj.poses(:,indekAgen);
                obj.v_j = obj.velos(:,indekAgen);
                obj.r_j = obj.radius(indekAgen);
                %tau_j = 2;
                %VO_i = menghitungVOtunggal(obj.p_i,obj.v_i,obj.r_i,obj.p_j,obj.v_j,obj.r_j,tau_j);
                VO_i = obj.menghitungVOtunggal(obj.p_i,obj.v_i,obj.r_i,obj.p_j,obj.v_j,obj.r_j);
                VO_i.objectID = indekAgen;                
                % VOmajemuk = [VOmajemuk,VO_i];% simpan VO dari setiap objects
                %VOmajemuk(indekVO) = VO_i;% simpan VO dari setiap objects
                VOmajemuk = [VOmajemuk, VO_i];
                indekVO = indekVO + 1;
            end %akhir for indekAgen=3:obj.jmlAgen 
        end %akhir function [VOmajemuk] = menghitungVOmajemuk(obj,~)
        %menghitung kecepatan yg menyebabkan benturan utk dihimpun dalam VO
        %function [VO] = menghitungVOtunggal(obj,pr,vr,rr,po,vo,ro,tau)
        %function [VO] = menghitungVOtunggal(obj,pr,vr,rr,po,vo,ro)
        function [VO] = menghitungVOtunggal(obj,pr,po)
            %menghitung parameter VO berbasis pose: pro,dro,alpharo,phiro
            %bagian RCC ini dihitung sekali utk setiap pasangan pose
            %RCC = RelativeCollisionCone
            rr = obj.r_i;
            ro = obj.r_j;
            pro = po - pr; % posisi relatif obstacle thdp robot
            rro = ro + rr; % ro yang diperbesar
            dro = norm(pro); %distance ro = jarak dari o ke r
            obj.alpharo = atan2(pro(2),pro(1)); %? ro = sudut garis dro thdp sumbu x = sudut antara obstacle dan robot
            obj.phiro = asin(rro/dro); %? ro = separuh sudut kerucut benturan
            
            %menghitung parameter VO berbasis velocity:
            %vro,magnitud_vro,betaro,psiro,apakah_vo_di_dlm_VO 
            %bagian ACC ini dihitung beberapa kali utk alternatif vr kandidat            
            %ACC = AbsoluteCollisionCone
            vr = obj.v_i;
            vo = obj.v_j;
%             [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
%                 obj.cekKandidat_vo_thdp_kerucutVO(vr,vo);
            apakahRVO = 0; %jenis VO berarti bukan RVO 
            [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
                obj.cekKandidat_vo_thdp_kerucutVO(vr,vo,apakahRVO);
            
            %kirim setiap parameter VO dlm structure
            VO = struct( ...
                'apex', vo, ...
                'posisiRelatif',pro, ...
                'radiusYgDiperbesar',rro, ...
                'sumbuVO',dro, ...
                'sudutSumbuVO', obj.alpharo, ...
                'separuhSudutKerucut',obj.phiro, ...
                'sudutKerucut',2*obj.phiro, ...
                'kecepatanRelatif',vro, ...
                'kelajuanRelatif',magnitud_vro, ...
                'sudutKecepatanRelatif',betaro, ...
                'sudut_vro_pro',psiro, ...
                'apakah_vo_di_dlm_VO',apakah_vo_di_dlm_VO, ...
                'waktuBenturan',tc, ...
                'apakahRVO',apakahRVO ...
            );
        end %akhir function [VO] = menghitungVOtunggal(pr,vr,rr,po,vo,ro)
        % definisi RVO
        %function [RVO] = menghitungRVOtunggal(obj,pr,vr,rr,po,vo,ro)
        function [RVO] = menghitungRVOtunggal(obj,pr,po)
% % %             %rr = obj.r_i;
% % %             %ro = obj.r_j;
% % %             vr = obj.v_i;
% % %             vo = obj.v_j;
% % %             
% % %             %[RVO] = obj.menghitungVOtunggal(pr,vr,rr,po,vo,ro);
% % %             [RVO] = obj.menghitungVOtunggal(pr,po);
            %menghitung parameter VO berbasis pose: pro,dro,alpharo,phiro
            %bagian RCC ini dihitung sekali utk setiap pasangan pose
            %RCC = RelativeCollisionCone
            rr = obj.r_i;
            ro = obj.r_j;
            pro = po - pr; % posisi relatif obstacle thdp robot
            rro = ro + rr; % ro yang diperbesar
            dro = norm(pro); %distance ro = jarak dari o ke r
            obj.alpharo = atan2(pro(2),pro(1)); %? ro = sudut garis dro thdp sumbu x = sudut antara obstacle dan robot
            obj.phiro = asin(rro/dro); %? ro = separuh sudut kerucut benturan
            
            %menghitung parameter VO berbasis velocity:
            %vro,magnitud_vro,betaro,psiro,apakah_vo_di_dlm_VO 
            %bagian ACC ini dihitung beberapa kali utk alternatif vr kandidat            
            %ACC = AbsoluteCollisionCone
            vr = obj.v_i;
            vo = obj.v_j;
%             [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
%                 obj.cekKandidat_vo_thdp_kerucutVO(vr,vo);
            apakahRVO = 1; %jenis VO berarti bukan RVO 
            [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
                obj.cekKandidat_vo_thdp_kerucutVO(vr,vo,apakahRVO);
            
            %kirim setiap parameter VO dlm structure
            RVO = struct( ...
                'apex', 0.5*vr + 0.5*vo, ...
                'posisiRelatif',pro, ...
                'radiusYgDiperbesar',rro, ...
                'sumbuVO',dro, ...
                'sudutSumbuVO', obj.alpharo, ...
                'separuhSudutKerucut',obj.phiro, ...
                'sudutKerucut',2*obj.phiro, ...
                'kecepatanRelatif',vro, ...
                'kelajuanRelatif',magnitud_vro, ...
                'sudutKecepatanRelatif',betaro, ...
                'sudut_vro_pro',psiro, ...
                'apakah_vo_di_dlm_VO',apakah_vo_di_dlm_VO, ...
                'waktuBenturan',tc, ...
                'apakahRVO',apakahRVO ...
            );
            %RVO.apex = 0.5*vr + 0.5*vo; % posisi apex diset standar RVO
        end %akhir function [RVO] = menghitungRVOtunggal(obj,pr,vr,rr,po,vo,ro)
        %menghitung kecepatan yg menyebabkan benturan utk dihimpun dalam NLVO
        %function [NLVO] = menghitungNLVOtunggal(obj,pr,vr,rr,po,vo,ro)
        function [NLVO] = menghitungNLVOtunggal(obj,pr,po)
            %menghitung parameter VO berbasis pose: pro,dro,alpharo,phiro
            %bagian RCC ini dihitung sekali utk setiap pasangan pose
            %RCC = RelativeCollisionCone
            rr = obj.r_i;
            ro = obj.r_j;            
            pro = po - pr; % posisi relatif obstacle thdp robot
            rro = ro + rr; % ro yang diperbesar
            dro = norm(pro); %distance ro = jarak dari o ke r
            obj.alpharo = atan2(pro(2),pro(1)); %? ro = sudut garis dro thdp sumbu x = sudut antara obstacle dan robot
            obj.phiro = asin(rro/dro); %? ro = separuh sudut kerucut benturan
            
            %menghitung parameter VO berbasis velocity:
            %vro,magnitud_vro,betaro,psiro,apakah_vo_di_dlm_VO 
            %bagian ACC ini dihitung beberapa kali utk alternatif vr kandidat            
            %ACC = AbsoluteCollisionCone
            vr = obj.v_i;
            vo = obj.v_j;
%             [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
%                 obj.cekKandidat_vo_thdp_kerucutVO(vr,vo);                        
            apakahRVO = 0; %jenis VO berarti bukan RVO 
            [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = ...
                obj.cekKandidat_vo_thdp_kerucutVO(vr,vo,apakahRVO);
            
            %kirim setiap parameter VO dlm structure
            NLVO = struct( ...
                'apex', vo, ...
                'posisiRelatif',pro, ...
                'radiusYgDiperbesar',rro, ...
                'sumbuVO',dro, ...
                'sudutSumbuVO', obj.alpharo, ...
                'separuhSudutKerucut',obj.phiro, ...
                'sudutKerucut',2*obj.phiro, ...
                'kecepatanRelatif',vro, ...
                'kelajuanRelatif',magnitud_vro, ...
                'sudutKecepatanRelatif',betaro, ...
                'sudut_vro_pro',psiro, ...
                'apakah_vo_di_dlm_VO',apakah_vo_di_dlm_VO, ...
                'waktuBenturan',tc, ...
                'apakahRVO',apakahRVO ...
            );
        end %akhir function [NLVO] = menghitungNLVOtunggal(pr,vr,rr,po,vo,ro)
        %menghitung parameter VO berbasis velocity:
        %vro,magnitud_vro,betaro,psiro,apakah_vo_di_dlm_VO 
        %bagian ini dihitung beberapa kali utk alternatif vr kandidat
        %sehingga perlu dibuatkan function
        %function [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = cekKandidat_vo_thdp_kerucutVO(obj,vr,vo)
%         function [vro, magnitud_vro, betaro, psiro, tc, ...
%                 apakah_vo_di_dlm_VO] = ...
%                 cekKandidat_vo_thdp_kerucutVO(obj,vr,vo)
        function [vro, magnitud_vro, betaro, psiro, tc, ...
                apakah_vo_di_dlm_VO] = ...
                cekKandidat_vo_thdp_kerucutVO(obj,vr,vo,apakahRVO)
            apakah_vo_di_dlm_VO = 0;            
            if apakahRVO
                vro = vr - (0.5*vr + 0.5*vo);%kecepatan relatif antara obstacle dan robot utk RVO, utk VO, vro = vr - vo
            else
                vro = vr - vo;
            end
            magnitud_vro = norm(vro); %kelajuan relatif antara obstacle dan robot
            betaro = atan2(vro(2),vro(1));%? ro = sudut kecepatan relatif antara obstacle dan robot
            psiro = abs(betaro - obj.alpharo); %? ro = sudut antara kecepatan relatif vro dan pose pro
            if psiro <= obj.phiro
                apakah_vo_di_dlm_VO = 1;
%                 obj.dro
%                 psiro
%                 obj.rro
%                 vro
                tc = ( (obj.dro.*cos(psiro)) - sqrt( obj.rro^2 - (obj.dro^2.*sin(psiro)^2) ) )/vro(1); %waktu benturan = time to collision
            else
                apakah_vo_di_dlm_VO = 0;
                tc = 0;
            end %akhir if - else apakah vo di dlm VO
        end %akhir definisi function [vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = cekKandidat_vo_thdp_kerucutVO(vr,vo)
        %definisi function menghitung escape velocities EV = FV - VO
        %function [EV] = menghitungEV(obj, FV, VO)
        function [EV] = menghitungEV(obj,FV, VO)
            EV = FV;
            jmlDataVelos = size(EV,1);%jika yg tersedia Vx, Vy, w maka jml data velos = 3
            for indekVdiFV=1:size(FV,2)
               kandidat_vr = FV(:,indekVdiFV);
               indekVO = 1; 
               status_vr = 0; %bernilai 0 jika vr di luar VO, bernilai 1 jika vr di dalam VO
               while indekVO <= length(VO) && status_vr ~= 1
                  %[vro, magnitud_vro, betaro, psiro, tc, apakah_vo_di_dlm_VO] = cekKandidat_vo_thdp_kerucutVO(kandidat_vr,VO(indekVO));
                  %[~, apakah_vo_di_dlm_VO] = cekKandidat_vo_thdp_kerucutVO(kandidat_vr,VO(indekVO));
                  %status_vr = apakah_vo_di_dlm_VO;
                  %[~, status_vr] = obj.cekKandidat_vo_thdp_kerucutVO(kandidat_vr,VO(indekVO).apex);
                  [~, status_vr] = obj.cekKandidat_vo_thdp_kerucutVO(kandidat_vr,VO(indekVO).apex,VO(indekVO).apakahRVO);
                  indekVO = indekVO + 1;
                  if status_vr %jika bernilai 1 maka beri nilai NaN
                    EV(:,indekVdiFV) = NaN(jmlDataVelos,1);
                  end %akhir if status_vr
               end %akhir while indekVO <= length(VO) && status_vr ~= 1
               EV = EV(:,~isnan(EV(1,:))); %EV hanya berisi nilai bukan NaN
            end % akhir for indekVdiFV=1:size(FV,2)
        end %akhir function [EV] = menghitungEV(obj, FV, VO)
        %hitung AV = EV yg optimal: paling dekat, atau kriteria lain
        %function [AV] = menghitungAV(obj, vAktual, EV)
        function [AV] = menghitungAV(obj,EV)
            vAktual = obj.desiredVelocity;
            jmlDataVelos = numel(vAktual);%jml data velos = 3 = [Vx; Vy; w]
            %menyiapkan variabel berisi himpunan EV di tumpukan atas
            %dan vektor kosongan di bagian bawahnya, 
            %nantinya ini berisi jarak antara vAktual dan kandidat vr*
            EVdgnJarakVeloAktualDanKandidat = vertcat(EV,zeros(1,size(EV,2)));
            
            %Kalo pakai ini muncul: Error using norm. The only matrix norms available are 1, 2, inf, and 'fro'.
            %EVdgnJarakVeloAktualDanKandidat = vertcat(EV,zeros(3,size(EV,2)));
            
            for indekEV=1:size(EVdgnJarakVeloAktualDanKandidat,2)
               %hitung jarak vAktual dan vr* = ||vAktual - vr* || dan simpan di vektor kosongan zeros
               %kandidatVr = EVdgnJarakVeloAktualDanKandidat(1:jmlDataVelos,indekEV);
               kandidatVr = EVdgnJarakVeloAktualDanKandidat(1,indekEV);
               EVdgnJarakVeloAktualDanKandidat(jmlDataVelos+1,indekEV) = ...
                   norm(vAktual(1),kandidatVr);               
            end %akhir for indekEV=1:size(EVdgnJarakVeloAktualDanKandidat,2)
            [~,indekJarakMinimal] = min(EVdgnJarakVeloAktualDanKandidat(jmlDataVelos+1),[],2);
            AV = EVdgnJarakVeloAktualDanKandidat(1:jmlDataVelos,indekJarakMinimal);
            if isempty(AV)
                AV = zeros(jmlDataVelos,1);
            end %akhir if isempty(AV)
        end %akhir function [AV] = menghitungAV(obj, vAktual, EV)
    end %akhir definisi methods
end %akhir definisi class

