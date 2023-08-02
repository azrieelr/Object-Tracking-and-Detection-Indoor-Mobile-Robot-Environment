%% MoBotDiffeSteer merupakan class yg mendefinisikan property dan method
%% dari object robot mobil jenis differential-steering
%% copyleft by Muhammad Fuad, Selasa 14 Januari 2020, 16:25, @b203

classdef MoBotDiffSteer_Generasi2 < handle
    %MOBOTDIFFSTEER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        radius_roda;
        radius_roda_kanan;
        radius_roda_kiri;
        diameter_body_robot;
    end %akhir definisi property
    
    methods
        function obj = MoBotDiffSteer(d,r)
            %MOBOTDIFFSTEER Construct an instance of this class
            %   Detailed explanation goes here
            obj.diameter_body_robot = d;
            obj.radius_roda = r;
        end
        function obj = MoBotDiffSteer_Generasi2(d,rR, rL)
            %MOBOTDIFFSTEER Construct an instance of this class
            %   Detailed explanation goes here
            obj.diameter_body_robot = d;
            if rR == rL
                obj.radius_roda = rR;
            end
            obj.radius_roda_kanan = rR;
            obj.radius_roda_kiri = rL;
        end
                
        function [wR, wL] = velocityKeWheelSpeed(obj,v,w)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % __ __       __      ___    __   __
            % | wR |      |1      d/2|  |   v   |
            % |    | =1/r |          |  |       |
            % | wL |      |1     -d/2|  |   w   |
            % __  __      __        __  __     __
            %
% % %             wR = ((1/obj.radius_roda)*1*v) + ((1/obj.radius_roda)*(obj.diameter_body_robot/2)*w);
% % %             wL = ((1/obj.radius_roda)*1*v) + ((1/obj.radius_roda)*(-obj.diameter_body_robot/2)*w);
            % atau rumus kedua jika ban gembos
            % INI ASLINYA
            % __ __       __            ___    __   __
            % | wR |      |1/rR      d/2*rR|  |   v   |
            % |    | =    |                |  |       |
            % | wL |      |1/rL     -d/2*rL|  |   w   |
            % __  __      __              __  __     __
            
%             wR = ((1/obj.radius_roda_kanan)*v) + ((1/obj.radius_roda_kanan)*(obj.diameter_body_robot/2)*w);
%             wL = ((1/obj.radius_roda_kiri)*v) + ((1/obj.radius_roda_kiri)*(-obj.diameter_body_robot/2)*w);
            % INI SEHARUSNYA: Selasa18Mei2021, 13.26AM @RumahTalon, Fuad
            % __ __       __            ___    __   __
            % | wR |      |1/rR      d/2*rL|  |   v   |
            % |    | =    |                |  |       |
            % | wL |      |1/rR     -d/2*rL|  |   w   |
            % __  __      __              __  __     __
            
            wR = ((1/obj.radius_roda_kanan)*v) + ((1/obj.radius_roda_kiri)*(obj.diameter_body_robot/2)*w);
            wL = ((1/obj.radius_roda_kanan)*v) + ((1/obj.radius_roda_kiri)*(-obj.diameter_body_robot/2)*w);
        end %akhir definisi fungsi
        
        function [v,w] = wheelSpeedKeVelocity(obj,wR, wL)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % __ __       __      ___    __    __
            % |  v |      |1/2    1/2|  |   wR   |
            % |    | = r  |          |  |        |
            % |  w |      |1/d   -1/d|  |   wL   |
            % __  __      __        __  __      __
            %
% % %             v = (obj.radius_roda*1/2*wR) + (obj.radius_roda*1/2*wL);
% % %             w = (obj.radius_roda*1/obj.diameter_body_robot*wR) + (obj.radius_roda*-1/obj.diameter_body_robot*wL);
            % atau jika ban gembos
            % INI ASLINYA
            % __ __       __             ___     __    __
            % |  v |      |rR*1/2      rR*1/2|  |   wR   |
            % |    | =    |                  |  |        |
            % |  w |      |rL*1/d   rL*(-1)/d|  |   wL   |
            % __  __      __                __  __      __
            %
%             v = (obj.radius_roda_kanan*1/2*wR) + (obj.radius_roda_kanan*1/2*wL);
%             w = (obj.radius_roda_kiri*1/obj.diameter_body_robot*wR) + (obj.radius_roda_kiri*(-1)/obj.diameter_body_robot*wL);
            % INI YG SEHARUSNYA
            % __ __       __             ___     __    __
            % |  v |      |rR*1/2      rL*1/2|  |   wR   |
            % |    | =    |                  |  |        |
            % |  w |      |rR*1/d   rL*(-1)/d|  |   wL   |
            % __  __      __                __  __      __

            v = (obj.radius_roda_kanan*1/2*wR) + (obj.radius_roda_kiri*1/2*wL);
            w = (obj.radius_roda_kanan*wR/obj.diameter_body_robot) + (obj.radius_roda_kiri*(-wL)/obj.diameter_body_robot);
        end %akhir definisi fungsi
        
        function [kecepatan_global] = forwardKinematics(obj,wR,wL,poseSebelumnya)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % __        __        __                          ___    __    __
            % |  x_dot    |      |r/2*cos(psi)    r/2*cos(psi)   |  |   wR   |
            % |  y_dot    | =    |r/2*sin(psi)    r/2*sin(psi)   |  |        |
            % |  psi_dot  |      |r/d            -r/d            |  |   wL   |
            % __        __        __                          __    __      __
            x_dot = (obj.radius_roda/2*cos(poseSebelumnya(3))*wR) + (obj.radius_roda/2*cos(poseSebelumnya(3))*wL);
            y_dot = (obj.radius_roda/2*sin(poseSebelumnya(3))*wR) + (obj.radius_roda/2*sin(poseSebelumnya(3))*wL);
            psi_dot = (obj.radius_roda*1/obj.diameter_body_robot*wR) + (obj.radius_roda*(-1)/obj.diameter_body_robot*wL);
            kecepatan_global = [x_dot;y_dot;psi_dot]; %kalo ; diganti , maka hasilnya salah yaitu skalar bukan vektor
        end %akhir definisi fungsi
        
        function [wR,wL] = inverseKinematics(obj,x_dot,y_dot,psi_dot)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % __        __        __                          ___     __    __
            % |  wR    |        |cos(psi)    sin(psi)   d/2     |    |   x_dot    |
            % |        | = 1/r  |                               |    |   y_dot    |
            % |  wL    |        |cos(psi)    sin(psi)   -d/2    |    |   psi_dot  |
            % __        __        __                          __      __     __
            wR = ((1/obj.radius_roda)*cos(psi_dot)*x_dot) + ((1/obj.radius_roda)*sin(psi_dot)*y_dot) + ((1/obj.radius_roda)*obj.diameter_body_robot/2*psi_dot);
            wL = ((1/obj.radius_roda)*cos(psi_dot)*x_dot) + ((1/obj.radius_roda)*sin(psi_dot)*y_dot) + ((1/obj.radius_roda)*obj.diameter_body_robot/-2*psi_dot);
        end %akhir definisi fungsi
        
        function kecepatan_global = kecepatanLokalKeGlobal(obj,kecepatan_lokal,pose)
            % __        __        __                          ___    __    __
            % |  x_dot_global    |      |cos(psi)    -sin(psi)   0   |  |   x_dot_local     |
            % |  y_dot_global    | =    |sin(psi)    cos(psi)    0   |  |   y_dot_local     |
            % |  psi_dot_global  |      |0            0          1   |  |   psi_dot_local   |
            % __        __        __                          __    __      __
            psi = pose(3);
            kecepatan_global = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0;0 0 1]*kecepatan_lokal;
        end %akhir definisi fungsi

    end %akhir definisi methods
end %akhir definisi class