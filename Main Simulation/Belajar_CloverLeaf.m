%% MoBotDiffeSteer merupakan class yg mendefinisikan property dan method
%% dari path planner bertipe CloverLeaf
%% copyleft by Muhammad Fuad, Senin 16 Des 2019, @rumah Talon Madura

%% cara makai 
%  utk bentuk CloverLeaf
%  Belajar_CloverLeaf(0.01, 10, 5, 8,8) ==> utk full 1 daun muncul
%  Belajar_CloverLeaf(0.01, 20, 5, 8,8) ==> utk full 2 daun muncul
%  Belajar_CloverLeaf(0.01, 30, 5, 8,8) ==> utk full 3 daun muncul
%  Belajar_CloverLeaf(0.01, 40, 5, 8,8) ==> utk full 4 daun muncul

function [x,y] = Belajar_CloverLeaf(del_theta, k, amplitude, XC, YC)
%function [x,y] = Belajar_Rose(del_theta, k, amplitude)
% inputs:
%   del_theta = del_theta is the discrete step size for discretizing the continuous range of angles from 0 to 2*pi
%   k = petal coefficient
%      if k is odd then k is the number of petals
%      if k is even then k is half the number of petals
%   amplitude = length of each petal
% outputs:
%   a 2D plot from calling this function illustrates an example of trigonometry and 2D Cartesian plotting
theta = 0:del_theta:2*pi;
%amplitude yg TDK dibagi 3 utk ukuran environment dari 0 hingga 15 MAKA
%ukuran kurva KEGEDEAN
% x = (amplitude) + (amplitude)*cos(k*theta).*cos(theta);
% y = (amplitude) + (amplitude)*cos(k*theta).*sin(theta);
%amplitude harus dibagi 3 utk ukuran environment dari 0 hingga 15

% ini nilai aslinya
% x = (amplitude/3)*cos(k*theta).*cos(theta);
% y = (amplitude/3)*cos(k*theta).*sin(theta);
% % % x = (amplitude)*cos(k*theta).*cos(theta);%pakai ini multi fungsi tinggal ganti k
% % % y = (amplitude)*cos(k*theta).*sin(theta);
% % % x = (amplitude)*sin(k*theta).*sin(theta);%pakai ini multi fungsi tinggal ganti k utk cloverleaf TIDUR
% % % y = (amplitude)*sin(k*theta).*cos(theta);
% ini nilai aslinya

% ini utk uji formula (9) dlm paper1
%cloverleaf tegak
% % x = (amplitude/3)*cos(k*theta).*cos(2*k*theta);
% % y = (amplitude/3)*sin(k*theta).*cos(2*k*theta);
% % x = (amplitude)*cos(k*theta).*cos(2*k*theta);%pakai yg ini utk tegak
% % y = (amplitude)*sin(k*theta).*cos(2*k*theta);
%cloverleaf tidur
% % % x = (amplitude/3)*sin(k*theta).*sin(2*k*theta);
% % % y = (amplitude/3)*cos(k*theta).*sin(2*k*theta);
% % % x = (amplitude)*sin(k*theta).*sin(2*k*theta);%pakai yg ini tidur
% % % y = (amplitude)*cos(k*theta).*sin(2*k*theta);
x = XC + (amplitude)*sin(k/40*theta).*sin(2*k/40*theta);%pakai yg ini tidur, utk diskrit
y = YC + (amplitude)*cos(k/40*theta).*sin(2*k/40*theta);
%lingkaran
% x = (amplitude/3).*sin(k*theta).*sin(k*theta);
% y = (amplitude/3).*cos(k*theta).*sin(k*theta);
% % % x = (amplitude).*sin(k*theta).*sin(k*theta);%pakai ini lingkaran
% % % y = (amplitude).*cos(k*theta).*sin(k*theta);
%garis miring
% x = (amplitude/3).*sin(2*k*theta).*cos(k*theta);
% y = (amplitude/3).*sin(2*k*theta).*cos(k*theta);
% ini utk uji formula (9) dlm paper1

% ini tuk uji bentuk lingkaran
% x = (amplitude/3)*cos(k*theta);
% y = (amplitude/3)*sin(k*theta);
% ini tuk uji bentuk lingkaran

%amplitude harus dibagi 3 dan ditambah 3 utk ukuran environment dari 0
%hingga 15 agar nilainya selalu positif utk semua leaf/daun dari CLOVER LEAF
% x = (amplitude/3) + (amplitude/3)*cos(k*theta).*cos(theta);
% y = (amplitude/3) + (amplitude/3)*cos(k*theta).*sin(theta);
%perlu ditambah dengan XC dan YC agar center dari kurva bisa diatur
% x = XC + x;
% y = YC + y;
% x = x + amplitude;
% y = y + amplitude;
plot(x,y,'k-');   
%xlim ylim perlu utk membatasi ukuran kurva CLOVER LEAF
% % xlim([0 amplitude]);
% % ylim([0 amplitude]);
xlim([0 amplitude*3]);%pakai yg ini utk limit
ylim([0 amplitude*3]);