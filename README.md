# Drone Flight Controller Project

Drone Video: https://www.youtube.com/watch?v=_IZWb5vrdCY 

**DRONE MECHANICAL ASSEMBLY**




**MATLAB STATE SPACE MODEL**


```matlab
m1 = 600/1000;  % drone mass in grams
r = 170/1000;   % radius of spherical center mass
m2 = 30/1000;   % motor mass grams
l = 500/1000;   % length of drone arms in meter
g = 9.81;   % gravity
l_3s = 63.62; % motor coefficient l_3s = Kt/Kf   |   F = Kf * w^2    | T = Kt * w^2
c_3s = 1/l_3s;
% l_4s = 58.80;
% c_4s = 1/l_4s;
Ixx=(2*m1*(r^2)/5)+2*m2*(l^2); % Moment of Inertia along x-axis
Iyy=(2*m1*(r^2)/5)+2*m2*(l^2); % Moment of Inertia along y-axis
Izz=(2*m1*(r^2)/5)+4*m2*(l^2); % Moment of Inertia along z-axis
A = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 -g 0 0 0 0;
     0 0 0 0 0 0 g 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     -1/m1 -1/m1 -1/m1 -1/m1;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 l/Ixx 0 -l/Ixx;
     l/Iyy 0 -l/Iyy 0;
     -c_3s/Izz c_3s/Izz -c_3s/Izz c_3s/Izz];
C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;];
D = zeros(6, 4);
sys = ss(A,B,C,D);

