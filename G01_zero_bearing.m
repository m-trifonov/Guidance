%==========================================================================
% Tutorial Guidance (Steering behaviors)
% Topic : Zero-bearing guidance method
% Author: M.Trifonov 
% Email: trifonov.m@yahoo.com
% Date(dd-mm-yyyy): 15-08-2020
%==========================================================================
clc;clear;close all;
% Case study, input data
m = 350; %[kg]
S = 0.3; %[m^2]
V = 600; %[m/s]
Vc = 500; %[m/s]
theta_c = deg2rad(0); %[grad]->[rad]
P = 5250; %[N]
Cya = 4; %[1/rad]
rho = 1.225; %[kg/m^3]
g = 9.81; %[m/s^2]
% Initial data
theta(1) = deg2rad(0); %[grad]->[rad]
% Vehicle#1
X(1) = 0; Y(1) = 600; %[m]
% Vehicle#2 (steering)
Xc(1) = 500; Yc(1) = 500; %[m]
% Initial distance
r(1) = sqrt((X(1)-Xc(1))^2 + (Y(1)-Yc(1))^2); %[m]
% Initial sight angle
phi(1) = atan((Y(1)-Yc(1))/(X(1)-Xc(1))); %[rad]
% Angle of attack
alpha(1) = phi(1) - theta(1);
% Input data for simulation
tk = 100; %end time, [s]
dt = 0.01; % step time, [s]
N = tk/dt; % number of iterations
% Plotting#1
figure(1);hold on
xlabel('\itrange \rm[\itm\rm]');ylabel('\italtitude \rm[\itm\rm]');
title('Zero-bearing guidance method');
% Integration ofequations using Euler's method
for i = 2:N
    % distance
    r(i) = r(i-1) + dt*(Vc*cos(phi(i-1)-theta_c)-V*cos(phi(i-1)-theta(i-1)));
    % sight angle
    phi(i) = phi(i-1) + dt*((-Vc*sin(phi(i-1)-theta_c) + V*sin(phi(i-1)-theta(i-1)))/r(i));
    % positions
    X(i) = X(i-1) + dt*(V*cos(theta(i-1)));
    Y(i) = Y(i-1) + dt*(V*sin(theta(i-1)));
    Xc(i) = Xc(i-1) + dt*(Vc*cos(theta_c));
    Yc(i) = Yc(i-1) + dt*(Vc*sin(theta_c));
    % climb angle
    theta(i) = theta(i-1) + dt*((P*sin(alpha(i-1)) + Cya*alpha(i-1)*S*0.5*rho*V^2 - m*g*cos(theta(i-1)))/(m*V));   
    % angle of attack
    alpha(i) = phi(i-1) - theta(i-1);
    % g-force, lateral load
    Ny(i) = ((V*theta(i-1))/g) + cos(theta(i-1));
    % conditional breakpoint
    D = sqrt((X(i)-Xc(i))^2 + (Y(i)-Yc(i))^2);
    if D < 1 %[m]
        break;
    end
    
    % Plotting#2    
    drawnow
    axis([0 3200 400 700])
    plot(X,Y,':b','Linewidth',3),grid on,hold on;
    plot(Xc,Yc,':r','Linewidth',3),grid on,hold on;

end
% figure(1);
% plot(X,Y,':b','Linewidth',3),grid on,hold on;
% plot(Xc,Yc,':r','Linewidth',3),grid on,hold on;
plot(X(end),Y(end),'*g','Linewidth',3),grid on,hold on;

