%==========================================================================
% Tutorial Guidance (Steering behaviors)
% Topic : Fixed lead-angle guidance method
% Author: M.Trifonov 
% Email: trifonov.m@yahoo.com
% Date(dd-mm-yyyy): 29-08-2020
%==========================================================================
clc;clear;close all;
% case study, input data
m = 350; %[kg]
S = 0.3; %[m^2]
g = 9.81; %[m/s^2]
% vehicle#1
X(1) = 0; Y(1) = 600; %[m]
V = 600; %[m/s]
% vehicle#2
Xc(1) = 500; Yc(1) = 500; %[m]
Vc = 300; %[m/s]
theta_c = deg2rad(0);
% distance, initial state
r(1) = sqrt((Yc(1) - Y(1))^2+(Xc(1) - X(1))^2); %[m]
% sight angle, initial state
phi(1) = atan((Yc(1) - Y(1))/(Xc(1) - X(1))); %[rad]
% calculation of the lead angle
eta = asin((Vc/V)*sin(phi(1)-theta_c));  % [rad]
% climb angle, initial state
theta(1) = phi(1)-eta(1); %[rad]

% input data for simulation
tk = 100; dt = 0.01;  %[s]
N = tk/dt; % number of iterations

% plotting#1
figure(1);hold on
xlabel('\itrange \rm[\itm\rm]');ylabel('\italtitude \rm[\itm\rm]');
title('Fixed lead-angle guidance method');

for i = 2:N
    % climb angle
    theta(i) = phi(i-1)-eta;
    % distance
    r(i) = r(i-1) + dt*(Vc*cos(phi(i-1)-theta_c)-V*cos(phi(i-1)-theta(i-1)));
    % sight angle
    phi(i) = phi(i-1) + dt*((-Vc*sin(phi(i-1)-theta_c)+V*sin(phi(i-1)-theta(i-1)))/r(i-1));
    % g-force, lateral load
    Ny = (-V/g)*((Vc*sin(phi(i-1)-theta_c)-V*sin(eta))/r(i-1));

    % calculation coordinates using the Euler's method
    X(i) = X(i-1) + dt*(V*cos(theta(i-1)));
    Y(i) = Y(i-1) + dt*(V*sin(theta(i-1)));
    Xc(i) = Xc(i-1) + dt*(Vc*cos(theta_c));
    Yc(i) = Yc(i-1) + dt*(Vc*sin(theta_c));
    
    % conditional breakpoint
    D = sqrt((X(i)-Xc(i))^2 + (Y(i)-Yc(i))^2);
    if D < 1 %[m]
        break;
    end
    
    % plotting#2 
    drawnow
    axis([0 1200 400 700])
    plot(X,Y,':b','Linewidth',3),grid on,hold on;
    plot(Xc,Yc,':r','Linewidth',3),grid on,hold on;
end
% plotting#3 
plot(X,Y,'b'),grid on,hold on;
plot(Xc,Yc,'r'),grid on,hold on;
plot(X(end),Y(end),'*g','Linewidth',3),grid on,hold on;