function [Ju, gut] = funQR1(u)
% Cost function
% Calculates the cost function value
%             J[u]
%
% Programmer: A. Matus-Vargas
% Date: 10 Oct 2017
% 
% Inputs: u
% Outputs: cost function value, cost function gradient
%--------------------------------------------------------

global Ix Iy Iz m g l JTP K R xsave Usave upd

% Simulation data
global t0 h noIter

% Quadrotor parameters and constants
Ix = 0.1167; %0.08;
Iy = 0.1105; %0.08;
Iz = 0.2218;  %0.08;
m = 0.9; %1;
l = 0.2275; %0.1785; %0.24;
g = 9.81;
JTP = 0; % neglected for now
% kp1 = u(1); kp2 = u(2); kp3 = u(3); kp4 = u(4);
% kd1 = u(5); kd2 = u(6); kd3 = u(7); kd4 = u(8);
K = u;
R = 0.0009; %10^-5;
% R = 0.000001;
% w1 = 0;
% w2 = 0;
% w3 = 0;
% w4 = 0;

% ----------------- Simulation --------------------------------------------
% p1 = 0; p2 = 0;
tc = t0;                       % Initial time
NumberEqt = 13;                % Number of states
k = 4;                         % Number of stages of the numerical method
% xc = [0; 0; 0; 0; 0; 0; pi/4; pi/4; pi/4; 0; 0; 0; 0];  % Initial conditions
xc = [0; 0; 0; 0; 0; 0; 0; 0; 0.0; 0; 0; 0; 0];
dxdtmtr = qr6(tc,xc);   % Model time derivatives at time tc
for i = 1: NumberEqt
    xsave(1,i) = xc(i); % Initial conditions are saved
end
for i = 1: 4
    Usave(1,i) = upd(i);
end

% ----------------- Simulation cycle --------------------------------------
for i = 1 : noIter
    xnew = RKStep(@qr6,tc,xc,dxdtmtr,h,k);
    
    for j = 1 : NumberEqt
        xsave(i+1,j) = xnew(j); % State variables are saved for later use      
    end
    
    % Guardamos los valores de los controles PD
    for j = 1 : 4
        Usave(i+1,j) = upd(j);
    end
    
    tc = tc + h;
    xc = xnew;
    dxdtmtr = qr6(tc,xc);    
end

tf = length(xsave);

% Cost function
Ju = xsave(tf,13); %+ ...
%        + w1*(xsave(tf,3) - 1)^2 + w2*xsave(tf,7)^2 + w3*xsave(tf,8)^2 ...
%        + w4*(xsave(tf,9) - pi/6)^2; % set points
 
gut = 0;  % Return null value to preserve format