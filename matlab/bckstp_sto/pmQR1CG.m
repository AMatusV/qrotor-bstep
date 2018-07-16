% Main code for CG
%
% Quadrotor 6 degrees of freedom
%
% This program simulates the response of the quadrotor subjeted to a
% nonlinear control. The parametric control input u = (k1, k2, ..., kn)^T
% which drives the system to desired state (xd, yd, zd, psid) = (1,
% 1, 1, 0) is found.
%
% Programmer: A. Matus-Vargas
% Date: 4 Oct 2017 
% 
% Inputs: none
%
%------------------------------------------------

% Clear memory and clean window
clc; clear;

% Global variables
global ContEvalf  % Number of callings to the functon
global ContIter   % Number of iterations of the algorithm
    
% Simulation data
global t0 h noIter xsave Usave stoSignals
    
t0 = 0.0;    % Initial time
tend = 10;    % Final time
h = 0.02;   % Integration step
noIter = ceil((tend - t0)/h); % Number of iterations to reach tend
    
% Space is reserved for
Usave = zeros(noIter+1, 4);   % Control inputs
xsave = zeros(noIter+1, 13);  % State variables

stoSignals = RandNumGen(tend, noIter+1);
    
% Initialize
ContEvalf = 0;
ContIter = 0;
% n = 12;
% u = zeros(n, 1);
% for i= 1:n 
%     u(i) = 0.1; % Initial values for u
% end
u = [1.61973845220904; 1.63983782403252; 2.56274647692537; ...
     1.60440398658478; 1.63847677211738; 2.56429456750881; ...
     5.38874642510460; 5.34909801308949; 0.501731055564672; ...
     5.38336697292691; 5.39597864101831; 0.397710228517107];
    
% CG options
opts = zeros(9,1);
opts(1) = 2;
opts(2) = 2;
   
fprintf ('Conjugate Gradient Method\n');
fprintf ('Gradiente aproximado por diferencias');
% [X, info, perf] = conj_grad(@IntrfzDC61, u, opts);

% u* = arg min J(u)
u = conj_grad(@IntrfzQR1, u, opts);   % x = u* = arg min J(u)
fprintf ('\nOptimal u*    = %.7f',u);
[f,g] = funQR1(u);
fprintf ('\nf(x)         = %.7f',f);
fprintf ('\nNumero de iteraciones = %d',ContIter);
fprintf ('\nNumero de evaluaciones de la funcion = %d\n',ContEvalf);
    
% Plots
tsave = linspace(0, tend, noIter+1)';

% figure(1)
% plot(tsave, xsave(:,1,:), tsave, xsave(:,2,:), tsave, xsave(:,3,:));
% hold on 
% plot(tsave, 1+stoSignals(:,1), '--', tsave, 1+stoSignals(:,2), '--', tsave, 1+stoSignals(:,3), '--')
% xlabel('Time [sec]');
% ylabel('Magnitude [m]'); % left y-axis
% legend('x', 'y', 'z', 'x_{ref}', 'y_{ref}', 'z_{ref}');
% % axis([0 5 -0.2 1.0 ]),
% grid

% figure(2)
% plot(tsave, xsave(:,7,:), tsave, xsave(:,8,:), tsave, xsave(:,9,:));
% xlabel('Time [sec]');
% ylabel('Magnitude [rad]'); % left y-axis
% legend('\phi', '\theta', '\psi');
% % axis([0 5 -0.2 1.0 ]),
% grid

figure(1)
plot(tsave, xsave(:,1,:), tsave, stoSignals(:,1)+1, '--');
xlabel('Time [sec]');
ylabel('Magnitude [m]'); % left y-axis
legend('x', 'x_{ref}');
% axis([0 5 -0.2 1.0 ]),
grid

figure(2)
plot(tsave, xsave(:,2,:), tsave, stoSignals(:,2)+1, '--');
xlabel('Time [sec]');
ylabel('Magnitude [m]'); % left y-axis
legend('y', 'y_{ref}');
% axis([0 5 -0.2 1.0 ]),
grid

figure(3)
plot(tsave, xsave(:,3,:), tsave, stoSignals(:,3)+1, '--');
xlabel('Time [sec]');
ylabel('Magnitude [m]'); % left y-axis
legend('z', 'z_{ref}');
% axis([0 5 -0.2 1.0 ]),
grid

figure(4)
plot(tsave, stoSignals(:,4:6));
xlabel('Time [sec]');
ylabel('Magnitude [Nm]'); % left y-axis
legend('\delta_\phi', '\delta_\theta', '\delta_\psi');
grid

clear tend t0 opts noIter n i h g f ContIter ContEvalf 
    