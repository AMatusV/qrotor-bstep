% Main code for CG
%
% Quadrotor 6 degrees of freedom
%
% This program simulates the response of the quadrotor subjeted to a
% inner PID controller. The parametric control input u = (k1, k2, ..., 
% kn)^T which drives the system to desired state (phid, thetad, psid) 
% = (0, 0, 0) is found.
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
    
% Datos de la simulacion
global t0 h noIter xsave Usave numberEqs
    
t0 = 0.0;    % Initial time
tend = 5;    % Final time
h = 0.005;    % Integration time step
noIter = ceil((tend - t0)/h); % Number of iterations to reach tend
numberEqs = 16;
    
% Space is reserved for
Usave = zeros(noIter+1, 4);   % Control inputs
xsave = zeros(noIter+1, numberEqs);  % State variables
    
% Initialize
ContEvalf = 0;
ContIter = 0;
n = 9;
u = zeros(n, 1);
for i= 1:n 
    u(i) = 2; % Initial values for u
end
    
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
figure(1)
plot(tsave, xsave(:,1,:), tsave, xsave(:,2,:), tsave, xsave(:,3,:));
xlabel('Time [sec]');
ylabel('Magnitude [m]'); % left y-axis
legend('x', 'y', 'z');
% axis([0 5 -0.2 1.0 ]),
grid

figure(2)
plot(tsave, xsave(:,7,:), tsave, xsave(:,8,:), tsave, xsave(:,9,:));
xlabel('Time [sec]');
ylabel('Magnitude [rad]'); % left y-axis
legend('\phi', '\theta', '\psi');
% axis([0 5 -0.2 1.0 ]),
grid

clear tend t0 opts noIter n i h g f ContIter ContEvalf 
    