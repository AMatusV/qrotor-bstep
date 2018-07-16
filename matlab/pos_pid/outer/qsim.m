global Ix Iy Iz m g l JTP K R upd numberEqs

% Simulation data
tc = 0.0;   % Initial time
tend = 10;  % Finel time
h = 0.005;   % Integration time step
noIter = ceil((tend - tc)/h); % Number of iterations to reach tend

% Space is reserved for
xsave = zeros(noIter+1, 13);  % State variables
Usave = zeros(noIter+1, 4);   % Control inputs

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
% K = [14.0220; 14.3814; 4.0883; 12.0213; 13.1102; 0.3115; 2.1080; 3.2090; 3.3265; 4.3082; 4.1561; 3.6703];
K = u;
R = 10^-5;
% R = 0.000001;
% w1 = 0;
% w2 = 0;
% w3 = 0;
% w4 = 0;

% ----------------- Simulation --------------------------------------------
numberEqs = 19;                % Number of states
k = 4;                         % Number of stages of the numerical method
% xc = [0; 0; 0; 0; 0; 0; pi/4; pi/4; pi/4; 0; 0; 0; 0];  % Initial conditions
xc = [zeros(6,1); 0.003; 0.002; 0.001; zeros(3,1); 0; zeros(3,1); -0.003; -0.002; 0.0010];
dxdtmtr = qr6(tc,xc); % Model time derivatives at time tc
for i = 1: numberEqs
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

    for j = 1 : 4
        Usave(i+1,j) = upd(j); % PD controller outputs, including phita and theta
    end
    
    tc = tc + h;
    xc = xnew;
    dxdtmtr = qr6(tc,xc);    
end

% Plots
tsave = linspace(0, tend, noIter+1)';
figure(1)
% plot(tsave, xsave(:,1,:), tsave, xsave(:,2,:), tsave, xsave(:,3,:));
plot(tsave, xsave(:,1,:), tsave, xsave(:,2,:), tsave, xsave(:,3,:), tsave, xsave(:,4,:), tsave, xsave(:,5,:), tsave, xsave(:,6,:));
xlabel('Time [sec]');
ylabel('Magnitude [m, m/s]'); % left y-axis 
% legend('x', 'y', 'z');
legend('x', 'y', 'z', 'v_x', 'v_y', 'v_z');
% axis([0 5 -0.2 1.0 ]), 
grid
    
figure(2)
% plot(tsave, xsave(:,7,:), tsave, xsave(:,8,:), tsave, xsave(:,9,:));
plot(tsave, xsave(:,7,:), tsave, xsave(:,8,:), tsave, xsave(:,9,:), tsave, xsave(:,10,:), tsave, xsave(:,11,:), tsave, xsave(:,12,:));
xlabel('Time [sec]');
ylabel('Magnitude [rad, rad/s]'); % left y-axis 
% legend('\phi', '\theta', '\psi');
legend('\phi', '\theta', '\psi', 'p', 'q', 'r');
% axis([0 5 -0.2 1.0 ]), 
grid 

figure(3)
    plotyy(tsave, Usave(:,1,:), [tsave, tsave, tsave], [Usave(:,2,:), ...
        Usave(:,3,:), Usave(:,4,:)])
%     plot(tsave, Usave(:,1), tsave, Usave(:,2), tsave, Usave(:,3), ...
%         tsave, Usave(:,4));
xlabel('Time [sec]');
ylabel('Magnitude [N]'); % left y-axis 
legend('U_1', 'U_2', 'U_3', 'U_4');
%axis([0 5 -0.2 1.0 ]), 
grid

% figure(4)
% plot(tsave, Usave(:,5,:), tsave, Usave(:,6,:));
% xlabel('Time [sec]');
% ylabel('Magnitude [rad]'); % left y-axis 
% legend('\phi_d', '\theta_d');
% % axis([0 5 -0.2 1.0 ]), 
% grid 

clear tc tend h noIter Ix Iy Iz m g l JTP K R upd NumberEqt k xc dxdtmtr i j