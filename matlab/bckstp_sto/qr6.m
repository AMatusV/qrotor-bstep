function dxdt = qr6(t,st)
%-------------------------------------------------
% Quadrotor mathematical model
% 6 degrees of freedom, Euler angles (roll, pitch, yaw)
%
% References: 
% - T. Bresciani, "Modelling, Identification and Control of a Quadrotor
% Helicopter," Master thesis, 2008.
% - J. Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and
% Rotation Vectors," Stanford University, 2006.
% - J. Colmenares-Vázquez et al., "Position Control of a Quadrotor under
% External Constant Disturbance," RED-UAS, 2015.
%
% Programmer: A. Matus-Vargas
% Date: 10 Oct 17
% 
% Inputs: time, current state vector
% Outputs: state vector time derivative
%------------------------------------------------

global Ix Iy Iz m g l JTP K R disturb
global upd  % Control law
% global ContModelo  % Number of callings to the quadrotor model

dxdt = zeros(13,1);
% upd = zeros(4,1);

x = st(1); y = st(2); z = st(3);
% xp = st(4); yp = st(5); zp = st(6);
phi = st(7); theta = st(8); psi = st(9); 
p = st(10); q = st(11); r = st(12); 

kp1 = K(1); kp2 = K(2); kp3 = K(3);    % Kp
kv1 = K(4); kv2 = K(5); kv3 = K(6);    % Kv
kn1 = K(7); kn2 = K(8); kn3 = K(9);    % Kn
ko1 = K(10); ko2 = K(11); ko3 = K(12); % Ko

Omega = 0; % must be the sum of the propellers rotational velocities in rad/s
kd = 0.1;  % mass normalized drag coefficient
zt = 1 + disturb(1); xt = 1 + disturb(2); yt = 1 + disturb(3); psit = 0; % Setpoints

dxdt(1:3) = st(4:6);
% U1 = kp1*(zt - z) - kd1*dxdt(3);
cx = m*((1 + kp1*kv1)*(xt - x) - (kp1 + kv1)*dxdt(1));
cy = m*((1 + kp2*kv2)*(yt - y) - (kp2 + kv2)*dxdt(2));
cz = m*((1 + kp3*kv3)*(zt - z) - (kp3 + kv3)*dxdt(3));
thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
U1 = (cz + m*g)/(cos(thetat)*cos(phit)); % fref
dxdt(4)   = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*U1/m - kd*st(4);
dxdt(5)   = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*U1/m - kd*st(5);
dxdt(6)   = cos(theta)*cos(phi)*U1/m - g - kd*st(6);
dxdt(7)   = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
dxdt(8)   = q*cos(phi) - r*sin(phi);
dxdt(9)   = (r*cos(phi) + q*sin(phi))/cos(theta);
U2 = disturb(4) + q*r*(Iz - Iy)*Ix + Ix*(-dxdt(7)*(kn1 + ko1) - (1 + kn1*ko1)*(phi - phit) + (dxdt(9)*(kn3 + ko1) + (1 + kn3*ko1)*(psi - psit))*sin(theta));
U3 = disturb(5) + p*r*(Ix - Iz)*Iy + Iy*((-(dxdt(8)*(kn2 + ko2) + (1 + kn2*ko2)*(theta - thetat)))*cos(phi) - (dxdt(9)*(kn3 + ko2) + (1 + kn3*ko2)*(psi - psit))*cos(theta)*sin(phi));
U4 = disturb(6) + p*q*(Iy - Ix)*Iz + Iz*((-(dxdt(9)*(kn3 + ko3) + (1 + kn3*ko3)*(psi - psit)))*cos(phi)*cos(theta) + (dxdt(8)*(kn2 + ko3) + (1 + kn2*ko3)*(theta - thetat))*sin(phi));
dxdt(10) = (U2 + q*r*(Iy - Iz) - JTP*q*Omega)/Ix; % ppunto
dxdt(11) = (U3 + p*r*(Iz - Ix) + JTP*p*Omega)/Iy; % qpunto
dxdt(12) = (U4 + p*q*(Ix - Iy))/Iz; % rpunto
dxdt(13) = (xt - x)^2 + (yt - y)^2 + (zt - z)^2 + ...
    (phit - phi)^2 + (thetat - theta)^2 + (psit - psi)^2 + ...
    + R*K'*K;

upd = [U1; U2; U3; U4];
% ContModelo = ContModelo + 1;
   
return

end
