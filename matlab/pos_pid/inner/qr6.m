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

global Ix Iy Iz m g l JTP K R numberEqs
global upd  % Control law
global ContModelo  % Number of callings to the quadrotor model

dxdt = zeros(numberEqs,1);
upd = zeros(4,1);

x = st(1); y = st(2); z = st(3);
% xp = st(4); yp = st(5); zp = st(6);
phi = st(7); theta = st(8); psi = st(9); 
p = st(10); q = st(11); r = st(12); 

kp_roll  = K(1); ki_roll  = K(2); kd_roll = K(3);
kp_pitch = K(4); ki_pitch = K(5); kd_pitch = K(6);
kp_yaw   = K(7); ki_yaw   = K(8); kd_yaw = K(9);

Omega = 0; % must be the sum of the propellers rotational velocities in rad/s
kd = 0.1;  % Viscous friction
phit = 0; thetat = 0; psit = 0; % Setpoints

dxdt(1:3) = st(4:6);
% U1 = kp1*(zt - z) - kd1*dxdt(3);
% cx = kpx*(xt - x) + kix*st(14) - kdx*dxdt(1);
% cy = kpy*(yt - y) + kiy*st(15) - kdy*dxdt(2);
% cz = kpz*(zt - z) + kiz*st(16) - kdz*dxdt(3);
% thetat = atan2(cy*sin(psi) + cx*cos(psi), cz + m*g);
% phit = atan2((cx*sin(psi) - cy*cos(psi))*cos(thetat), cz + m*g);
% U1 = (cz + m*g)/(cos(thetat)*cos(phit)); % fref
U1 = m*g; % fref
dxdt(4) = (sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*U1/m - kd*st(4);
dxdt(5) = (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*U1/m  - kd*st(5);
dxdt(6) = cos(theta)*cos(phi)*U1/m - g - kd*st(6);
dxdt(7) = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
dxdt(8) = q*cos(phi) - r*sin(phi);
dxdt(9) = (r*cos(phi) + q*sin(phi))/cos(theta);
U2 = kp_roll*(phit - phi) + ki_roll*st(14) - kd_roll*dxdt(7);
U3 = kp_pitch*(thetat - theta) + ki_pitch*st(15) - kd_pitch*dxdt(8);
U4 = kp_yaw*(psit - psi) + ki_yaw*st(16) - kd_yaw*dxdt(9);
dxdt(10) = (U2 + q*r*(Iy - Iz) - JTP*q*Omega)/Ix; % ppunto
dxdt(11) = (U3 + p*r*(Iz - Ix) + JTP*p*Omega)/Iy; % qpunto
dxdt(12) = (U4 + p*q*(Ix - Iy))/Iz; % rpunto
%dxdt(13) = (xt - x)^2 + (yt - y)^2 + (zt - z)^2 + ...
dxdt(13) = (phit - phi)^2 + (thetat - theta)^2 + (psit - psi)^2 + ...
    R*(K'*K);  %+ R*(U1^2 + U2^2 + U3^2 + U4^2);
dxdt(14) = phit - phi;
dxdt(15) = thetat - theta;
dxdt(16) = psit - psi;

upd = [U1; U2; U3; U4];
ContModelo = ContModelo + 1;
   
return

end
