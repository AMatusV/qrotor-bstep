function stoSignals = RandNumGen(tend, noSamples)
% Stochastic signals generator for quadrotor
%
% Inputs: final time, time step
% Outpts: four stochastic signals
%--------------------------------------------------------------------------

t = linspace(0, tend, noSamples); % Number of samples

m = size(t);                % Secuencias de numeros aleatorios
Rx = 0 + sqrt(3)*randn(m);  % Numeros aleatorios media = 0, var = 3                                   
Ry = 0 + sqrt(3)*randn(m);                               
Rz = 0 + sqrt(3)*randn(m);
R1 = 0 + sqrt(0.1)*randn(m);
R2 = 0 + sqrt(0.1)*randn(m);
R3 = 0 + sqrt(0.1)*randn(m);
                               
zeta = 0.7071;       % Factor de amortiguamiento 
frecc_natural = 0.2; % Freccuencia natural del cuadrotor 11.5948
fd = 1;

wr = tf([0 0 frecc_natural^2], ...
    [1 2*zeta*frecc_natural frecc_natural^2]); %Second order transfer function
stoX = lsim(wr, Rx, t); % The stochastic signals is passed through the s.o. filter 
stoY = lsim(wr, Ry, t);
stoZ = lsim(wr, Rz, t);

w1 = tf([0 fd], [1 fd]); % First order transfer function
stoS1 = lsim(w1, R1, t);

stoS2 = lsim(w1, R2, t);
             
stoS3 = lsim(w1, R3, t); % The stochastic signals is passed through the f.o. filter       


stoSignals = [stoX, stoY, stoZ, stoS1, stoS2, stoS3];

end