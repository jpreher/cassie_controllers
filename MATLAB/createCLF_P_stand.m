%% Function: createCLF_P_stand.m
%
% Description: Tune the CLF, this function prints the assosciated diagonal and off-diagonal terms of P.
% 	The resulting gains can be copy-pasted to the controller launch file.
%
% Author: Jenna Reher, (jreher@caltech.edu)
% _____________________________________________________________________

ny = 6;

% Compute eta and the Lyapunov function
F = [zeros(ny),eye(ny);
     zeros(ny),zeros(ny)];
G = [zeros(ny);eye(ny)];
Q = eye(2*ny);

% Tune Q
% Current best
Q(1:2, 1:2) = Q(1:2, 1:2) * 8000; % Px Py
Q(3,3)      = Q(3,3)      * 1000; % Pz
Q(4:5, 4:5) = Q(4:5, 4:5) * 250; % Roll, Pitch
Q(6,6)      = Q(6,6)      * 100; % Yaw

Q(7:8,7:8)     = Q(7:8,7:8)     * 15; % Px Py
Q(9,9)         = Q(9,9)         * 10; % Pz
Q(10:11,10:11) = Q(10:11,10:11) * 8; % Roll, Pitch
Q(12,12)       = Q(12,12)       * 6; % Yaw


%% Compute
[P,~,~] = care(F,G,Q);
P
dP = diag(P);
dPstr = num2str(dP(1));
for i = 2:length(dP)
    dPstr = strcat(dPstr, ', ', num2str(dP(i)));
end
disp(dPstr);

dPo = diag(P(1:ny,ny+1:end));
dPostr = num2str(dPo(1));
for j = 2:length(dPo)
    dPostr = strcat(dPostr, ', ', num2str(dPo(j)));
end
disp(dPostr);


gam = min(eig(Q))/ max(eig(P))
