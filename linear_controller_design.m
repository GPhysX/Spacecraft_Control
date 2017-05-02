%% Linear (pd) controller design
% Script for the design of the linear outer loop controllers.
hold on
s = tf('s');
z = tf('z',0.1);
A = 1/s^2;                      % Ideal model
B = exp(-0.02*s)*1/s^2;         % Model of the ideal system with slight time delay
C = exp(-0.1*s)*1/s^2;          % Model with maximum time delay

D = c2d(A,0.1);                 % Zero order hold representation of the sampled system
E = c2d(B,0.1);
F = c2d(C,0.1);
% sisotool(F);

G = 10 * (z-0.99) * z^(-1);

% sisotool(E,G);
% bode(D)
% bode(E)
% bode(E2)
margin(D*G)
margin(E*G)
margin(F*G)
hold off

%% Quaternions
figure
% sisotool(C)
Gq = (3*s+1.5);
H = c2d(A*Gq, 0.1);
margin(H)

