%% Linear (pd) controller design
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

figure
hold on
% sisotool(E,G);
% bode(A)
% bode(B)
% bode(D)
% bode(E)
% margin(D*G)
% margin(E*G)
% margin(F*G)
hold off

%% Quaternions
% figure
% sisotool(C)
Gq = 2*(s+0.1);
H = c2d(B*Gq, 0.1);
margin(0.5*H)


% 
% %% 
% G2 = Krone_approx(1, 4, 0.01, 10);
% % margin(C* s*G2/(0.1s +1)
% %%
% 
% H = c2d(G2*exp(-.4*s), 0.1);
% % sisotool(E,H);
% margin(E*.5*H)