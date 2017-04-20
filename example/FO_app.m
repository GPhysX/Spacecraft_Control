clear; clc; close all;
P = bodeoptions;
P.FreqUnits = 'Hz';

%% Demo

w_L = 1*10^-1;
w_H = 10^4;
N = 3;              % Output order =  2N+1
r = 1.5;              % Approximated order s^r
G3 = Krone_approx(r,N,w_L,w_H)

N = 10;              % Output order =  2N+1
G5 = Krone_approx(r,N,w_L,w_H)

figure; hold on;
bodeplot(tf([1 0],[1]),P,'k--');
bodeplot(tf([1 0 0],[1]),P,'k:');
bodeplot(G3,'g--',P);
bodeplot(G5,'g',P);
legend('s^1','s^2','s^{1.5} , N=3','s^{1.5} , N = 10');
%% Outer loop design
% G0 = tf([1 0],[1]);
% w_L = 1*10^-1;
% w_H = 5*10^3;
% N = 3;              % Output order =  2N+1
% r = 1.5;              % Approximated order s^r
% Gm = Krone_approx(r,N,w_L,w_H)
% 
% Gd = Krone_approx(0.7,N,w_L,w_H)

% LC1 = (Gf/(2*pi*60/10)^1.7 + 1)*tf([1],[1 0])
% LCf = minreal((Gf/(2*pi*45/8)^0.8 + 1)/(Gf/(2*pi*45*8)^0.8 + 1));
% Gw = minreal(1/(Gm/(17*2*pi)^1.6 + Gd/300))
% figure; hold on;
% bodeplot(Gw,P);
% load Gx;
% bodeplot(TFx2,P);