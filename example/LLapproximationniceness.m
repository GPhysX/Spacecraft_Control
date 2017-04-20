clc;
clear; 
close all;

f = logspace(-1,4,10000);
j = sqrt(-1);
s = j*f;

wi = 2*pi*14.15;
wz = 2*pi*14.15;
wp = 2*pi*240.46;
wlp = 2*pi*412.23;

LL = (s.^1.2/wz^1.2 + 1)./(s.^1.2/wp^1.2 + 1);
LP = 1./((s/wlp).^1.62 + (s/wlp).^0.81 + 1)./(s/wlp+1);

LLf = (24.29 * s.^3 + 3760 * s.^2 + 2.818e05 * s + 1.418e06)./...
     (s.^3 + 1495 * s.^2 + 2.76e05 * s + 1.418e06);
% LLf =   (27.12 *s.^3 + 5462 *s.^2 + 4.579e05 *s  + 2.968e06)./...
%         (s.^3 + 1544 * s.^2 + 4.456e05 * s + 2.967e06);
LPf =    (200.7 * s.^9 + 4.273e06 * s.^8 + 3.097e10 * s.^7 + 8.083e13 * s.^6 + 3.291e16 * s.^5 + 4.925e18 * s.^4 + 2.707e20 * s.^3 + 2.322e21 * s.^2 + 7.173e21 * s + 7.542e21)./...                                                                                                          
         (s.^10 + 1.294e04 * s.^9 + 5.587e07 * s.^8 + 1.148e11 * s.^7 + 1.166e14 * s.^6 + 3.877e16 * s.^5 + 5.28e18 * s.^4 + 2.745e20 * s.^3  + 2.336e21 * s.^2 + 7.191e21 * s + 7.544e21);
C = LL.*LP;     
Cf = LLf.*LPf;
%%
    figure; 
% subplot(211),semilogx(f,20*log10(abs(LL)),'linewidth',2), grid on, hold on
% subplot(212),semilogx(f,180*(phase(LL))/pi,'linewidth',2), grid on, hold on
% 
% subplot(211),semilogx(f,20*log10(abs(LLf)),'linewidth',2), grid on, hold on
% subplot(212),semilogx(f,180*(phase(LLf))/pi,'linewidth',2), grid on, hold on

% subplot(211),semilogx(f,20*log10(abs(LP)),'linewidth',2), grid on, hold on
% subplot(212),semilogx(f,180*(phase(LP))/pi,'linewidth',2), grid on, hold on
% 
% subplot(211),semilogx(f,20*log10(abs(LPf)),'linewidth',2), grid on, hold on
% subplot(212),semilogx(f,180*(phase(LPf))/pi,'linewidth',2), grid on, hold on

subplot(211),semilogx(f,20*log10(abs(C)),'k--','linewidth',2), grid on, hold on
subplot(212),semilogx(f,180*(phase(C))/pi,'k--','linewidth',2), grid on, hold on

subplot(211),semilogx(f,20*log10(abs(Cf)),'k','linewidth',2), grid on, hold on
ylabel('Magnitude (dB)'); axis([10 10^4 -3 30]);
subplot(212),semilogx(f,180*(phase(Cf))/pi,'k','linewidth',2), grid on, hold on
xlabel('Frequency (Hz)'); ylabel('Phase (deg)'); axis([10 10^4 -150 90]);
legend('Fractional controller','Approximated controller');
