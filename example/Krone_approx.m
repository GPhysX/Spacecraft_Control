% function G = Krone_approx(r, N, w_L, w_H)
w_L = 1*10^(-2);
w_H = 10^1;
r = 1.5;
N =5;


zeros = -w_L * (w_H/w_L).^( (2*(1:N)-1-r) / (2*N) );
poles = -w_L * (w_H/w_L).^( (2*(1:N)-1+r) / (2*N) );
G = tf(zpk(zeros, poles, 1));
wmean = sqrt(w_L*w_H);
if isreal(r)
        G = G * abs((j*wmean)^r) / abs(squeeze(freqresp(G, wmean)));
else
        G = G * ((j*wmean)^r) / squeeze(freqresp(G, wmean));
end

H = c2d(G, 0.1);

% end