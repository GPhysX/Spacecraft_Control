function G = Krone_approx(r, N, w_L, w_H)
    zeros = -w_L * (w_H/w_L).^( (2*(1:N)-1-r) / (2*N) );
    poles = -w_L * (w_H/w_L).^( (2*(1:N)-1+r) / (2*N) );
    G = tf(zpk(zeros, poles, 1));
    wmean = sqrt(w_L*w_H);
    if isreal(r)
        G = G * abs((j*wmean)^r) / abs(squeeze(freqresp(G, wmean)));
    else
        G = G * ((j*wmean)^r) / squeeze(freqresp(G, wmean));
    end
end