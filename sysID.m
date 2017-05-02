% System identification script to check the gain of the different
% controlled systems.

commands = {[0 0 0]'};
times = [500];
fig = 0;


%%
[sc, c, c2, time, thetas] = run(spacecraft_q, wn, commands, times, ndi_q, 1, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%%
sc = spacecraft_q;
sc.Ts= 0.01;
[sc, c, c2, time, thetas] = run(sc, wn, commands, times, ndi_q, 10, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%%
sc = spacecraft_q;
sc.Ts= 0.01;
[sc, c, c2, time, thetas, q_s] = run(sc, wn, commands, times, indi_q, 10, fig);
[x,y] = pwelch(q_s);
figure
loglog(y,x)

%%
sc = spacecraft_q;
[sc, c, c2, time, thetas] = run(sc, wn, commands, times, p, 10, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%%
sc = spacecraft;
sc.Ts= 0.01;
[sc, c, c2, time, thetas, q_s] = run(sc, wn, commands, times, indi, 10, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

