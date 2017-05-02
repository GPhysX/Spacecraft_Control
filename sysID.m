% System identification script to check the gain of the different
% controlled systems. A white noise is introduced to the outer loop of the
% system

commands = {[0 0 0]'};
times = [500];
fig = 0;


%% quaternion, ndi, no timesep
[sc, c, c2, time, thetas] = run(spacecraft_q, wn, commands, times, ndi_q, 1, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%% quaternion, ndi, timesep
sc = spacecraft_q;
sc.Ts= 0.01;
[sc, c, c2, time, thetas] = run(sc, wn, commands, times, ndi_q, 10, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%% quaternion, indi, timesep
sc = spacecraft_q;
sc.Ts= 0.01;
[sc, c, c2, time, thetas, q_s] = run(sc, wn, commands, times, indi_q, 10, fig);
[x,y] = pwelch(q_s);
figure
loglog(y,x)

%% quaternion, scaled by J
sc = spacecraft_q;
[sc, c, c2, time, thetas] = run(sc, wn, commands, times, p, 1, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)

%% euler angles, indi, timesep
sc = spacecraft;
sc.Ts= 0.01;
[sc, c, c2, time, thetas, q_s] = run(sc, wn, commands, times, indi, 10, fig);
[x,y] = pwelch(thetas);
figure
loglog(y,x)