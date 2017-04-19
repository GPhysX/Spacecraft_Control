commands = {[0 0 0]'; [70 70 70]'; [-70 -70 -70]';[0 0 0]'};
times = [9.99, 50, 90, 150];
fig = 1;

%% PD, quaternions
sc = spacecraft_q;
c = pd_q;
c.kp = 0.1*c.kp;
run(sc, c, commands, times, p, 1, fig);

%% PD, euler
sc = spacecraft;
sc.Ts = 0.1;
c = pd;
c.kp = 0.1*c.kp;
run(sc, c, commands, times, p, 1, fig);

%% NDI, quaternions, no timesep
sc = spacecraft_q;
% sc.Ts = 0.1;
c = pd_q;
c.kp = 2*pi * c.kp;
run(sc, c, commands, times, ndi_q, 1, fig);
%% NDI, euler, no timesep
sc = spacecraft;
sc.Ts = 0.1;
c = pd;
% c.kp = 2;
run(sc, c, commands, times, ndi, 1, fig);

%% NDI, quaternions, timesep
sc = spacecraft_q;
c = pd_q;
c.kp = 2*pi*c.kp;
sc.Ts = 0.01;
run(sc, c, commands, times, ndi_q, 10, fig);

%% NDI, euler angles, timesep
sc = spacecraft;
c = pd;
sc.Ts = 0.01;
run(sc, c, commands, times, ndi, 10, fig);

%% INDI, quaternions, timesep
sc = spacecraft_q;
sc.Ts = 0.01;
c = pd_q;
c.kp = 2*pi*c.kp;
run(sc, c, commands, times, indi_q, 10, fig);

%% INDI, euler angles, timesep
sc = spacecraft;
sc.Ts = 0.01;
c = pd;
% c.kp = c.kp*0.5;
[sc_, c_, indi_] = run(sc, c, commands, times, indi, 10, fig);