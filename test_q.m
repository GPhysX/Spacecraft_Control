commands = {[0 0 0]'; [70 70 70]'; [-70 -70 -70]';[0 0 0]'};
times = 1*[9.99, 50, 90, 150];
fig = 1;

%% PD, quaternions
s = spacecraft_q;
c = pd_q;
c.kp = 5 * c.kp;
run(s, c, commands, times, p, 1, fig);

%% PD, euler
s = spacecraft;
s.dt = 0.01;
c = pd;
c.kp = 2 * c.kp;
run(s, c, commands, times, p, 10, fig);

%% NDI, quaternions, no timesep
s = spacecraft_q;
s.dt = 0.01;
c = pd_q;
c.kp = 10 * c.kp;
run(s, c, commands, times, ndi_q, 10, fig);
%% NDI, euler, no timesep
s = spacecraft;
s.dt = 0.1;
c = pd;
c.kp = 3*c.kp;
run(s, c, commands, times, ndi, 1, fig);

%% NDI, quaternions, timesep
s = spacecraft_q;
c = pd_q;
c.kp = .4*c.kp;
run(s, c, commands, 2*times, ndi_q, 10, fig);

%% NDI, euler angles, timesep
s = spacecraft;
c = pd;
c.kp = .05*c.kp;
run(s, c, commands, 2*times, ndi, 10, fig);

%% INDI, quaternions, timesep
s = spacecraft_q;
c = pd_q;
c.kp = .3 * c.kp;
run(s, c, commands, 3*times, indi_q, 10, fig);


%% INDI, euler angles, timesep
s = spacecraft;
c = pd;
c.kp = .05 * c.kp;
run(s, c, commands, 3*times, indi, 10, fig);
