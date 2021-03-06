%% Main script calling all requested simulation cases
% Per case, the script creates a spacecraft opbject, two controller objects
% and simulates them based on the 'commands' and 'times' variables. Where
% needed, the default sample time (0.1) is adjusted and a time separation
% between the controllers is introduced. The simulations are performed
% using the 'run.m' function call.
commands = {[0 0 0]'; [70 70 70]'; [-70 -70 -70]';[0 0 0]'};
times = [9.99, 50, 90, 150];
fig = 1;

%% Linear, euler
run(spacecraft,linear, commands, times,p,1,fig);

%% Linear, quaternions
run(spacecraft_q,pd_q,commands,times,p,1,fig);


%% NDI, quaternions, no timesep
sc = spacecraft_q;
c = pd_q;
run(sc, c, commands, times, ndi_q, 1, fig);
%% NDI, euler, no timesep
sc = spacecraft;
c = pd;
run(sc, c, commands, times, ndi, 1, fig);

%% NDI, quaternions, timesep
sc = spacecraft_q;
c = pd_q;
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
run(sc, c, commands, times, indi_q, 10, fig);

%% INDI, euler angles, timesep
sc = spacecraft;
sc.Ts = 0.01;
c = pd;
run(sc, c, commands, times, indi, 10, fig);

%% NDI, quaternions, timesep, faulty internal model
sc = spacecraft;
c = pd;
sc.Ts = 0.01;
c2 = ndi;
c2.sc.J = 1.2 *sc.J;
c2.title = 'NDI_with_1_2J';
run(sc, c, commands, times, c2, 10, fig);

%% INDI, quaternions, timesep, faulty internal model
sc = spacecraft;
sc.Ts = 0.01;
c = pd;
c2 = indi;
c2.sc.J = 1.2 *sc.J;
c2.title = 'INDI_with_1_2J';
run(sc, c, commands, times, c2, 10, fig);
%%
close all;