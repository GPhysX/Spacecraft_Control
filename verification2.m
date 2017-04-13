%% Test for single rotation around orbit
% initialization: disturbance are set to 0, dt is increased to speed up the sim
s = spacecraft_q;
s.Td = [0 0 0]';
s.Ts = 1.25;
% simulate until 1/8th an orbit period has passed.
while s.t < s.orbit_period/8
    s = s.sim;
end
% Test whether the rotation matches the expectation
if s.theta_(2) > pi/4
    disp 'No initial rotation'
    disp 'succes'
end

%% initial rotation around z
s = spacecraft_q;
s.Td = [0 0 0]';
s = s.set_orient_r([0 0 pi/2]');
s.Ts = 10;
while s.t < s.orbit_period/8
    s = s.sim;
end
if s.theta_(1) > pi/4
    disp 'Initial rotation around z'
    disp 'succes'
end

%% Initial velocity
sq = spacecraft_q;
s = spacecraft;
s.Td = [0 0 0]';
sq.Td = [0 0 0]';
s = s.update_x([0 0 0 1 0 0]');
sq = s.update_x([0 0 0 1 1 0 0]');
while s.t < pi
    s = s.sim;
    sq = sq.sim;
end

if max(abs(sq.theta - s.theta)) < 1.e-5
    disp 'Initial rate around x'
    disp 'succes'
end

%% Initial velocity
sq = spacecraft_q;
s = spacecraft;
s.Td = [0 0 0]';
sq.Td = [0 0 0]';
s = s.update_x([0 0 0 0 1 0]');
sq = s.update_x([0 0 0 1 0 1 0 ]');
while s.t < 10
    s = s.sim;
    sq = sq.sim;
end

if max(abs(sq.theta - s.theta)) < 1.e-5
    disp 'Initial rate around all axes'
    disp 'succes'
end

%% Disturbance
sq = spacecraft_q;
s = spacecraft;
s.Td = [1 1 1]';
sq.Td = [1 1 1]';
while s.t < 10
    s = s.sim;
    sq = sq.sim;
end
if max(abs(sq.theta - s.theta)) < 1.e-5
    disp 'Disturbance around all axes'
    disp 'succes'
end


