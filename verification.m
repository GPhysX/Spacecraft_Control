%% Verification
% This will test the spacecraft model subject to a number of simple initial
% conditions and disturbances.

%% Test for single rotation around orbit
% initialization: disturbance are set to 0, dt is increased to speed up the sim
s = spacecraft;
s.Td = [0 0 0]';
s.Ts = 2.5;
% simulate until an entire orbital period has passed.
while s.t < 5917.5
    s = s.sim;
end
% Test whether the rotation matches the expectation
if s.x_(2) > 2*pi
    disp 'No initial rotation'
    s.theta
    disp 'succes'
end

%% initial rotation around z
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 pi/2 0 0 0]');
s.Ts = 10;
while s.t < 3000
    s = s.sim;
end
if s.x_(1) > pi
    disp 'Initial rotation around z'
    s.theta
    disp 'succes'
end

%% Initial velocity
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 0 1 0 0]');
s.Ts = .1;
while s.t < pi
    s = s.sim;
end
if s.x_(1) > pi
    disp 'Initial rate around x'
    s.theta
    disp 'succes'
end

%% around y
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 0 0 1 0]');
s.Ts = .1;
while s.t < pi
    s = s.sim;
end
if s.x_(2) > pi
    disp 'Initial rate around y'
    s.theta
    disp 'succes'
end

%% around z
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 0 0 0 1]');
s.Ts = .1;
while s.t < pi
    s = s.sim;
end
if s.x_(3) > pi
    disp 'Initial rate around z'
    s.theta
    disp 'succes'
end

%% precession >> takes rather long to run
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 0 1 1 1]');
s.Ts = 1;
while s.t < 1000
    s = s.sim;
end
%energy lost:
if abs(sum(s.J * s.rot.^2)-sum(s.J*[1 1 1]'.^2)) < 1.e-5
    disp 'Energy lost ~ 0'
    disp 'succes'
else
    disp 'Energy lost > 0'
    disp 'failure'
end

sum(s.J * s.rot.^2)-sum(s.J*[1 1 1]'.^2)
%% disturbance
s = spacecraft;
s.Td = [s.J(1,1) 0 0]';
while s.t < 9.99
    s = s.sim;
end
if s.x_(4) > 9.9
    disp 'Single axis disturbance'
    s.rot
    disp 'succes'
end