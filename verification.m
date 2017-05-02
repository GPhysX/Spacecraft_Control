%% Verification
% This will test the spacecraft model subject to a number of initial
% conditions and disturbances.

%% Test for rotation around orbit
% initialization: disturbance are set to 0, dt is increased to speed up the step
s = spacecraft;
s.Td = [0 0 0]';
s.Ts = 1.25;
% simulate until 1/8th an orbit period has passed (to stay out of any singularities).
while s.t < s.orbit_period/8
    s = s.step;
end
% Test whether the rotation matches the expectation
if s.x_(2) > pi/4
    disp 'No initial rotation'
    s.theta
    disp 'succes'
end

%% initial rotation around z
s = spacecraft;
s.Td = [0 0 0]';
s = s.update_x([0 0 pi/2 0 0 0]');
s.Ts = 10;
while s.t < s.orbit_period/8
    s = s.step;
end
if s.x_(1) > pi/4
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
    s = s.step;
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
    s = s.step;
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
    s = s.step;
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
    s = s.step;
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
    s = s.step;
end
if s.x_(4) > 9.9
    disp 'Single axis disturbance'
    s.rot
    disp 'succes'
end