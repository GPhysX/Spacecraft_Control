s = spacecraft;
s = s.update_x([70*pi/180 70*pi/180 70*pi/180 0 0 0]')
x0 = s.x;
s.Td = [0 0 0]';
Gamma = s.G;
Phi = zeros(length(s.x),length(s.x));
for i  = 1:length(s.x)
    x = x0;
    x(i) = x(i) + 0.01;
    s = s.update_x(x);
    Phi(:,i) = s.x_dot * 100;
end
Phi
% Gamma

sys = ss(Phi, Gamma(:,1), [eye(1) zeros(1,5)], 0, 0.1);
