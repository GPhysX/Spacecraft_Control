s = spacecraft;
for i = 1:1:100
    s = s.update_x(s.x + s.x_dot);
    plot(s.theta)
end
s.x