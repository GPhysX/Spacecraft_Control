s = spacecraft_q;
thetas = zeros(15000,3);
time = zeros(15000,1);
c = pd_q;
i = 0;
cont = zeros(15000,3);
commands = {[0 0 0]'; [70 70 70]'; [-70 -70 -70]';[0 0 0]'};
times = 1*[9.99, 50, 90, 150];


for j = 1:length(times)
    c = c.setref_d(commands{j});
    while s.t < times(j)
        i = i+1;        
        if mod(i,1) == 0
            i
            c = c.control(s.x);            
        end
        s.Tc = s.J*c.v + cross(s.rot,s.J*s.rot) - s.Td;         %ndi
        cont(i,:) = s.Tc;
        s = s.sim;
        thetas(i,:) = s.theta';
        time(i) = s.t;
    end
end

figure
plot(time(1:i), thetas(1:i,:))
c.er