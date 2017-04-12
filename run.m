function [time, thetas, cont, c] = run(s, c, commands, times, c2, timesep, figureflg)
    i = 0;
    k = 0;
    cont = zeros(15000,3);
    thetas = zeros(15000,3);
    time = zeros(15000,1);
    e1 = zeros(15000,3);
    e2 = zeros(15000,3);
    
    if timesep
        ts = timesep;
    else
        ts = 1;
    end
    
    for j = 1:length(times)
        c = c.setref_d(commands{j});
        while s.t < times(j)
            i = i+1;        
            if mod(i,ts) == 0
                e1(i,:) = c.v - s.rot;
                i;
                s.theta * 180 / pi;
                c = c.control(s.x);            
            end
            k = k + 1;
            e2(k,:) = 180/pi*s.theta'- [1 1 1]*commands{j};
            c2 = c2.control(s, c.v);
            s.Tc = c2.u;
            cont(i,:) = s.Tc;
            s = s.sim;
            thetas(i,:) = 180/pi*s.theta';
            time(i) = s.t;
            if abs(max(s.rot)) > 2*pi/s.dt              % > one revolution/simulation step
                disp 'unstable'
                return
            end
        end
    end
    title1 = string(strcat({'Response of spacecraft in '}, s.title, {' with '}, c2.title, {' controller.'}));
    title2 = strcat({'dt = '},  string(s.dt), {'; time seperation factor = '}, string(ts));

    if figureflg
        figure
        plot(time(1:i), thetas(1:i,:))
        title({title1;title2});
        xlabel('Angle (degrees)');
        ylabel('Time (seconds)');
        legend('theta1', 'theta2', 'theta3');
    else
        title1
        title2
        error = sum(sum(abs(e2)))
        control_in = sum(sum(abs(cont)))
    end
%     figure
%     plot(time(1:i), c2.us(1:i,:))
    

end