function [time, thetas, cont, c] = run(s, c, commands, times, c2, timesep, figureflg)
    i = 0;
    cont = zeros(15000,3);
    thetas = zeros(15000,3);
    time = zeros(15000,1);
    e1 = zeros(15000,3);
    
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
                i
                s.theta * 180 / pi;
                c = c.control(s.x);            
            end
            c2 = c2.control(s, c.v);
            s.Tc = c2.u;
            cont(i,:) = s.Tc;
            s = s.sim;
            thetas(i,:) = 180/pi*s.theta';
            time(i) = s.t;
            if abs(max(s.rot)) > 100
                disp 'unstable'
                return
            end
        end
    end
    if figureflg
        figure
        plot(time(1:i), thetas(1:i,:))
    end
%     figure
%     plot(time(1:i), e1(1:i,:))
%     sum(c.er)
end