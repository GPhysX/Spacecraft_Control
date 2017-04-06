function [time, thetas, cont] = run(s, c, commands, times, control_fcn, timesep)
    i = 0;
    cont = zeros(15000,3);
    thetas = zeros(15000,3);
    time = zeros(15000,1);
    s = spacecraft_q;
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
                c = c.control(s.x);            
            end
            s.Tc = control_fcn(s, c.v);
            cont(i,:) = s.Tc;
            s = s.sim;
            thetas(i,:) = s.theta';
            time(i) = s.t;
            if max(s.rot) > 1000
                disp 'unstable'
                return
            end
        end
    end

    figure
    plot(time(1:i), thetas(1:i,:))
    figure
    plot(time(1:i), e1(1:i,:))
    c.er
end