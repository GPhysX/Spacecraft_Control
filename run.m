function [sc, c, c2, time] = run(sc, c, commands, times, c2, timesep, figureflg)
    i = 0;
    k = 0;
    cont = zeros(15000,3);
    thetas = zeros(15000,3);
    time = zeros(15000,1);
    q_s = zeros(15000,4);
    e1 = zeros(15000,3);
    e2 = zeros(15000,3);
    
    if timesep
        ts = timesep;
    else
        ts = 1;
    end
    
    for j = 1:length(times)
        c = c.setref_d(commands{j});
        while sc.t < times(j)
            i = i+1;        
            if mod(i,ts) == 0
                e1(i,:) = c.v - sc.rot;
                i;
                sc.theta * 180 / pi;
                c = c.control(sc.x, sc);            
            end
            k = k + 1;
            e2(k,:) = 180/pi*sc.theta'- [1 1 1]*commands{j};
            c2 = c2.control(sc, c.v);
            sc.Tc = c2.u;
            cont(i,:) = sc.Tc;
            sc = sc.step;
            thetas(i,:) = 180/pi*sc.theta';
            if isprop(sc,'q')
                q_s(i,:) = sc.q';
            end
            time(i) = sc.t;
            if abs(max(sc.rot)) > 2*pi/sc.Ts              % > one revolution/simulation step
                disp 'unstable'
                return
            end
        end
    end
    title1 = string(strcat({'Response of spacecraft in '}, sc.title, {' with '}, c2.title, {' controller.'}));
    title2 = strcat({'Ts = '},  string(sc.Ts), {'; time seperation factor = '}, string(ts));

    if figureflg
        figure
        plot(time(1:i), thetas(1:i,:))
        title({title1;title2});
        ylabel('Angle (degrees)');
        xlabel('Time (seconds)');
        legend('theta1', 'theta2', 'theta3');
        saveas(gcf,strcat('./report/img/', c2.title, '_', int2str(timesep), sc.title, '.png'))
        if isprop(sc,'q')
            figure
            plot(time(1:i), q_s(1:i,:))
            title({title1;title2});
            ylabel('Quaternion');
            xlabel('Time (seconds)');
            legend('q1', 'q2', 'q3', 'q4', 'ref1', 'ref2', 'ref3', 'ref4');
            saveas(gcf,strcat('./report/img/','q_', c2.title, '_', int2str(timesep), sc.title, '.png'))
        end
    else
        title1
        title2
        error = sum(sum(abs(e2)))
        control_in = sum(sum(abs(cont)))
    end
%     figure
%     plot(time(1:i), c2.us(1:i,:))
    

end