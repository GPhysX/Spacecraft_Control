function [sc, c, c2, time, thetas, q_s] = run(sc, c, commands, times, c2, timesep, figureflg)
    % This function combines a spacecraft model, an inner loop and an
    % outer loop controller to simulate the spacecraft subject to the
    % attitude commands. The states are returned, plotted and interpreted
    % depending on the usage of the 'figure' flag. The integer input 'timesep' allow the inner
    % and outer loop controller to run at different rates. 
    i = 0;
    k = 0;
    n = max(times)/sc.Ts;
    cont = zeros(n,3);
    thetas = zeros(n,3);
    time = zeros(n,1);
    q_s = zeros(n,4);
    e = zeros(n,3);
    % Make sure sample times are similar
    if isprop(c2, 'Ts')
        c2.Ts = sc.Ts;
    end
    if timesep
        ts = timesep;
    else
        ts = 1;
    end
    
    for j = 1:length(times)
        c = c.setref_d(commands{j});
        while sc.t < times(j)
            i = i+1;
            % Outer loop -----------------------
            if mod(i,ts) == 0
                %find the virtual control
                c = c.control(sc.x);            
            end % ------------------------------

            %find the control torque
            c2 = c2.control(sc.x, c.v);            
            sc.Tc = c2.u;
            cont(i,:) = sc.Tc;
            %propagate the simulation
            sc = sc.step;
            % update history
            e(i,:) = (180/pi*sc.theta'- [1 1 1]*commands{j});
            thetas(i,:) = 180/pi*sc.theta';
            if isprop(sc,'q')
                q_s(i,:) = sc.q';
            end
            time(i) = sc.t;
            if max(abs(sc.rot)) > 2*pi/sc.Ts              % > one revolution/simulation step
                disp 'unstable'
                return
            end
        end
    end
    %-----------------------PLOTS----------------------------------------
    title1 = string(strcat({'Response of spacecraft in '}, sc.title));
    title2 = string(strcat({' with '}, c2.title, {' controller.'}));
    title3 = strcat({'Ts = '},  string(sc.Ts), {'; time seperation factor = '}, string(ts));

    if figureflg
        figure
        plot(time(1:i), thetas(1:i,:))
        title({title1;title2;title3},'Interpreter', 'none');
        ylabel('Angle (degrees)');
        xlabel('Time (seconds)');
        legend('theta1', 'theta2', 'theta3');
        saveas(gcf,strcat('./report/img/', c2.title, '_', int2str(timesep), sc.title, '.png'))
        if isprop(sc,'q')
            figure
            plot(time(1:i), q_s(1:i,:))
            title({title1;title2;title3},'Interpreter', 'none');
            ylabel('Quaternion');
            xlabel('Time (seconds)');
            legend('q1', 'q2', 'q3', 'q4');
            saveas(gcf,strcat('./report/img/','q_', c2.title, '_', int2str(timesep), sc.title, '.png'))
        end
    % Evaluate score  
    else
        title1
        title2
        error = sum(sum(abs(e*sc.Ts)))
        control_in = sum(sum(abs(cont*sc.Ts)))
        score = 1.e5/(error + control_in)
    end
end