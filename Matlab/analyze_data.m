function [left_deadzone, right_deadzone] = analyze_data(name)

    pi = 3.1415926535;
    diameter = 0.121; %m
    CPR = 1632.67;

    table = readtable(name);
    data = table2array(table);

    [range, temp] = size(data);

    % Calculo do periodo medio com media movel
    ts = data(1, 1);
    for i = 2:range
        ts = (ts*(i-1)+(data(i,1)-data(i-1,1)))/i;
    end
    
    % Conversao das leituras dos encoders para metro
    for i = 1:(range)
        data(i, 3) = abs( data(i, 3)*pi*diameter/CPR );
        data(i, 4) = abs( data(i, 4)*pi*diameter/CPR );
    end

    % % Inicio da velocidade
    % vel_esq = zeros(range, 1);
    % vel_dir = zeros(range, 1);
    % 
    % % Calculo 'simples'
    % for i = 1:(range-1)
    %     vel_esq(i) = (data(i+1,3) - data(i, 3))/(data(i+1,1) - data(i, 1));
    %     vel_dir(i) = (data(i+1,4) - data(i, 4))/(data(i+1,1) - data(i, 1));
    % end
    % vel_esq(range) = vel_esq(range-1);
    % vel_dir(range) = vel_dir(range-1);

    % Inicio da velocidade
    vel_esq = zeros(range, 1);
    vel_dir = zeros(range, 1);

    vel_esq(1) = 0;
    vel_dir(1) = 0;
    % Calculo com derivada seguida de um filtro passa-baixas e ZOH
    for i = 2:range
        vel_esq(i) = 30*data(i,3) - 30*data(i-1,3) + exp(-30*ts)*vel_esq(i-1);
        vel_dir(i) = 30*data(i,4) - 30*data(i-1,4) + exp(-30*ts)*vel_dir(i-1);
    end

    % % Plot input
    % yyaxis left
    % plot(data(1:range, 1), data(1:range, 2), '-b');
    % hold on
    % 
    % % Plot speed
    % plot(data(1:range, 1), vel_esq(1:range), '-r');
    % plot(data(1:range, 1), vel_dir(1:range), '-g');
    % 
    % % Plot encoder readings in meter
    % yyaxis right
    % plot(data(1:range, 1), data(1:range, 3), '-y');
    % plot(data(1:range, 1), data(1:range, 4), '-m');

    % Start of dead-zone calculations
    time = data(1:range, 1);
    input = data(1:range, 2);

    % Window size
    window = 10;

    % Accepted error
    error = 0.02;

    for i = 1:2
        if i == 1
            output = vel_esq;
        else
            output = vel_dir;
        end

        mean = sum(output(1:window))/window;
        % First state is 0, leaving dead-zone and going positive
        state = 0;

        % Array of dead-zone boundaries, depending of the signal it may be good to
        % increase its size
        limits = zeros(10, 1);
        index = 1;

        % Used to save the last position of dead-zone boundary and avoid getting
        % data too close to each other
        % The condition (n-window-last) > 200 avoids this ^^^
        last = 0;

        for n = (window+1):size(time)
            % Start in position (window+1) to update mean
            mean = mean + (output(n) - output(n-window))/window;
            if state == 0
                if abs(mean) >= error && (n-window-last) > 200
                    % Just left dead-zone, save the index 'n-window' and change state
                    last = n-window;
                    limits(index, 1) = last;
                    index = index + 1;
                    state = 1;
                end
            elseif state == 1
                if abs(mean) <= error && (n-window-last) > 200
                    % Just got back from the non dead-zone values to the dead-zone
%                     last = n-window;
%                     limits(index, 1) = last;
%                     index = index + 1;
                    state = 0;
                end
            end    
        end

        % Positive limit
        pos_limit = 0;
        n_pos = 0;

        % Adaptative mean of the values of input signal
        for n = 1:(index-1)
            n_pos = n_pos + 1;
            pos_limit = (pos_limit*(n_pos - 1) + input(limits(n)))/n_pos;
        end    

        if i == 1
            left_limits = limits(1:(index-1));
            left_deadzone = pos_limit;
        else
            right_limits = limits(1:(index-1));
            right_deadzone = pos_limit;
        end

    end

    % Does the plotting
    plot(time, input, '-b')
    hold on
    
    yyaxis right
    plot(time, vel_esq, '-r')
    plot(time, vel_dir, '-g')
    yyaxis left
    
    for n = 1:length(left_limits)
        plot(time(left_limits(n)), input(left_limits(n)), 'r*')
    end
    for n = 1:length(right_limits)
        plot(time(right_limits(n)), input(right_limits(n)), 'g*')
    end    

    hold off
    
    
end

