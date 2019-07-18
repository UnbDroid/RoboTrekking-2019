function [k_esq, tau_esq, k_dir, tau_dir] = get_system(name, left_deadzone, right_deadzone)

    pi = 3.1415926535;
    diameter = 0.121; %m
    CPR = 1632.67;

    table = readtable(name);
    data = table2array(table);

    [range, temp] = size(data);
    
    % Used for all plotting
    spaced_time = data(1,1):((data(range, 1) - data(1,1))/(range-1)):data(range,1);
    spaced_time = spaced_time.';
    
    % Calculo do periodo medio com media movel
    ts = data(1, 1);
    for i = 2:range
        ts = (ts*(i-1)+(data(i,1)-data(i-1,1)))/i;
    end
    
    % Conversao das leituras dos encoders para metro
    for i = 1:(range)
        data(i, 3) = abs( data(i, 3)*pi*diameter/CPR ); %left
        data(i, 4) = abs( data(i, 4)*pi*diameter/CPR ); %right
    end
    
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
    
    % Input considerando deadzone
    input_left = zeros(range, 1);
    input_right = zeros(range, 1);
    
    for i = 1:(range)
       if(data(i, 2) <= left_deadzone)
           input_left(i) = 0;
       else
           input_left(i) = data(i, 2) - left_deadzone;
       end
       if(data(i, 2) <= right_deadzone)
           input_right(i) = 0;
       else
           input_right(i) = data(i, 2) - right_deadzone;
       end
    end
    
    % System identification through system dynamics
    error = 0.05;
    time_error = 0.2;
    
    peak_left = max(vel_esq);
    medium_peak_left = 0;
    time_to63_left = 0;
    
    start_time = 0;
    count = 1;
    state = 1;
    
    for i = 1:(range)
        if(abs(peak_left - vel_esq(i)) <= error)
            medium_peak_left = (medium_peak_left*(count-1) + vel_esq(i))/count;
            count = count + 1;
        end
    end
    
    count = 1;
    
    points_0_left = zeros(10, 1);
    points_63_left = zeros(10, 1);
    
    for i = 1:(range)
        if((vel_esq(i) < error) && (vel_esq(i) > error/10) && state == 1)
            start_time = data(i, 1);
            state = 0;
            points_0_left(count) = i;
        end
        if(abs(vel_esq(i)-0.63*medium_peak_left) < error/2 && state == 0)
            time_to63_left = (time_to63_left*(count-1) + (data(i, 1)-start_time))/count;
            points_63_left(count) = i;
            count = count + 1;
            state = 2;
        end
        if(vel_esq(i) < error/10 && state == 2)
            state = 1;
        end
    end
    
    points_0_left = points_0_left(1:(count-1));
    points_63_left = points_63_left(1:(count-1));
    
    peak_right = max(vel_dir);
    medium_peak_right = 0;
    time_to63_right = 0;
    
    start_time = 0;
    count = 1;
    state = 1;
    
    for i = 1:(range)
        if(abs(peak_right - vel_dir(i)) <= error)
            medium_peak_right = (medium_peak_right*(count-1) + vel_dir(i))/count;
            count = count + 1;
        end
    end
    
    count = 1;
    
    points_0_right = zeros(10, 1);
    points_63_right = zeros(10, 1);
    
    for i = 1:(range)
        if((vel_dir(i) < error) && (vel_dir(i) > error/10) && state == 1)
            start_time = data(i, 1);
            state = 0;
            points_0_right(count) = i;
        end
        if(abs(vel_dir(i)-0.63*medium_peak_right) < error/2 && state == 0)
            time_to63_right = (time_to63_right*(count-1) + (data(i, 1)-start_time))/count;
            points_63_right(count) = i;
            count = count + 1;
            state = 2;
        end
        if(vel_dir(i) < error/10 && state == 2)
            state = 1;
        end
    end
    
    points_0_right = points_0_right(1:(count-1));
    points_63_right = points_63_right(1:(count-1));    
    
    max_voltage_left = max(input_left);
    max_voltage_right = max(input_right);
    
    k_left_dynamic = medium_peak_left/max_voltage_left;
    tau_left_dynamic = time_to63_left;
    
    k_right_dynamic = medium_peak_right/max_voltage_right;
    tau_right_dynamic = time_to63_right;
    
    left_sys_dynamic = tf([k_left_dynamic], [tau_left_dynamic 1]);
    right_sys_dynamic = tf([k_right_dynamic], [tau_right_dynamic 1]);
    
    % Plotting dynamic analysis system
    figure(1);
    plot(spaced_time, input_left);
    hold on
    plot(spaced_time, vel_esq);
    line([spaced_time(1) spaced_time(range)], [medium_peak_left medium_peak_left]);
    lsim(left_sys_dynamic, input_left, spaced_time);
    title("Identificação motor esquerda (Dinamico)");
    
    for i=1:length(points_0_left)
        plot(spaced_time(points_0_left(i)), vel_esq(points_0_left(i)), 'r*');
    end
    for i=1:length(points_63_left)
        plot(spaced_time(points_63_left(i)), vel_esq(points_63_left(i)), 'r*');
    end    
    
    hold off
    
    figure(2);
    plot(spaced_time, input_right);
    hold on
    plot(spaced_time, vel_dir);
    line([spaced_time(1) spaced_time(range)], [medium_peak_right medium_peak_right]);
    lsim(right_sys_dynamic, input_right, spaced_time);    
    title("Identificacao motor direita (Dinamico)");

    for i=1:length(points_0_right)
        plot(spaced_time(points_0_right(i)), vel_dir(points_0_right(i)), 'r*');
    end
    for i=1:length(points_63_right)
        plot(spaced_time(points_63_right(i)), vel_dir(points_63_right(i)), 'r*');
    end    
    
    hold off
    
    % System identification through math
    M_left = [vel_esq(1:range-1) input_left(1:range-1)];
    M_right = [vel_dir(1:range-1) input_right(1:range-1)];
    
    A_left = [vel_esq(2:range)];
    A_right = [vel_dir(2:range)];
    
    Theta_left = ((M_left.'*M_left)^-1)*M_left.'*A_left;
    Theta_right = ((M_right.'*M_right)^-1)*M_right.'*A_right;
    
    tau_left_mat = ts/(1.0 - Theta_left(1));
    k_left_mat = (tau_left_mat*Theta_left(2))/ts;
    
    tau_right_mat = ts/(1 - Theta_right(1));
    k_right_mat = (tau_right_mat*Theta_right(2))/ts;
    
    % plotting mathematical system
    left_sys_mat = tf([k_left_mat], [tau_left_mat 1]);
    right_sys_mat = tf([k_right_mat], [tau_right_mat 1]);
    
    figure(3);
    plot(spaced_time, input_left);
    hold on
    plot(spaced_time, vel_esq);
    lsim(left_sys_mat, input_left, spaced_time);
    title("Identificação motor esquerda (Matricial)");
    hold off
    
    figure(4);
    plot(spaced_time, input_right);
    hold on
    plot(spaced_time, vel_dir);
    lsim(right_sys_mat, input_right, spaced_time);    
    title("Identificacao motor direita (Matricial)");
    hold off

    % Save best values into return variables
    k_esq = k_left_dynamic;
    tau_esq = tau_left_dynamic;
    
    k_dir = k_right_dynamic;
    tau_dir = tau_right_dynamic;
end

