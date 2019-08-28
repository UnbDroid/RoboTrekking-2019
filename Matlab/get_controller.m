function [G, H, C] = get_controller(km, tau)

    % Using the parameters of the system, generate a discrete state space
    % controller

    % The output Y is the distance
    
    % Matrix initialization
    A = [0 1 ; 0 -1/tau];
    B = [0 ; km/tau];
    C = [1 0];
    
    T = 0.2;

    %%%%%%%%%%%%%%%%%%%%
    %%% First Method %%%
    %%%%%%%%%%%%%%%%%%%%

    % Through matlab state-space to tf and inverse functions

    % Continuous transfer function representation
    [num, den] = ss2tf(A, B, C, [0]);
    sysc = tf(num, den);

    % Transform to discrete
    sysd = c2d(sysc, T, 'zoh');
    [num, den] = tfdata(sysd);

    num = cell2mat(num);
    den = cell2mat(den);
    
    % Discrete matrixes
    [G1, H1, C1, D1] = tf2ss(num, den);

    %%%%%%%%%%%%%%%%%%%%
    %%% Secnd Method %%%
    %%%%%%%%%%%%%%%%%%%%

    % Using c2d directly
    [G2 , H2] = c2d(A,B,T);
    C2 = C;
    D2 = 0;
    
    %%%%%%%%%%%%%%%%%%%%
    %%% Third Method %%%
    %%%%%%%%%%%%%%%%%%%%

    % Manual method through paper calculations

    % Funcoes de transferencia ja na FCC
    G3 = [0 1 ; -exp(-T/tau) (1+exp(-T/tau))];
    H3 = [0 ; 1];
    C3 = [(km*tau*(1-exp(-T/tau)) - km*T*exp(-T/tau)) (km*tau*(exp(-T/tau) -1) + km*T)];
    D3 = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Actual Controller %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Here we choose from one of the methods above
    G = G3;
    H = H3;
    
    
    
end