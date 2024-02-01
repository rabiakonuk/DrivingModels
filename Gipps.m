function cycle_f = Gipps(cycle,d_min, v_max, a_max, b_comf, v_f0,a_f0,s_f0,L, Ts)

% GIPPS' MODEL
% Simulates the behavior of a following vehicle based on Gipps' car-following model.
%
% INPUTS:
% cycle - Matrix with columns [time, lead vehicle speed, lead vehicle position].
% d_min - Minimum safe following distance.
% v_max - Desired speed of the following vehicle.
% a_max - Maximum acceleration of the following vehicle.
% b_comf - Comfortable deceleration rate.
% v_f0 - Initial speed of the following vehicle.
% a_f0 - Initial acceleration of the following vehicle.
% s_f0 - Initial position of the following vehicle.
% L - Length of the following vehicle.
% Ts - Time step for the simulation.
%
% OUTPUT:
% cycle_f - Simulation results with columns [time, following vehicle speed, following vehicle position].

% Initialize state variables for the following vehicle.

t_ref = cycle(:,1);
v_l = cycle(:,2);
s_l = cycle(:,3);

% Initialize state variables for the following vehicle
v_f = v_f0; 
a_f = a_f0; 
s_f = s_f0; 
d_act = zeros(size(cycle, 1), 1); % Distance between vehicles
d_act(1) = cycle(1,3) - s_f0 - L; % Initial actual distance

% Simulate the following vehicle dynamics
for i = 1:length(cycle)
    d_act(i+1) = cycle(i,3) - s_f(i) - L; % Update distance between vehicles
    
    if d_act(i) - d_min >= 0
        v_safe = sqrt((b_comf^2) * (Ts^2) + (cycle(i,2)^2) + (2 * b_comf * (d_act(i) - d_min))) - b_comf * Ts;
        v_normal = v_f(i) + a_max * Ts;
        v_f(i+1) = min([v_normal, v_safe, v_max]); % Determine next speed
        s_f(i+1) = s_f(i) + 0.5 * Ts * (v_f(i+1) + v_f(i)); % Determine next position
    else
        v_f(i+1) = 0; % Stop if too close
        s_f(i+1) = s_f(i);
    end
    
    v_f(i+1) = max(min(v_f(i+1), v_max), 0); % Enforce speed limits
end

t_f = [t_ref; t_ref(end)+Ts];
v_f = v_f';
s_f = s_f';
cycle_f = [t_f v_f s_f];
end
