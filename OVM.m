function cycle_f = OVM(cycle, v_max, tao, del_s, beta, v_f0, a_f0, s_f0, L, Ts, a_max, b_comf)
% OPTIMAL VELOCITY MODEL (OVM)
% This model calculates the following vehicle's acceleration based on its optimal velocity, 
% which depends on the distance to the vehicle in front.

% INPUTS:
% cycle - A matrix with columns representing the time, speed, and location of the lead vehicle.
% v_max - The desired speed of the following vehicle in free traffic.
% tao - Time gap or reaction time.
% del_s - Transition width in the optimal velocity function.
% beta - Form factor in the optimal velocity function.
% v_f0 - Initial speed of the following vehicle.
% a_f0 - Initial acceleration of the following vehicle.
% s_f0 - Initial position of the following vehicle.
% L - Length of the vehicle.
% Ts - Time step for the simulation.
% a_max - Maximum acceleration of the following vehicle.
% b_comf - Comfortable braking deceleration.

% OUTPUT:
% cycle_f - A matrix with columns [time, following vehicle speed, following vehicle position].

% Initialize state variables for the following vehicle
v_f = v_f0; 
a_f = a_f0; 
s_f = s_f0; 
d_act = zeros(length(cycle), 1); % Actual distance to the leading vehicle

for i = 1:length(cycle)

    d_act(i) = s_l(i) - s_f(i) - L;
    
    % optimal velocity
    v_opt = v_max * (tanh(d_act(i) / del_s - beta) + tanh(beta)) / (1 + tanh(beta));

    a_f(i+1) = (v_opt - v_f(i)) / Ts;
    a_f(i+1) = min(max(a_f(i+1), -b_comf), a_max);

    v_f(i+1) = max(min(v_f(i) + 0.5 * Ts * (a_f(i) + a_f(i+1)), v_max), 0);
    
    s_f(i+1) = s_f(i) + 0.5 * Ts * (v_f(i+1) + v_f(i)); 
    
    % If the distance is too small reset it to prevent crashes
    if d_act(i) <= 0.2
        v_f(i+1) = 0;
        a_f(i+1) = -b_comf;
    end
end

cycle_f = [cycle(:, 1), v_f', s_f'];
end
