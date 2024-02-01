function cycle_f = IFVDM(cycle, v_max, tao, del_s, beta, gama, v_f0, a_f0, s_f0, L, Ts)
% IMPROVED FULL VELOCITY DIFFERENCE MODEL (IFVDM)
% Simulates the following vehicle's behavior with enhanced sensitivity to the leading vehicle's speed changes.
%
% INPUTS:
% cycle - Matrix with columns [time, lead vehicle speed, lead vehicle position].
% v_max - Desired speed in free traffic.
% tao - Time gap or reaction time of the following vehicle.
% del_s - Transition width in the optimal velocity function.
% beta - Form factor in the optimal velocity function.
% gama - Sensitivity factor for relative velocity.
% v_f0 - Initial speed of the following vehicle.
% a_f0 - Initial acceleration of the following vehicle.
% s_f0 - Initial position of the following vehicle.
% L - Length of the following vehicle.
% Ts - Time step for the simulation.
%
% OUTPUT:
% cycle_f - Simulation results with columns [time, following vehicle speed, following vehicle position].

% Initialize state variables for the following vehicle.
v_f = v_f0; 
a_f = a_f0; 
s_f = s_f0;
d_act = zeros(length(cycle), 1); % Distance to the leading vehicle

for i = 1:length(cycle)
    % actual distance
    d_act(i) = s_l(i) - s_f(i) - L;
    
    % optimal velocity
    v_opt = v_max * (tanh((d_act(i) / del_s) - beta) + tanh(beta)) / (1 + tanh(beta));

    % relative velocity
    r = v_f(i) - v_l(i);
    
    a_f(i+1) = (v_opt - v_f(i)) / tao - gama * r / max(1, d_act(i) / (v_max * Ts));
    v_f(i+1) = max(min(v_f(i) + Ts * (a_f(i) + a_f(i+1)) / 2, v_max), 0);    
    s_f(i+1) = s_f(i) + Ts * v_f(i+1);

    % Check for a crash scenario and reset speed if necessary
    if s_l(i) - s_f(i+1) <= 1.5 * L
        v_f(i+1) = 0; % 
    end
end

cycle_f = [cycle(:,1), v_f', s_f'];

end
