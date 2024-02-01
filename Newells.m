function cycle_f = Newells(cycle, v_max, v_f0, a_f0, s_f0, L, Ts)
% NEWELL'S CAR-FOLLOWING MODEL
% Computes the trajectory of a following vehicle using Newell's simplified car-following logic.
%
% INPUTS:
% cycle - Matrix with columns [time, lead vehicle speed, lead vehicle position].
% v_max - Desired speed in free traffic.
% v_f0 - Initial speed of the following vehicle.
% a_f0 - Initial acceleration of the following vehicle (unused in Newell's model).
% s_f0 - Initial position of the following vehicle.
% L - Length of the following vehicle.
% Ts - Time step for the simulation.
%
% OUTPUT:
% cycle_f - Simulation results with columns [time, following vehicle speed, following vehicle position].

% Initialize state variables for the following vehicle.
v_f = v_f0; 
s_f = s_f0; 
effective_length = L + 2; % Effective vehicle length accounting for minimum following distance

for i = 1:length(cycle)

    s_f(i+1) = s_l(i) - effective_length;
    
    v_f(i+1) = max(min((s_f(i+1) - s_f(i)) / Ts, v_max), 0); % Speed is constrained between 0 and v_max.

    s_f(i+1) = s_f(i) + Ts * v_f(i+1);
end

cycle_f = [cycle(:, 1), v_f', s_f'];

end
