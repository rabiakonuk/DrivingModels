function cycle_f = IDM(cycle, d_min, T_head, v_max, a_max, b_comf, v_f0, a_f0, s_f0, L, delta, Ts)
% INTELLIGENT DRIVER MODEL (IDM)
% Simulates the longitudinal behavior of a following vehicle based on IDM.
%
% INPUTS:
% cycle - Matrix with columns [time, lead vehicle speed, lead vehicle position].
% d_min - Minimum desired spacing in free traffic.
% T_head - Desired time headway to the leading vehicle.
% v_max - Desired speed in free traffic.
% a_max - Maximum vehicle acceleration.
% b_comf - Comfortable deceleration.
% v_f0 - Initial speed of the following vehicle.
% a_f0 - Initial acceleration of the following vehicle.
% s_f0 - Initial position of the following vehicle.
% L - Length of the following vehicle.
% delta - Acceleration exponent.
% Ts - Time step for the simulation.
%
% OUTPUT:
% cycle_f - Simulation results with columns [time, following vehicle speed, following vehicle position].

% Initialize state variables for the following vehicle
v_f = v_f0;
a_f = a_f0; 
s_f = s_f0; 
d_act = zeros(size(cycle, 1), 1); % Actual distance to the leading vehicle

% Main simulation loop
for i = 1:length(cycle)
    % Check for a crash
    if s_l(i) - s_f(i) - L <= 0
        error('Crash detected!');
    end
    
    % Calculate the actual and desired distances
    d_act(i) = s_l(i) - s_f(i) - L; 
    d_des = d_min + T_head * v_f(i) + (v_f(i) * (v_f(i) - v_l(i))) / (2 * sqrt(a_max * b_comf)); 
    
    % Update the acceleration using the IDM formula.
    a_f(i+1) = a_max * (1 - (v_f(i) / v_max)^delta - (d_des / d_act(i))^2);
    
    % Update the speed and position of the following vehicle.
    v_f(i+1) = max(min(v_f(i) + Ts * a_f(i+1), v_max), 0); % Ensure speed limits
    s_f(i+1) = s_f(i) + Ts * v_f(i) + 0.5 * Ts^2 * a_f(i+1); 
    
end

cycle_f = [cycle(:, 1), v_f', s_f'];

end
