
% a = importdata("US06.txt");
% 
% datam = a.data;
% datam = [datam zeros(length(a.data),1)];
% datam(1,3) = 1000;
% ts = 0.1;
% datam(:,2) = datam(:,2)*0.44704;
% for i = 2:length(a.data)
%     datam(i,3) = datam(i-1,3) + ((datam(i,2) + datam(i-1,2))*0.5*ts);
% end
% 
% Gipps(datam, 2, 40, 3.08, 1.4, 0, 1, 900, 4, 0.1)

function cycle_f = Gipps(cycle,d_min, v_max, a_max, b_comf, v_f0,a_f0,s_f0,L, Ts)
% Gipps' Model

% This code generates the following vehicle's speed (v_f) and location
% (s_f) when the lead vehicle's speed (v_l) and location(s_l) are given

% cycle: array composed of lead vehicle's time, speed and location (size: time by 3)
% v_f0, a_f0, s_f0: initial speed, acceleration and location of the following vehicle
% L: vehicle length
% Ts: simulation time step

% Gipps parameters:
% d_min: minimum gap (typical value in traffic: 2m)
% v_max: desired speed (v sub 0 in eqn)
% a_max: acceleration (a in eqn)
% b_comf: comfortable deceleration

t_ref = cycle(:,1);
v_l = cycle(:,2);
s_l = cycle(:,3);

v_f = v_f0;
a_f = a_f0;
s_f = s_f0;
d_act(1) = s_l(1) - s_f0 - L;

%     if s_l(i)-s_f(i)-1.5*L<= 0
%         v_f(i+1) = 0;
%         s_f(i+1) = s_f(i);
% d_act(i) - d_min  - v_f(i)*Ts + v_l(i)^2/2*b_comf - v_f(i)^2/2*b_comf  >= 0


for i = 1:length(cycle)

    d_act(i+1) = s_l(i)-s_f(i)-L; %the distance between vehicles
    
    if d_act(i) - d_min  >= 0

        v_safe(i+1) =  sqrt(((b_comf^2)*(Ts^2))+(v_l(i)^2)+(2*b_comf*(d_act(i)-d_min))) -b_comf*Ts ;
               
        v_normal(i+1) = v_f(i)+ a_max*Ts;
        
        v_f(i+1) = min([v_normal(i+1) v_safe(i+1) v_max]);
       
        s_f(i+1) = s_f(i) + 0.5*Ts*(v_f(i+1)+v_f(i));

%       d_act(i+1) = s_l(i)-s_f(i)-L;

    else

        v_f(i+1) = 0;
        s_f(i+1) = s_f(i);
        

    end
        
        if  v_f(i) > v_max
             v_f(i) = v_max;  
        end
        if v_f(i) < 0
            v_f(i) = 0;
        end
        
end
% csvwrite('v_safe.csv', v_safe);
t_f = [t_ref; t_ref(end)+Ts];
v_f = v_f';
s_f = s_f';
cycle_f = [t_f v_f s_f];
%d_act = [d_act'; 0];
% hold on
% plot(cycle_f(:,1),cycle_f(:,2))
% plot(cycle(:,1), cycle(:,2)) 
end
