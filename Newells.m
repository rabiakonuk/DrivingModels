% Newell's driving model
% the logic behind is different from the others
% check Newells paper pg.5 & Treiberg's book pg. 181
% clear all
% 
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
% Newells(datam,40,0.65,0,1,900,4,0.1)

function cycle_f = Newells(cycle,v_max,v_f0,a_f0,s_f0,L,Ts)

% This code generates the following vehicle's speed (v_f) and location (s_f) 
% when the lead vehicle's speed (v_l) and location (s_l) are given

% cycle: array composed of lead vehicle's time, speed and location (size: time by 3)
% v_f0, a_f0, s_f0: initial speed, acceleration and location of the following vehicle
% L: vehicle length
% Ts: simulation time step

% Newell's parameters
% tao: adaptation time
% v_max: desired speed (v0)
% delta = s_0 + L_vehicle


%     according to Newell's model:
%     delta = s_0 + L_vehicle
%     d_n(t)= v_n(t)*tao + delta
%     v(t+tao) = min(v_max, (d_n(t)-delta)/tao)
%     x_n(t+tao) = x_n + v_n(t+tao)*tao
%     NCM adopts a single wave speed –δ/τ independent of traffic states.
%     traffic info in the v_max or wave speed


t_ref = cycle(:,1);
v_l = cycle(:,2); 
s_l = cycle(:,3); 

v_f = v_f0;
a_f = a_f0;
s_f = s_f0;
%beta = 1.5;
del_s = 4;
l_eff = 5;

for i = 1:length(cycle)

    s_f(i+1) = s_l(i) - l_eff;
    v_f(i+1) = (s_f(i+1) - s_f(i))/Ts;
    if  v_f(i) > v_max
        v_f(i) = v_max;
        s_f(i+1) = s_f(i) + 0.5*Ts*(v_f(i+1)+v_f(i));
    end
    if v_f(i) < 0
        v_f(i) = 0;
        s_f(i+1) =  s_f(i) ;
    end



%         d_act(i+1) = s_l(i)-s_f(i)-L; %the distance between vehicles
%     
%     if (d_act(i) - del_s)/(v_max*Ts)  <= 1 && (d_act(i) - del_s) >0
% 
%          a_f(i+1) = (v_l(i)-v_f(i))/tao;
%          v_opt(i+1) = v_max*(tanh(d_act(i)/del_s-beta)+tanh(beta))/(1+tanh(beta));
%          v_f(i+1) = v_opt(i+1);
%          % v_f(i+1) = v_f(i) + 0.5*Ts*(a_f(i+1)+a_f(i));
%          s_f(i+1) = s_f(i) + 0.5*Ts*(v_f(i+1)+v_f(i));
%     else
% 
%         v_f(i+1) = 0;
%         s_f(i+1) = s_f(i);
%         
% 
%     end
%         
%         if  v_f(i) > v_max
%              v_f(i) = v_max;  
%         end
%         if v_f(i) < 0
%             v_f(i) = 0;
%         end


end

t_f = [t_ref; t_ref(end)+Ts];
a_f = a_f';
v_f = v_f';
s_f = s_f';
cycle_f = [t_f v_f s_f];

end

