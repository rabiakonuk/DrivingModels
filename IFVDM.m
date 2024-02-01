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
% IFVDM(datam,40,0.65,15,1.5,0.6,0,1,1000,2,0.1)
% cycle = datam;
% v_max =40;
% tao=5;
% del_s=15;
% beta=1.5;
% gama=0.6;
% v_f0=0;
% a_f0=1;
% s_f0=900;
% L=2;
% Ts=0.1;

function cycle_f = IFVDM(cycle,v_max, tao, del_s, beta, gama, v_f0,a_f0,s_f0,L, Ts)
% Improved Full Velocity Difference Model

% This code generates the following vehicle's speed (v_f) and location
% (s_f) when the lead vehicle's speed (v_l) and location (s_l) are given

% cycle: array composed of lead vehicle's time, speed and location (size: time by 3)
% v_f0, a_f0, s_f0: initial speed, acceleration and location of the following vehicle
% L: vehicle length
% Ts: simulation time step
% OVM parameters
% tao: adaptation time
% v_max: desired speed (v0)
% del_s: transition width 
% beta: form factor
% gama: speed difference sensitivity

t_ref = cycle(:,1);
v_l = cycle(:,2); 
s_l = cycle(:,3);

v_f = v_f0;
v_a = v_f0; 
a_f = a_f0;
s_f = s_f0;

for i = 1:length(cycle)
%     if s_l(i)-s_f(i)-L<= 0
%         crash = s_l(i)-s_f(i)-L; break
% %         v_a(i) = 0; 
% %        Use the keyboard command to pause execution of a program and modify  
% %        a variable before continuing.
%        keyboard
%     end

    d_act(i) = s_l(i)-s_f(i)-L; %the distance between vehicles
    
    % optimal velocity
    v_opt(i+1) = v_max*(tanh(d_act(i)/del_s-beta)+tanh(beta))/(1+tanh(beta));

    % relative velocity
    r(i) = v_a(i)- v_l(i); 

    % acceleration
    a_f(i+1) = ((v_opt(i)-v_a(i))/tao) - gama*r(i)/max([1 d_act(i)/(v_max*Ts)]);

    % actual velocity
    v_a(i+1) = v_a(i)+ (a_f(i)+a_f(i+1))*0.5*Ts;


    if  v_a(i) > v_max
        v_a(i) = v_max;  
    end
    
    if v_a(i) < 0
        v_a(i) = 0;
    end

    s_f(i+1) = s_f(i) + 0.5*Ts*(v_a(i+1)+v_a(i)); %uses Heun's method 
    
    if s_l(i)-s_f(i)-1.5*L<= 0
        v_a(i) = 0;
    end

end

t_f = [t_ref; t_ref(end)+Ts];
a_f = a_f';
v_a = v_a';
s_f = s_f';
cycle_f = [t_f v_a s_f];

% hold on
% plot(cycle_f(:,1),cycle_f(:,2))
% plot(cycle(:,1), cycle(:,2))
end

