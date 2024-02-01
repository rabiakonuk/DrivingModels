% a = importdata("US06.txt");

% datam = a.data;
% datam = [datam zeros(length(a.data),1)];
% datam(1,3) = 1000;
% ts = 0.1;
% datam(:,2) = datam(:,2)*0.44704;
% for i = 2:length(a.data)
%     datam(i,3) = datam(i-1,3) + ((datam(i,2) + datam(i-1,2))*0.5*ts);
% end
% 
% OVM(datam,40,0.65,15,1.5,0,1,985,4.5,0.1)

function cycle_f = OVM(cycle,v_max, tao, del_s, beta, v_f0,a_f0,s_f0,L, Ts, a_max, b_comf)
% Optimal Velocity Model
% the model marginally satisfies all plausibility conditions

% This code generates the following vehicle's speed (v_f) and location (s_f)
% when the lead vehicle's speed (v_l) and location (s_l) are given


% cycle: array composed of lead vehicle's time, speed and location (size: time by 3)
% v_f0, a_f0, s_f0: initial speed, acceleration and location of the following vehicle
% L: vehicle length
% Ts: simulation time step
% OVM parameters
% tao: adaptation time
% v_max: desired speed (v0)
% del_s: transition width
% beta: form factor
% v_a: actual velocity

t_ref = cycle(:,1);
v_l = cycle(:,2); %not used in this case
s_l = cycle(:,3);

v_a = v_f0;
v_f = v_f0;
a_f = a_f0;
s_f = s_f0;

for i = 1:length(cycle)
    if s_l(i)-s_f(i)-L<= 0
        crash = s_l(i)-s_f(i)-L;
        break
        %Use the keyboard command to pause execution of a program and modify a variable before continuing.
        %keyboard;
    end

    d_act(i) = s_l(i)-s_f(i)-L; %the distance between vehicles
    % optimal velocity
    v_opt(i) = v_max*(tanh(d_act(i)/del_s-beta)+tanh(beta))/(1+tanh(beta));

    % acceleration
    a_f(i+1) = (v_opt(i)-v_f(i))/Ts;
    if  a_f(i+1) > a_max
        a_f(i+1) = a_max;
    end
    if  a_f(i+1) < -b_comf
        a_f(i+1) = -b_comf;
    end

    %     % actual velocity
    %     v_a(i+1) = v_a(i)+ (a_f(i)+a_f(i+1))*0.5*Ts;
    % v_f(i+1) = v_f(i) + Ts*0.5*(a_f(i)+a_f(i+1));
    
    v_f(i+1) = v_f(i) + Ts*0.5*(a_f(i)+a_f(i+1));
    if  v_f(i+1) > v_max
        v_f(i+1) = v_max;
    end
    if  v_f(i+1) < 0
        v_f(i+1) = 0;
    end

    s_f(i+1) = s_f(i) + 0.5*Ts*(v_f(i+1)+v_f(i)); %uses Heun's method
    

end

t_f = [t_ref; t_ref(end)+Ts];
a_f = a_f';
v_f = v_f';
v_a = v_a';
s_f = s_f';
v_opt = v_opt';
% length(t_f)
% length(v_f)
% length(s_f)
% length(a_f)
d_act
a_f
v_f
v_opt

cycle_f = [t_f v_f s_f];
d_act = [d_act'; 0];


end