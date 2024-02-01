function cycle_f = IDM(cycle,d_min,T_head,v_max,a_max,b_comf,v_f0,a_f0,s_f0,L,delta,Ts)
% Intelligent Driver Model

t_ref = cycle(:,1);
v_l = cycle(:,2);
s_l = cycle(:,3);
% Ts = 1;
v_f = v_f0;
a_f = a_f0;
s_f = s_f0;
% 
% if size(v_max,1) > 1
%     speed_limit = v_max ;
% end
% v_max = v_max(1,2) ;

for i = 1:length(cycle)
    
    %v_max = speed_limit(i) ;

    
    if s_l(i)-s_f(i)-L<= 0
        crash = s_l(i)-s_f(i)-L
        keyboard;
    end
    d_act(i) = s_l(i)-s_f(i)-L;
    r(i) = v_f(i)- v_l(i);
    
    d_des(i) = d_min + T_head*v_f(i) + v_f(i)*r(i)/2/sqrt(a_max*b_comf);
    a_f(i+1) = a_max * (1 - (v_f(i)/v_max)^delta - (d_des(i)/d_act(i))^2);
    
%     v_idm_des = 25;
%     if v_f(i) > v_idm_des
%         a_f(i+1) = -b_comf * ( 1 - (v_idm_des/v_f(i))^delta ) ;
%     end
    
    v_f(i+1) = v_f(i) + 0.5*Ts*(a_f(i+1)+a_f(i));
    
    if v_f(i+1) > v_max
        v_f(i+1) = v_max;
    end
    if v_f(i+1) < 0
        v_f(i+1) = 0;
    end
    
    s_f(i+1) = s_f(i) + 0.5*Ts*(v_f(i+1)+v_f(i));
    
end
t_f = [t_ref; t_ref(end)+Ts];
v_f = v_f';
s_f = s_f';
cycle_f = [t_f v_f s_f];
d_act = [d_act'; 0];
end
