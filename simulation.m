function [T, X] = simulation(state0,u1)

tspan =[0;0.01];
[T, STATE] = ode45(@inverse_dynamics_single_pendulum,tspan,state0,[],u1);
X = STATE';

%X(2,:) = mod(X(2,:),2*pi); %one full turn doesnt matter as long as we dont have a spring!  
end

function [dstate] = inverse_dynamics_single_pendulum(t,state,u1)

q1 = state(1);
dq1 = state(2);
    
%g = 9.81;
g= 0;
m1 = 5; 
l1 = 1;
Jyy1 = 5; %Nm 

M = Jyy1;
c = 0;
B = 1;
G = cos(q1)*g;

% B*u1 = M*ddq1 + c + G

ddq1 = inv(M)*(-c - G +B*u1);
dq1 = dq1;

dstate = [dq1; ddq1];
end

