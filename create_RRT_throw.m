function [X_RRT, I_closest throw_X, DISTANCE] = create_RRT_throw(iterations,u_max,plot_on)

if exist('plot_on')
else
    plot_on = 1;
end

if plot_on
    close all
    %which joints to plot
    a = 1; %x axis
    b= 2; %y axis
end

x0 = [0;0];

%check here if legal

if exist('iterations')
else
    iterations = 1000
end
N = iterations;

if exist('u_max')
else
    u_max = [2];
end

u1_max = u_max;

X = x0;
I_closest = [0];
throw_X = [];
DISTANCE = [];

if plot_on
    plot(X(a,1),X(b,1),'r+')
    hold on
    pause(0.05)                    
end

for i = 1:N    
    x_demanded = 2*pi*rand(2,1) .* sign(0.5-rand(2,1));
    [x_closest i_closest] = find_closest_x(X, x_demanded);

    u1 = compute_torque_LIP_PID(x_closest,x_demanded,u1_max)
    [T, STATE] = simulation(x_closest,u1);
    [throw distance] = ask_does_throw_happen(STATE);
    x_new = STATE(:,end);
    
    %if check_if_pose_is_legal(struct_pose,struct_biped_parameter)
    if throw 
        throw_X = [throw_X x_new];
        DISTANCE = [DISTANCE distance];
        if plot_on
                    line([x_closest(a), x_new(a)],[x_closest(b), x_new(b)])
                    plot(throw_X(a,end),throw_X(b,end),'b+')
                    title(['iteration: ',num2str(i),'  RRT size: ',num2str(size(X,2)),'  number of throws: ',num2str(size(throw_X,2))])
                    pause(0.05)
            end
    else    
        X = [X, x_new];
        I_closest = [I_closest i_closest]; 
            if plot_on
                    line([x_closest(a), x_new(a)],[x_closest(b), x_new(b)])                    
                    title(['iteration: ',num2str(i),'  RRT size: ',num2str(size(X,2)),'  number of throws: ',num2str(size(throw_X,2))])
                    pause(0.05)
            end
    end
end

X_RRT = X;

number_of_poses_created = size(X_RRT,2)
out_of_iterations = N
end

function [x_closest i_closest] = find_closest_x(X, x_demanded)

distance = zeros(1,size(X,2));
for i = 1:size(X,2)
    distance(i) = norm(X(:,i) - x_demanded);
end
[v k] = min(distance);

x_closest = X(:,k);
i_closest = k;
end

function u1 = compute_torque_LIP_PID(x_closest,x_demanded,u1_max)
    p1 =1;
    d1 =1;
    u1 = [p1 d1] * (x_demanded - x_closest);
    if u1 > u1_max
        u1 = u1_max;
    elseif u1 < -u1_max;
        u1 = -u1_max;
    end
end

function [throw distance] = ask_does_throw_happen(STATE)

if max(abs(STATE(:,2))) > 0.1
    dq1_start = STATE(2,1);
    if dq1_start >= 0    
        if sum(sign(STATE(2,2)-1)) < 0
            throw = true;
        end
    elseif dq1_start < 0
        if sum(sign(STATE(2,2)+1)) > 0
            throw = true;
        end
    else
        throw = 0;
        distance = 0;
    end
else
    throw = 0;
    distance = 0;
end

    
if throw
    distance = calculate_throw_distance(STATE);
end
end

function distance = calculate_throw_distance(STATE)
distance = 0;
end

    
