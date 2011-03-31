function auto_test()
clc
options_create_RRT_throw = struct('nIterations',[],'u_max',[],'plot_on',[],'plot_pause_time',[]);

options_create_RRT_throw.nIterations = 1000;
options_create_RRT_throw.u_max = 1;
options_create_RRT_throw.plot_on =1;
options_create_RRT_throw.plot_pause_time = 0.001;

%options_create_RRT_throw = init_options_create_RRT_throw();

[X_RRT I_closest throw_X DISTANCE] = create_RRT_throw(options_create_RRT_throw); 

end