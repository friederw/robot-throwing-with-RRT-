function auto_test()

nIteratons = 1000;
u1_max = 1;
plot_on =1;

[X_RRT I_closest throw_X DISTANCE] = create_RRT_throw(nIteratons,u1_max,plot_on); 

end