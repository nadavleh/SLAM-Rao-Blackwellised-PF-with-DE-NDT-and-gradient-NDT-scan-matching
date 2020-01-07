function [ gm ] = GMM_map( obstacles, map_lim , plot_var)
%Create a gmm from the map's obstacles with uniform cov matrix for all and
%equall weights for each gaussian, plot_var = 1, will plot the map.


    sigma=[0.8 0; 0 0.8];
    gm = gmdistribution(obstacles,sigma);
    if (plot_var==1)
        figure(1); clf;
        ezsurf(@(x,y)pdf(gm,[x y]),[map_lim(1)-2 map_lim(2)+2],[map_lim(3)-2 map_lim(4)+2])
        view([-1.1 82]);

    end

end

