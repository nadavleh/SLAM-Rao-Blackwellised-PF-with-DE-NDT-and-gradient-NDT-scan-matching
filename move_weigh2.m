function [ particles_state,particles_weight ] = move_weigh2( obstacle_vector, particles_state, scan_cell, omometry_data1,dt )
%% move particles
    particles_state= move_prtcls( particles_state,omometry_data1(1), omometry_data1(2),dt );
%% Build a likelyhood function from
    % empirical cov-matrix, can be changed with trial and error.
    sigma=[0.8 0; 0 0.8]; 
    gm = gmdistribution(obstacle_vector(:,:),sigma);

%% weigh the particle
    particles_weight  = weigh_particles_slam2( particles_state, gm, scan_cell );
end

