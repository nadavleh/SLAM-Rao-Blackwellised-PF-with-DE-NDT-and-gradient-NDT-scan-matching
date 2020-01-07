%% Implementing SLAM form the data of a Gazebo simulation
% This SLAM simulation takes the same data used in the PF_twoRobots.m
% simulation where two robot localize in a known map. This simulation is
% carried out with Nm maps, where each map has Np particles.
% Once we initialize all the particles to be at the middle of the occupancy
% grid (whose size we can vary), an initial map is created and shared for all
% Nm maps.
%% Load Robot 1 data

clear;

load('robot1.mat');
dt1=cell2mat(dt1);
omometry_data1=cell2mat(omometry_data1);
X1=cell2mat(X1);

map_to_obsticale_vector;
real_obsticle_vector=obsticle_vector;

clearvars -except  dt1 omometry_data1 X1 scans1 real_obsticle_vector

clc;
%% discard k first entries because robot is static at the begining
k=40;
scans1(1:k)=[];
dt1(1:k)=[];
omometry_data1(1:k,:)=[];
X1(1:k,:)=[];

%% initialize particles and maps
% build a probability map with size CellNum_x-by-CellNum_x each cell
% with probability 0.5

% define number of cells and create occupancy grid with designated function
% (it doesnt really need a designated function.. but he code is more tidy)
CellNum_x=181;
CellNum_y=CellNum_x;

initial_particles_map= OccGrid([CellNum_y,CellNum_x],1);
res=0.2; % 0.1 meters per cell [m/cell]

robot_cell_pose = [round(CellNum_x/2),round(CellNum_y/2),0];
%% version V5
% this is like version V4 but instead of calculating the mean of the
% resampled prtcls in each map, i take the partcl with the highest weight
% and build the map from his state

%% Build initial map from scans
% we need to tranform the scans to pixels using Bresenham's line algorithm:
% "https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm"
% [~,~,~,X_ray_cells,Y_ray_cells]=bresenham1(ones(21),[robot_cell_pose_x robot_cell_pose_y ; end_of_ray_x end_of_ray_y],ShowImage=0(no),1 (yes))
% This is a function i have downloaded. It will give us the x and y
% coordinates of the ray. we need to update the cells along the ray
% accordingly. One difficulty is that the pixel indxs rom which the lidar
% measured, is somtimes the last pixel and other times the first. we will
% deal with that issue soon.


% turn scan ends of each ray (in the first timestep), to cartisian coordinates
scans_cart = scan2cart(scans1{1,1}, robot_cell_pose(3));

% now we need to turn the cartisian reading to cell indecies along the
% laser ray
X_cells=[];
Y_cells=[];

% Get the cells along each ray and update them according to the bayesian
% model in update_map2()
for k=1:length(scans_cart)
    % retreive the cell indx of the end of each ray
    ray_end_cell=[robot_cell_pose(1)+ round(scans_cart(k,1)/res), robot_cell_pose(2)+round(scans_cart(k,2)/res)];
    % Get cell X and Y crdnts along each ray.
    [~,~,~,X_cells,Y_cells]=bresenham1( initial_particles_map(:,:), [robot_cell_pose(1:2); ray_end_cell] ,0);
    % Verify that the robot's cell corresponds to the first cell in the
    % ray's crdnts. this is easily done in verify_cell_order() just to save
    % space in this script.
    [X_cells,Y_cells,~]=verify_cell_order(robot_cell_pose,X_cells,Y_cells);
    %  update initial map along the ray 
    initial_particles_map(:,:)=update_map2(initial_particles_map(:,:),X_cells,Y_cells);
end 




%% Turn probability maps back to obsticle vector of cartisian coordinates

Nm=50; %Number of global maps, each on with Np particels
threshold=0.85;

obstacle_vector=cell(1,Nm); % Preallocate Memory for Cell Array for better use (we can do without it but its better practice)
obstacle_vector{1}=prob_map_to_obsticale_vector( initial_particles_map,res,threshold);

% all obstacle vectors are the same initially because the prob maps are..
% so we can just duplicate the obstacle vector for all other Nm maps.
% NOTE: when using braces '()' in cell arrays it means refering to the cell
% itself, while using curley braces '{}' refers to the specified cell's
% content
obstacle_vector(:)=obstacle_vector(1);



%% initialize particles
% number of particles per map
Np = 2; 

particles_state=zeros(Np,3,Nm);
particles_weight=zeros(Np,Nm);

%%  paricle filtering within a given map.

prob_map=repmat(initial_particles_map, 1, 1, Nm);

Num_Of_Sim_steps=length(dt1)-1;  % dt1 is 327 time-steps
% Num_Of_Sim_steps=100; % length(dt1)-1:

best_map=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);
maps_state=zeros(Num_Of_Sim_steps,3);
maps_cell=zeros(Num_Of_Sim_steps,2);

for t=1:Num_Of_Sim_steps
    disp('simulation progress [%]: ');
    sim_prog= t*100/Num_Of_Sim_steps;
    disp(sim_prog);
    
% % %  Write a text file stating simulation progress  % %
% % Uncomment to show                                   %
%                                                       %
%     fid = fopen('progress.txt','wt');                 %
%     fprintf(fid,'%g',sim_prog);                       %
%     fprintf(fid,'%c\t','%');                          %
%     fprintf(fid,'\n');                                %
%     fclose(fid);                                      %
%                                                       %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % %   


%%%%%%%%%%%%%%%%% Weigh particles and resample maps indecies %%%%%%%%%%%%%%
% weigh the particles in each map, then sum the weights in each map. the
% map that gets the largest sum is the best map. resample the map indecies
% with the according sumed particles weights.

    for m=1:Nm
    % Build a likelyhood function from
        % empirical cov-matrix, can be changed with trial and error.
        sigma=[0.8 0; 0 0.8]; 
        gm = gmdistribution(obstacle_vector{m},sigma);

    % weigh the particle
        particles_weight(:,m)  = weigh_particles_slam2(...
            particles_state(:,:,m), gm, scans1{t,1} );
        
        % weigh_particles_slam2() returns the log-weight (we set it up that
        % way for the followoing reason). inorder to get the weights we
        % need to exponentiate them, however this may lead to only zeros
        % due to truncation errors. So, we check if we only get zeros, if
        % so, set the maximum log-weight to 1 and all others to 0. (not the
        % most ellegant way but nah.. who cares it works.
        if sum(exp(particles_weight(:,m)))==0
            [~,max_idx]=max(particles_weight(:,m));
            particles_weight(:,m)=exp(particles_weight(:,m));
            particles_weight(max_idx,m)=1;
        else
            particles_weight(:,m)=exp(particles_weight(:,m));
            particles_weight(:,m)=particles_weight(:,m)/sum(particles_weight(:,m));
        end
    end
    % map weihgts is the weights of each map as the sum of particles
    % weights in each map. we offcourse normalize this vector.
    map_weights=sum(particles_weight)./sum(sum(particles_weight));
    % the best map is the one with the highest score.
    [~,max_weight_indx]=max(map_weights);
    % best map
    best_map(:,:,t)=prob_map(:,:,max_weight_indx);
    % the MAP state of the robot (particles) within that map
    maps_state(t,:)=mean(particles_state(:,:,max_weight_indx),1);
    % the cell of the robot corresponding to the map's mean state 
    maps_cell(t,:)=robot_cell(maps_state(t,:),res,CellNum_x);  
    % resample those maps
    resampled_maps_indxs=randsample((1:Nm),Nm,true,map_weights);
    
% % % % % % % % % % % %  plot Robot and map % % % % % % % % % % % % %   
    figure(1); clf;                                                 %
    subplot(1,2,1)                                                  %
    imagesc(best_map(:,:,t))                                        %
    colormap(flipud(gray))                                          %
    hold on                                                         %
    scatter(maps_cell(t,2),maps_cell(t,1),'b')                      %
    hold off                                                        %
    axis square;                                                    %
    camroll(180)                                                    %
    drawnow                                                         %
    subplot(1,2,2)                                                  %
    scatter(real_obsticle_vector(:,1),real_obsticle_vector(:,2), 1) %
    hold on                                                         %
    % plot real robot location                                      %
    plot_robot(X1(t,:)+[10 10 0],0.3)                               %
    axis square;                                                    %
    hold off                                                        %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 


%%%%%%%%% move particles and update maps  - only in resampled indecies %%%%
% resampled_maps_indxs is a vector of Nm indecies which can be repeated
% e.g. if the first map's weight is w=1 and all other maps are w=0 we
% get resampled_maps_indxs=[1,1,1,...,1] (Nm times). Inorder to avoid
% duplicating the first map Nm times and then moving the particles in
% each map which is the same one, we move the particles only in the
% resampled maps.for that rsui is defined as the resampled indecies
% with no repeating arguments, arranged in acsending order.     

    rsui=unique(resampled_maps_indxs); % rsui = re-sampled unique indecies
    MAP_state = zeros(length(rsui),3);
    for i=1:length(rsui)
        % move the particles in ech resampled map
        particles_state(:,:,rsui(i))= move_prtcls( ...
            particles_state(:,:,rsui(i)),omometry_data1(t,1),...
            omometry_data1(t,2),dt1(t) );
        
        % weigh the particle in each resampled map
        gm = gmdistribution(obstacle_vector{rsui(i)},sigma);
        % updates the partical's weights by incorporating the next time
        % step's measurments (LIDAR).
        particles_weight(:,rsui(i))  = weigh_particles_slam2( particles_state(:,:,rsui(i)), gm, scans1{t+1,1} );
        % weigh_particles_slam2() returns the log-weight (we set it up that
        % way for the followoing reason). inorder to get the weights we
        % need to exponentiate them, however this may lead to only zeros
        % due to truncation errors. So, we check if we only get zeros, if
        % so, set the maximum log-weight to 1 and all others to 0. (not the
        % most ellegant way but nah.. who cares it works.
        if sum(exp(particles_weight(:,rsui(i))))==0
            [~,max_idx]=max(particles_weight(:,rsui(i)));
            particles_weight(:,rsui(i))=exp(particles_weight(:,rsui(i)));
            particles_weight(max_idx,rsui(i))=1;
        else
            particles_weight(:,rsui(i))=exp(particles_weight(:,rsui(i)));
            particles_weight(:,rsui(i))=particles_weight(:,rsui(i))/sum(particles_weight(:,rsui(i)));
        end
        % resample the best particles in each of the resampled maps,
        % inorder to calc their mean arithmetically (and not weightedt
        % mean). from the calculated mean we will build the next time
        % step's map.
        [~,max_idx]=max(particles_weight(:,rsui(i)));
        MAP_state(i,:)=particles_state(max_idx,:,rsui(i));
               
        resampled_prtcl_idxs=randsample((1:Np),Np,true,particles_weight(:,rsui(i)));
        particles_state(:,:,rsui(i)) = particles_state(resampled_prtcl_idxs,:,rsui(i));
 
        % Get mean state variables [x y theta]
        mean_x=MAP_state(i,1); mean_y=MAP_state(i,2);
        mean_theta=MAP_state(i,3);
        % Get the next time step scan's cartisian end-points coordinantes with respect to
        % the mean, inorder to build the next time step's map
        scans_cart = scan2cart(scans1{t+1,1}, mean_theta );
        % Get the cell that corresponds to the particle's mean coordinates
        prtcl_cell = [round(CellNum_x/2+mean_x/res), round(CellNum_y/2+mean_y/res)];

        % now we can update each of the resampled maps, with the robot
        % position assumed in the mean state of each maps' particals.
        % To avoid the numerical overhead accompanied by updateing all 1080
        % scans from scans_cart, we randomly choose L rays, and update
        % according to them only.
        L=500; %number of scans used from the available 1080
        scan_number=length(scans_cart); % the number of scans ( should be 1080)
        idx=randsample(scan_number,L);
        % Get the cells along each chosen ray, using Bersenham's
        % alorithm, and update these cells accordingly.
        for j=1:length(idx)
            [~,~,~,X_cells,Y_cells]=bresenham1(prob_map(:,:,rsui(i)),...
                [prtcl_cell(1), prtcl_cell(2) ;...
                prtcl_cell(1)+round(scans_cart(idx(j),1)/res),...
                prtcl_cell(2)+round(scans_cart(idx(j),2)/res)],0);
            % Verify that the particl's cell corresponds to the first cell in the
            % ray's crdnts. this is easily done in verify_cell_order().
            [X_cells,Y_cells,~]=verify_cell_order(prtcl_cell,X_cells,Y_cells);
            %  update the probability map along the ray
            prob_map(:,:,rsui(i))=update_map2( prob_map(:,:,rsui(i)),X_cells,Y_cells);
        end   

        % We need to get the new obticale vector for each updated map.
        obstacle_vector{rsui(i)}=prob_map_to_obsticale_vector( prob_map(:,:,rsui(i)),res);
    end
    
%%%%%%%%%%%%%% duplicate resampled maps and particles %%%%%%%%%%%%%%%%%%%%%
    particles_state = particles_state(:,:,resampled_maps_indxs);
    prob_map=prob_map(:,:,resampled_maps_indxs);
    obstacle_vector(:)=obstacle_vector(resampled_maps_indxs);
end  


%% Save and Make Video
% save the Best_Maps array
% t=datetime('now');
% DateString = datestr(t);
% DateString(12)='_';DateString(3)='_';DateString(7)='_';DateString(15)='_';DateString(18)='_';
% save([DateString,'_best_map'], 'best_map');
% save([DateString,'_maps_cell'], 'maps_cell');
% % make video
% str=['Nm=',num2str(Nm),' Np=',num2str(Np),' L=',num2str(L)];
% SLAM_video3( best_map,maps_cell,real_obsticle_vector,X1,8,str)



