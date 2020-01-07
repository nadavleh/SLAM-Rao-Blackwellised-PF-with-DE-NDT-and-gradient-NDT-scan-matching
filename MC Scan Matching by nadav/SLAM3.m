%% Implementing SLAM form the data of a Gazebo simulation
% This SLAM simulation akes the same data used in the PF_twoRobots.m
% simulation where two robot localize in a known map. This simulation is
% carried out with a Np particles, where each particle has its own map.
% Once we initialize all the particles to be at the middle of the occupancy
% grid (whose size we can vary), an initial map is created and shared for all
% particles. Then we can move each particle, update its map according to the Baesian binary 
% update rule, and weigh each particle according to its map. We then
% resample particles and their matching maps, and so on.
%% Difference from SLAM2
%This simulation differs from SLAM2 by using only L measurements to either
%scan match or/and reconstruct the map. This is done by using
%weigh_particles_slam4 instead of weigh_particles_slam3 which uses all
%observation (this function is used in the MC_ScanMatch2 function)

%Another thing it differs with is the use of particles_obstacle_cells
%instead of obsatcle vectors which are of different size with each
%particle(because their  maps are different). so SLAM2 was not built well
%at all

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
% build a probability map for each particle with size CellNum_x-by-CellNum_x each cell
% with probability 0.5

% define number of cells and create occupancy grid with designated function
% (it doesnt really need a designated function.. but he code is more tidy)
CellNum_x=400;
CellNum_y=CellNum_x;
initial_particles_map= OccGrid([CellNum_y,CellNum_x],1);
res=0.1; % 0.1 meters per cell [m/cell]

% robots inital pose will be set to be known in the middle with heading
% zero i.e. in the direction of the arrow: -> 
robot_cell_pose = [round(CellNum_x/2),round(CellNum_y/2),0];% if we change the robots headin from 0 to 3*pi/8+pi, we get up orientation
%% Build initial map from scans
% we need to tranform the scans to pixels using Bresenham's line algorithm:
% "https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm"
% [~,~,~,X_ray_cells,Y_ray_cells]=bresenham1(ones(21),[robot_cell_pose_x robot_cell_pose_y ; end_of_ray_x end_of_ray_y],ShowImage=0(no),1 (yes))
% This is a function i have downloaded. It will give us the x and y
% coordinates of the ray. we need to update the cells along the ray
% accordingly

% turn scan ends of each ray, to cartisian coordinates
scans_cart = scan2cart(scans1{1,1}, robot_cell_pose(3));

% now we need to turn the cartisian reading to cell indecies along the
% laser ray
X_cells=[];
Y_cells=[];

for k=1:length(scans_cart)
    % inorder to use Bresenham's line algorithm, we send the pixel(cell) coordinates of
    % the ray's starting point (at the robots pixel whish is the middl of
    % the map) and the end of the ray's cells:
    ray_end_cell=[robot_cell_pose(1)+ round(scans_cart(k,1)/res), robot_cell_pose(2)+round(scans_cart(k,2)/res)];
    % send the function the map, the coordinates, and a flag=0 to not
    % display nothing.
    [~,~,~,X_cells,Y_cells]=bresenham1( initial_particles_map(:,:), [robot_cell_pose(1:2); ray_end_cell] ,0);
    % Verify that the robot's cell corresponds to the first cell in the
    % ray's crdnts. this is easily done in verify_cell_order() just to save
    % space in this script.
    [X_cells,Y_cells,~]=verify_cell_order(robot_cell_pose,X_cells,Y_cells);  
    % % % % % % % % % % % % update initial map % % % % % % % % % % % % % % 
    % Now we want to update each cell along each ray, to be either occupied
    % or free. This is done in the following function using Binary Bayes
    % Filter in the fom of log-odds, just like in the book "probabilistic
    % roboics - S.Thrun". Open the function for further description.
    initial_particles_map(:,:)=update_map2(initial_particles_map(:,:),X_cells,Y_cells);
end 

%% Turn probability maps back to obsticle vector of cartisian coordinates

obstacle_vector=prob_map_to_obsticale_vector( initial_particles_map(:,:),res);

gm_sigma=[0.8 0; 0 0.8];

%% initialize particles
% number of total particles
Np = 1; 

% initialize a map for each of the Np particles, with the initial obsticle
% vector. 
particles_obstacle_cells=cell(Np,1); % Preallocate Memory for Cell Array for better use (we can do without it but its better practice)
particles_obstacle_cells{1}=obstacle_vector;
particles_obstacle_cells(:)=particles_obstacle_cells(1);


% also, initialize a probabiliry map for each particle with th initial map
particles_prob_map=repmat(initial_particles_map, 1, 1, Np);
% all particles are initialized to be in the (0,0) coordinate with heading
% set to 0 radian. the state is = [x,y,theta].
particles_state=zeros(Np,3);
particles_weight=[];


best_map_state(1,:)=particles_state(1,:);
best_map_cell(1,:)=[0,0]; % the cell in the grid map in which the robot is assumed to be in the best map

%%  paricle filtering within a given map.
Num_Of_Sim_steps=length(dt1)-1;  % dt1 is 327 time-steps
% Num_Of_Sim_steps=100;

% define a best map array for each time-step
best_map=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);
best_map(:,:,1)=initial_particles_map;



for t=2:Num_Of_Sim_steps
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

    % move the particles accoding to motion model
    particles_state= move_prtcls( particles_state,omometry_data1(t-1,1),...
                                omometry_data1(t-1,2), dt1(t-1) );
    % do a monte carlo scan match with particles_state as initial
    % conditions. the obstacle vectors inside obstacle_cells are retrived
    % from the previous scan and we match the current scans1{t,1}
    particles_state=MC_ScanMatch2(particles_state, scans1{t,1}...
                               , particles_obstacle_cells);
      
                                
    % Now, we need to update the chosen particles' maps.
    % For each particle:
    for k=1:Np
        % Get its state variables [x y theta]
        prtcl_x=particles_state(k,1); prtcl_y=particles_state(k,2);
        prtcl_theta=particles_state(k,3);
        % Get the scan's cartisian end-points coordinantes with respect to
        % the particle
        scans_cart = scan2cart(scans1{t,1}, prtcl_theta );
        % Get the cell that corresponds to the particle's coordinates
        prtcl_cell = [round(CellNum_x/2+prtcl_x/res), round(CellNum_y/2+prtcl_y/res)];
        % Get the cells along each ray in the scan using Bersenham's
        % alorithm and update these cells accordingly.
        for j=1:length(scans_cart)
            ray_end_cell=[prtcl_cell(1)+round(scans_cart(j,1)/res)...
                          ,prtcl_cell(2)+round(scans_cart(j,2)/res)];
                      
            [~,~,~,X_cells,Y_cells]=bresenham1(particles_prob_map(:,:,k),...
                                               [prtcl_cell(1), prtcl_cell(2)...
                                               ;ray_end_cell],0);
                                               
                                           
            [X_cells,Y_cells,~]=verify_cell_order(prtcl_cell,X_cells,Y_cells);  

            particles_prob_map(:,:,k)=update_map2( particles_prob_map(:,:,k),X_cells,Y_cells);
        end   

        % We need to get the new obticale vector for each updated map.
        particles_obstacle_cells{k}=prob_map_to_obsticale_vector( particles_prob_map(:,:,k),res);
        
        gm = gmdistribution(particles_obstacle_cells{k},gm_sigma);
        particles_weight(k,:)=weigh_particles_slam4( particles_state(k,:)...
                                                     ,gm, scans1{t,1} );
        
    end
    
    %normalize
    particles_weight=particles_weight./sum(particles_weight);
    % we can get the best particle to determine the best map.
    [~,max_weight_indx]=max(particles_weight);
    best_map(:,:,t)=particles_prob_map(:,:,max_weight_indx);
    
    best_map_state(t,:)=particles_state(max_weight_indx,:);
    best_map_cell(t,:)= [round(CellNum_x/2+best_map_state(t,1)/res),...
                     round(CellNum_y/2+best_map_state(t,2)/res)];
    % now we resample indecies according to the weights and re-instate the
    % corresponding particles and maps.
    resampled_indxs=randsample((1:Np),Np,true,particles_weight);
    particles_state = particles_state(resampled_indxs,:);
    particles_prob_map=particles_prob_map(:,:,resampled_indxs);
    
    % % % % % UNCOMENT TO SHOW BEST MAP % % % % % %
    figure(1); clf;                               %
    subplot(1,2,1)                                %
    imagesc(best_map(:,:,t));                     %
    colormap(flipud(gray))                        %
    hold on                                       %
    scatter(best_map_cell(t,2),...                %
            best_map_cell(t,1),'b')               %
    hold off                                      %
    axis square;                                  %
    camroll(180)                                  %
    drawnow                                       %
    pause(0.1)                                    %
    subplot(1,2,2)                                %
    scatter(real_obsticle_vector(:,1),...         &
            real_obsticle_vector(:,2), 1)         %
    hold on                                       %
    % plot real robot location                    %
    plot_robot(X1(t,:)+[10 10 0],0.3)             %
    axis square;                                  %
    hold off                                      %
                                                  %
    % % % % % % % % % % % % % % % % % % % % % % % %
    
end
%% Save and Make Video
% % save the Best_Maps array

t=datetime('now');
DateString = datestr(t);
DateString(12)='_';DateString(3)='_';DateString(7)='_';DateString(15)='_';DateString(18)='_';
save([DateString,'_best_map'], 'best_map');
save([DateString,'best_map_cell'], 'best_map_cell');

% make video
str=['Np=',num2str(Np)];
SLAM_video3( best_map,best_map_cell,real_obsticle_vector,X1,8,str)


