%% Scan Matching version 
% in the following section i perform scan matching with fmincon()  - a
% matlab gradient based optimizer:
% "https://www.mathworks.com/help/optim/ug/choosing-the-algorithm.html#bsbwxm7"
% the cost-function minimization is set to be the likelyhood of the last
% scan, to which we match the next scan (very very symillar to NDT).

% UNFORTUNATELY!!!!!!!!!!!!!! whenever i set the bounderies of the
% optimization to +- delta around the odometry, the algorithm always choose
% the boundery. i checked that we optimize the correct function and that
% everything is set up right (see example in the "plot to see its all
% calibrated ok" commentt square below)

%  to isolate the problam, i suggest starting with k=210 (discarding the
%  first 210 time steps), threshold=0.9; and Np=100;
% another option is to use a different optimizing function!

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
k=180;
scans1(1:k)=[];
dt1(1:k)=[];
omometry_data1(1:k,:)=[];
X1(1:k,:)=[];
%% initialize particles and maps
% build a probability map with size CellNum_x-by-CellNum_x each cell
% with probability 0.5

% define number of cells and create occupancy grid with designated function
% (it doesnt really need a designated function.. but he code is more tidy)
CellNum_x=300;
CellNum_y=CellNum_x;

initial_particles_map= OccGrid([CellNum_y,CellNum_x],1);
res=0.15; % 0.1 meters per cell [m/cell]

robot_cell_pose = [round(CellNum_x/2),round(CellNum_y/2),0];

%% Build initial map from scans

% turn scan ends of each ray (in the first timestep), to cartisian coordinates
scans_cart = scan2cart(scans1{1,1}, robot_cell_pose(3));
% now we need to turn the cartisian reading to cell indecies along the
% laser ray
X_cells=[];
Y_cells=[];


%%%%%%%%%%%%%%%%%%%% build initial map - this is meyootar %%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Turn probability maps back to obsticle vector of cartisian coordinates

Nm=3; %Number of global maps, each on with Np particels
threshold=0.8; % the threshold to turn the initial prob map to actual obsticales

obstacle_vector=cell(1,Nm); % Preallocate Memory for Cell Array for better use (we can do without it but its better practice)
obstacle_vector{1}=prob_map_to_obsticale_vector( initial_particles_map,res,threshold);
% the threshold to turn the simulation prob maps to, obsticales this needs
% to be abit different than the 0.8 befor, empirically.
% threshold=0.7; 

% all obstacle vectors are the same initially because the prob maps are..
% so we can just duplicate the obstacle vector for all other Nm maps.
% NOTE: when using braces '()' in cell arrays it means refering to the cell
% itself, while using curley braces '{}' refers to the specified cell's
% content
obstacle_vector(:)=obstacle_vector(1);
sigma=[ 0.8,    0;...
         0,   0.8]; 
     
% the following sigma is for scan matcher only:     
sigma2=[ 0.4,    0;...
         0,   0.4];


%% initialize particles
% number of particles per map
Np = 100; 

particles_state=zeros(Np,3,Nm);
particles_weight=zeros(Np,Nm);

%%  paricle filtering within a given map.

prob_map=repmat(initial_particles_map, 1, 1, Nm);

Num_Of_Sim_steps=length(dt1)-1;  % dt1 is 327 time-steps
% Num_Of_Sim_steps=100; 

best_map=zeros(CellNum_x,CellNum_y,Num_Of_Sim_steps);
maps_state=zeros(Num_Of_Sim_steps,3);
maps_cell=zeros(Num_Of_Sim_steps,2);

map_previous_state=zeros(Nm,3); 
% map_previous_state=[0 0 0];

for t=1:Num_Of_Sim_steps
    
%%%%%%%%%%%%%%%%%%%%%%% move particles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:Nm
        % move the particles in each map
        particles_state(:,:,i)= move_prtcls( ...
            particles_state(:,:,i),omometry_data1(t,1),...
            omometry_data1(t,2),dt1(t) );
        
        % weigh the particle in each resampled map
        gm = gmdistribution(obstacle_vector{i},sigma);
        % updates the partical's weights by incorporating the next time
        % step's measurments (LIDAR).
        particles_weight(:,i)  = weigh_particles_slam3( particles_state(:,:,i), gm, scans1{t+1,1} );

        if sum(particles_weight(:,i))==0
            [~,max_idx]=max(particles_weight(:,i));
            particles_weight(max_idx,i)=1;
        else
            particles_weight(:,i)=particles_weight(:,i)/sum(particles_weight(:,i));
        end
        % resample the best particles in each of the resampled maps,
        % inorder to calc their mean arithmetically (and not weightedt
        % mean). The calculated mean we will act as a good initial
        % condition for the scan matcher
        resampled_prtcl_idxs=randsample((1:Np),Np,true,particles_weight(:,i));
        particles_state(:,:,i) = particles_state(resampled_prtcl_idxs,:,i);
        map_mean_state(i,:)=mean(particles_state(:,:,i),1);
               
%%%%%%%%%%%%%%%%%%%%%%%% Scan Match %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        trans_vec=(map_mean_state(i,:)-map_previous_state(i,:)); %tranformation vector
        cur_scan = scan2cart(scans1{t,1}, 0); % let the current scan be the one prior to moving partcls
        % Change next scan to scans1{t+10,1} and plot transformation, to
        % see it actually does the job
        next_scan1=scan2cart(scans1{t+1,1}, 0);
        cur_scan=cur_scan';
        next_scan1=next_scan1';

% % % %  plot to see its all calibrated ok  % % % % % % % % % % % % % % % %
% % Change next scan to scans1{t+10,1} and see it matches nicely          %    
%         next_scan1=scan2cart(scans1{t+20,1}, 0);                        %    
%         next_scan1=next_scan1';                                         %
%         fun = @(x) -sum( pdf(gm, (([cos(x(3)) -sin(x(3));...            %  
%                                     sin(x(3)) cos(x(3))])*...           %      
%                                     (next_scan1)+[x(1);x(2)] )' )+eps); %                  
%         lb = [];                                                        %      
%         ub = [];                                                        %  
%         A = [];                                                         %                      
%         b = [];                                                         %                  
%         Aeq = [];                                                       %              
%         beq = [];                                                       %                                  
%         x0 = trans_vec;                                                 %                                  
%         t_calc = fmincon(fun,x0,A,b,Aeq,beq,lb,ub);                     %                                                                  
%         J=@(x) [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];             %                                          
%         tran_back=@(x) (J(x))*(next_scan1)+[x(1);x(2)];                 %                                          
%         next_scan_calc =   tran_back(t_calc);                           %                      
%         scatter(cur_scan(1,:),cur_scan(2,:),'b')                        %                          
%         hold on                                                         %                      
%         scatter(next_scan1(1,:),next_scan1(2,:),'r')                    %                          
%         hold on                                                         %                              
%         scatter(next_scan_calc(1,:),next_scan_calc(2,:),'c')            %                              
%         hold off                                                        %                      
%         axis square                                                     %                                          
%                                                                         %                      
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
        
        % the likleyhood is based on the current scan (if this were gm as
        % above, it will lead to problems as we saw befor, just comment out
        % the gm2 line to see)
        gm2 = gmdistribution(cur_scan',sigma2);
        fun = @(x) -sum( pdf(gm2, ( ([cos(x(3)) -sin(x(3));...
                                     sin(x(3)) cos(x(3))])*...
                                     (next_scan1)+[x(1);x(2)] )' ));

                                 
% %different tries for bounderies for optimization 
% % no bounderies:
% lb = [];
% ub = []; 
% % dont optimize anything, set the bounderies to the odometry vector:
% lb = [trans_vec(1), trans_vec(2), trans_vec(3)];
% ub = [trans_vec(1), trans_vec(2), trans_vec(3)]; 
% % optimize only the angle: we now see that as we increase the +- value of
% % the boundery we always get the value of the boundery, i still dont know
% % why:
% lb = [trans_vec(1), trans_vec(2), trans_vec(3)-0.5];
% ub = [trans_vec(1), trans_vec(2), trans_vec(3)+0.5];  
% % optimize only the position:
% lb = [trans_vec(1)-0.5, trans_vec(2)-0.5, trans_vec(3)];
% ub = [trans_vec(1)+0.5, trans_vec(2)+0.5, trans_vec(3)];
% % optimize the whole transformation:
lb = [trans_vec(1)-0.1, trans_vec(2)-0.1, trans_vec(3)-0.5];
ub = [trans_vec(1)+0.1, trans_vec(2)+0.1, trans_vec(3)+0.5]; 

        A = [];
        b = [];
        Aeq = [];
        beq = [];     
        x0 = trans_vec; % set initial starting point to be the mean from 
                        % the PF and odometry
                        
        t_calc = fmincon(fun,x0,A,b,Aeq,beq,lb,ub);
        
% % once we have found the transformation vector, which is the detla
% % from the current state to the previous one (t=current-previous),
% % we simply turn all particles to this state by
% % current=previous+delta, though we will try it differently using
% % the transformation and not via:
%  particles_state(:,:,i)=repmat([map_previous_state(i,1:2)+t_calc(1:2),map_previous_state(i,3)+t_calc(3)] ,Np,1);
% % but rather via:
        J=@(x) [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
        prtclsXY=(J(t_calc))*map_previous_state(i,1:2)'+[t_calc(1);t_calc(2)];
        prtclsXY=prtclsXY';
        prtcls_theta=map_previous_state(i,3)+t_calc(3);
        particles_state(:,:,i)=repmat([prtclsXY prtcls_theta],Np,1); 
        
% % different tries for defining the right transformation using only the
% % theta of the optimization or only [tx,ty] may be used:
% particles_state(:,:,i)=repmat([prtclsXY map_previous_state(i,3)],Np,1);
% particles_state(:,:,i)=repmat([map_mean_state(i,1:2) prtcls_theta],Np,1);  
% particles_state(:,:,i)=repmat(map_mean_state(i,:),Np,1);
% particles_state(:,:,i)=repmat(map_previous_state(i,:)-t_calc,Np,1);
% particles_state(:,:,i)=repmat([map_previous_state(i,1:2)+t_calc(1:2),map_previous_state(i,3)+t_calc(3)] ,Np,1);
% % dont set all partcls to the cac but rather move them around the new
% % calulated mean
% particles_state(:,:,i)=particles_state(:,:,i) + repmat([t_calc(1),t_calc(2),t_calc(3)],Np,1);

        % update the previous state to the current one for next timestep
        map_previous_state(i,:)=[prtclsXY prtcls_theta];
%         map_previous_state(i,:)=particles_state(1,:,i);

        % the error between the odometry and the calculation from fmincon()
        norm(trans_vec-t_calc)
      
% % % % % % % % % % % % % % % % test  % % % % % % % % % % % % % % % % % % % % %
% % test to see why the fuck it doesnt match as good as the example above     %
% % and starting the second timestep it diverges                              %
%     figure(2); clf;                                                         %
%     J=@(x) [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];                     %
%     tran_back=@(x) (J(x))*(next_scan1)+[x(1);x(2)];                         %
%     next_scan_calc =   tran_back(t_calc);                                   %
%     scatter(cur_scan(1,:),cur_scan(2,:),0.5,'b')                            %
%     hold on                                                                 %
%     scatter(next_scan1(1,:),next_scan1(2,:),0.5,'r')                        %
%     hold on                                                                 %
%     scatter(next_scan_calc(1,:),next_scan_calc(2,:),0.5,'c')                %
%     hold off                                                                %
%     axis square                                                             %
%     zoom(1.5)                                                               %
%     title('current scan (b), next scan (r), next to current ref-frame (c)') %
%     drawnow                                                                 %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


              
%%%%%%%%%%%%%%%%%%%%%%%%%%%% update maps %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Get mean state variables [x y theta]
        mean_x=particles_state(1,1,i); mean_y=particles_state(1,2,i);
        mean_theta=particles_state(1,3,i);
        
% % if i want to update the map according to the PF without optimization
% % just set the partcls position to the mean, but dont forget to do:
% % "particles_state(:,:,i)=repmat(map_mean_state(i,:),Np,1)" above, or
% % alternatively, resample!
%  mean_x=map_mean_state(i,1); mean_y=map_mean_state(i,2);
%  mean_theta=map_mean_state(i,3);
% % or:
%  mean_x=map_previous_state(i,1); mean_y=map_previous_state(i,2);
%  mean_theta=map_previous_state(i,3);
       

        % Get the next time step scan's cartisian end-points coordinantes with respect to
        % the mean, inorder to build the next time step's map
        scans_cart = scan2cart(scans1{t+1,1}, mean_theta );
        % Get the cell that corresponds to the particle's mean coordinates
        prtcl_cell = [round(CellNum_x/2+mean_x/res), round(CellNum_y/2+mean_y/res)];
        
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% now we can update each of the resampled maps, with the robot          %
% position assumed in the mean state of each maps' particals.           %
% To avoid the numerical overhead accompanied by updateing all          %
% scans from scans_cart, we randomly choose L rays, and update          %
% according to them only.                                               %
L=500; %number of scans used from the available 1080                    %
scan_number=length(scans_cart); % the number of scans (should be 1080)  %
idx=randsample(scan_number,L);                                          %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
        % Update the map based on all 1080 lidar readings (comment out the
        % box above)
%         idx=1:1080;

        % Get the cells along each chosen ray, using Bersenham's
        % alorithm, and update these cells accordingly.
        for j=1:length(idx)
            [~,~,~,X_cells,Y_cells]=bresenham1(prob_map(:,:,i),...
                [prtcl_cell(1), prtcl_cell(2) ;...
                prtcl_cell(1)+round(scans_cart(idx(j),1)/res),...
                prtcl_cell(2)+round(scans_cart(idx(j),2)/res)],0);
            % Verify that the particl's cell corresponds to the first cell in the
            % ray's crdnts. this is easily done in verify_cell_order().
            [X_cells,Y_cells,~]=verify_cell_order(prtcl_cell,X_cells,Y_cells);
            %  update the probability map along the ray
            prob_map(:,:,i)=update_map2( prob_map(:,:,i),X_cells,Y_cells);
        end   
        % We need to get the new obticale vector for each updated map.
        obstacle_vector{i}=prob_map_to_obsticale_vector( prob_map(:,:,i),res);
    end
    
%%%%%%%%%%%%%%%%%%%%% weigh and resample maps %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for m=1:Nm
    % Build a likelyhood function from new obstacle vector
        % empirical cov-matrix, can be changed with trial and error.
        sigma=[0.8 0; 0 0.8]; 
        gm = gmdistribution(obstacle_vector{m},sigma);

    % weigh the particles
        particles_weight(:,m)  = weigh_particles_slam3(...
            particles_state(:,:,m), gm, scans1{t,1} );
        
        if sum(particles_weight(:,m))==0
            [~,max_idx]=max(particles_weight(:,m));
            particles_weight(max_idx,m)=1;
        else
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
    % the state of the robot (particles) within that map
    maps_state(t,:)=particles_state(1,:,max_weight_indx);    
    % the cell of the robot corresponding to the map's mean state 
    maps_cell(t,:)=robot_cell(maps_state(t,:),res,CellNum_x);    
    % resample the maps
    resampled_maps_indxs=randsample((1:Nm),Nm,true,map_weights);       
    % duplicate resampled maps and particles 
    particles_state = particles_state(:,:,resampled_maps_indxs);
    prob_map=prob_map(:,:,resampled_maps_indxs);
    obstacle_vector(:)=obstacle_vector(resampled_maps_indxs);
    
    

    
    
    
%%%%%%%%%%%%%%%%%%%%%%%% plot Robot and map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    figure(1);    
    clf; 
    % the image of best_map is tranformed upside-down so it is more nice to
    % view. this is down with a simple linear transformation
    tform = maketform('affine',[-1 0 0; 0 -1 0; 0 0 1]);
    J = imtransform(best_map(:,:,t),tform);                                         
    subplot(1,3,1)                                                  
    imagesc(J)                                        
    colormap(flipud(gray))                                          
    hold on                                                         
%     scatter(maps_cell(t,2),maps_cell(t,1),'b')   
    plot_robot([maps_cell(t,2),maps_cell(t,1),maps_state(t,3)+pi/2],2)
    hold off                                                        
    axis square;                                                    
    zoom(2)
    str=['Nm=',num2str(Nm),' Np=',num2str(Np), ' fmincon (gradient based) Scan Match'];
    title(str)
    subplot(1,3,2)                                                  
    scatter(real_obsticle_vector(:,1),real_obsticle_vector(:,2), 1) 
    hold on                                                         
    % plot real robot location                                      
    plot_robot(X1(t,:)+[10 10 0],0.3)                               
    axis square;                                                    
    hold off  
    drawnow
    subplot(1,3,3)  
    J=@(x) [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];                    
    tran_back=@(x) (J(x))*(next_scan1)+[x(1);x(2)];                         
    next_scan_calc =   tran_back(t_calc);
    scatter(cur_scan(1,:),cur_scan(2,:),0.5,'b')                            
    hold on                                                                 
    scatter(next_scan1(1,:),next_scan1(2,:),0.5,'r')                        
    hold on                                                                 
    scatter(next_scan_calc(1,:),next_scan_calc(2,:),0.5,'c')                
    hold off                                                                
    axis square                                                             
    title('current scan (b), next scan (r), next to current ref-frame (c)')
    drawnow
    pause(0.02)                                                     
    F(t)=getframe(gcf);    
    
%%%%%%%%%%%%%%%%%%%%% display simulation progress %%%%%%%%%%%%%%%%%%%%%%%%%
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
    
end  


%% Save and Make Video
% % save the Best_Maps array
% t=datetime('now');
% DateString = datestr(t);
% DateString(12)='_';DateString(3)='_';DateString(7)='_';DateString(15)='_';DateString(18)='_';
% save([DateString,'_best_map'], 'best_map');
% save([DateString,'_maps_cell'], 'maps_cell');
% % make video
% str=['Nm=',num2str(Nm),' Np=',num2str(Np), ' fmincon (gradient based) Scan Match'];
% video = VideoWriter(['SLAM_',DateString]);
% video.FrameRate = 8;
% open(video)
% writeVideo(video,F(10:end));
% close(video)

