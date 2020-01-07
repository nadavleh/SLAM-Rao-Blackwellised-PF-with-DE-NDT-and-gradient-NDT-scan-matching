function [particles_state] = MC_ScanMatch2(particles_state,scan_cell, obstacle_cells)
%MC_ScanMatch2 stands for Monre-Carlo Scan Matcher. 
%The function matches the previous obstacle vector to a recent scan.
%It Differs from MC_ScanMatch b utilizing only L random scans instead of
%all readings of scan_cell and thus saves some run time

%Another thing it differs with is the use of particles_obstacle_cells
%instead of obsatcle vectors which are of different size with each
%particle(because their  maps are different). so MC_ScanMatch was not built well
%at all

% In this function we send the particles states gien by a matrix Np-by-3
% and an obstacle vector of x,y oordinates, given by a cell array of
% Np-by-1 (each entry is the particle's obstacle vec), and the scans cell.
% The function generates a Gussian and samples Ns times from it. then it
% weighs each sample, resamples according to the weights and calculates the
% mean. it generates a new gaussian around that mean and samples-resamples
% again. it does so until two consecutive means differs by no more than a
% pre-determined tol.

    [Np,~]=size(particles_state); %number of particals
    ranges = scan_cell.Ranges; %convert the 1080 range reading to a vector
    angle_increment = scan_cell.AngleIncrement; %retrieve the angle incriment
    scan_number=length(ranges); % the number of scans ( should be 1080)
    
    Ns=100; % number of samples to be used in each itteration
    
    gm_sigma=[0.8 0; 0 0.8]; % the covariance matrix of each of the GMM
                             % componnents in the maps likelyhood GMM 
    
    
    Gaussian_sigma=[0.01 0 0;  % the covariance matrix of the gaussian we 
                    0 0.01 0;  % will sample from in each itteration
                    0 0 0.05];

    weights=zeros(Ns,1); % the weights of the samples in each itteration
                         % (not the weight of the particle!)
    
    max_iter=1000;                   
    % Scan match for all particles                     
    for p=1:Np
        
        % set a tollerance convergence criterion. tol=0.1 is good
%         tol=0.01;
        tol=10;
        % pre allocate space for the state of the samples (my be redundant)
        samples=zeros(Ns,3);
        % creat the GMM of the map (the likelyhood function)
        gm = gmdistribution(obstacle_cells{p},gm_sigma);
        % initialize all samples at the state of the particle
        samples=repmat(particles_state(p,:),Ns,1);
        % initialize the mean of the particles (they are all the same as
        % the particles state)
        current_mean=particles_state(p,:);
        % initialize error to be surely greater than any tol value we might
        % choose
        err=10;
        % set an itteration counter
        counter=1;
       while err>tol && counter<max_iter
           counter=counter+1;
           % initialize the last step's mean to determine convergence at
           % the end of the loop
           last_mean=current_mean;
           % sample Ns times from the 3D Gaussian [x y theta], with its
           % mean set as the last_mean and with the empirical cov-matrix
           % listed above
           samples = mvnrnd(last_mean,Gaussian_sigma,Ns);
           % weigh the samples according to the likelyhiid gm 
           weights=MCscan_match_weighing( samples, gm, scan_cell );
           % normalize the weights
           weights=weights./sum(weights);
           % re-sample indecies with corresponding weights
           resampled_indxs=randsample((1:Ns),Ns,true,weights);
           % update sample-set according to the re-sample indecies.
           samples=samples(resampled_indxs,:);
           % calc the mean of the re-sampled samples
           current_mean=mean(samples,1);
           % calc the error by the norm of the two recent estimates
           % difference
           err=norm(current_mean-last_mean);
           % If the procedure doesnt converge within 500 consecutive steps,
           % update the tollerance to be more forgiving
           if mod(counter,500) == 0
               tol=tol*1.5;
           end
           % Show error after ten itterations
           if mod(counter,10) == 0
               disp(err);
           end
                     
       end
       % after convergence, set the state to be the calculated mean
       particles_state(p,:)=current_mean;

    end
    
    
    
    
    
    
    
    
    
end

