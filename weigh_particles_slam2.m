function [ weights ] = weigh_particles_slam2( particles, gm, scan_cell )
% This function is a slight modification (really slight) of the one used in
% the PF localization simulations. Here we simply take the particles state,
% attach the scans to it, and evaluate their liklyhood (the GMM) value. We
% showld theoretically muiltiply all reading values for the specific
% particle, howere due to truncation error, we get 0 most of the time so we
% sum their log values (which is equivallent in the Gaussian likelyhood
% case), which is not entirely right (as its a GMM and not a Gaussian)
% however its a good solution to the errors.

%% initialization
    % originally used to avoid the calculation of all readings, however its
    % imperative now to use all readings in the specific time step. Also,
    % the particles number is only 1 in the slam simulations, so we can
    % comment out this section if we want to and set N to be N=1.
% %     L=100; %number of scans used from the available 1080

    [Np,~]=size(particles); %number of particals
    ranges = scan_cell.Ranges; %convert the 1080 range reading to a vector
    angle_increment = scan_cell.AngleIncrement; %retrieve the angle incriment
    scan_number=length(ranges); % the number of scans ( should be 1080)
    reading_position = zeros(scan_number,2); % the reading of the L range scans that we sampled (we sampled their corresponding index)
%% calculation 
    for p=1:Np
        for k=1:scan_number
            % aor = angle of reading. angle_increment*(535.5-idx(k)) [rad]
            % calculates angle of rading numbered: idx(k), from the
            % horizon, to that, we need to add the partical's heading.
            % index 535.5 in "ranges" corresponds to the heading 0 [rad]
            % from the horizon, becaue the eading range is -2.3562 to
            % 2.3562 [rad] and so -2.3562+angle_increment*535.5 = 0
            aor=particles(p,3)-angle_increment*(535.5-k); 

            reading_position(k,:)= [particles(p,1), particles(p,2)] + ranges(k)*[ cos(aor) sin(aor)];
        end 
        % values of reading - evaluated on the maps gmm
        vor = pdf(gm, reading_position)+eps;

        % now we actually need to multiply all this vector's component but
        % instead we will sum theire log values (almost equivillant).
        weights(p,:) = sum(log(vor));
    end
end









