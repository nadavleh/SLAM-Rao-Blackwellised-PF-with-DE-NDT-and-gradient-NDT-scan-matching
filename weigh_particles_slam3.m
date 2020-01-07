function [ weights ] = weigh_particles_slam3( particles, gm, scan_cell )
% this is different from weigh_particles_slam2 only in the sense that it
% returns the sum "weights(p,:)=sum(vor)" and not the sum of log
% probabilities like before "...=sum(log(vor))"


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
        weights(p,:) = sum(vor);
    end
end









