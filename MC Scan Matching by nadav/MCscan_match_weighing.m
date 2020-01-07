function [ weights ] = MCscan_match_weighing( samples, gm, scan_cell )


%% Initialization
[Ns,~]=size(samples); %number of samples
ranges = scan_cell.Ranges; %convert the 1080 range reading to a vector
angle_increment = scan_cell.AngleIncrement; %retrieve the angle incriment
scan_number=length(ranges);  % the number of scans ( should be 1080)



L=500; %number of scans used from the available 1080     


reading_position = zeros(L,2);    % the reading of the L range 
                                            % scans that we sampled 
                                            % (we sampled their 
                                            % corresponding index)

% pre-allocation of weights
weights=zeros(Ns,1);                                            
%% calculation 
for p=1:Ns


    % sample L randon indecies
    idx=randsample(scan_number,L);

    for k=1:length(idx)
        % aor = angle of reading. angle_increment*(535.5-idx(k)) [rad]
        % calculates angle of rading numbered: idx(k), from the
        % horizon, to that, we need to add the partical's heading.
        % index 535.5 in "ranges" corresponds to the heading 0 [rad]
        % from the horizon, becaue the eading range is -2.3562 to
        % 2.3562 [rad] and so -2.3562+angle_increment*535.5 = 0
        aor=samples(p,3)-angle_increment*(535.5-idx(k)); 

        reading_position(k,:)= [samples(p,1), samples(p,2)] + ranges(idx(k))*[ cos(aor) sin(aor)];
    end 
    % values of reading - evaluated on the maps gmm
    vor = pdf(gm, reading_position)+eps;

    % now we actually need to multiply all this vector's component but
    % instead we will sum theire log values (almost equivillant).
    weights(p,:) = sum(vor);
end


end









