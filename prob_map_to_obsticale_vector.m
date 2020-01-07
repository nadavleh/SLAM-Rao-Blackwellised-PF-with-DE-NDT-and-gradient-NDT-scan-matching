function [obsticle_vector] = prob_map_to_obstacle_vector(prob_map,res,threshold)

%% convert map to vector 
[y_size,x_size] = size(prob_map);
if nargin<3
    threshold = 0.7;
end

% find the indecies in the probability map whose cell values are great or
% equal to the threshold
[row,col] = find(prob_map>=threshold);
obsticle_vector=[row,col];
% now, make the zeroth coordinate at the middle of the map
obsticle_vector=obsticle_vector-round(y_size/2)*ones(length(row),2);
% turn back from indecies to meters using the resolution input res
% [meters/cell_index]
obsticle_vector=obsticle_vector.*res;

end

