function [prob_posterior_map] = update_map2(prob_prior_map,x_crdnts,y_crdnts)
%update_map updates an occupancy probability map, according to a LIDAR
%reading along some pixels (cells). update_map2() differs from update_map()
%only in inxs order (we swaped the order log_odds_posterior(x_crdnts(i),y_crdnts(i))
%by log_odds_posterior(y_crdnts(i),x_crdnts(i))) Which made is the correct
%way. 
%       We will update each cell along the ray (the cell's indecies are
%       specified in the x_crdnts,y_crdnts vectord). It is done using the
%       log-odds ratio seen in the book Probabilistic Robotics (2005, Mit Press)
%       or the thesis: "Localization and Mapping with Autonomous Robots" by Benjamin Mader
%       namely the algorithm goes as follows: (l is short for
%       log-odds-ratio)
%    for all cells mi do
%       if mi in the perceptual eld of zt then
%           l_t,i = l_t-1,i + inverse sensor model - l_0
%       else
%           l_t,i = l_t-1,i
%       end if
%    end for
%    return l_t,i
%       
%       The log-odds ration is defined as l_t,i=log( p(cell_i=occupied)/p(cell_i=free))
%       because "free" is the complementry of occupied it turns to:
%       l_t,i=log( p(cell_i=occupied)/(1-p(cell_i=free)) ). Further more
%       l_0 is the prior, which is l_0=log(0.5/0.5)=log(1)=0, and so we
%       just ignore it in this function.
%       We empirically set  p(cell_i=occupied)=0.8. and can change it to
%       experiment.
%       The inverse sensor model is simply the method y which we determine
%       if a cell is considered free or not along the ray. Here, we
%       consider each cell along he ray as free, unless its the edge of he
%       reading, where the ray is stopped by an obsticle and the
%       corresponding cell is considered occupied.

    % Get the map's size
    L=size(prob_prior_map);
    
    % initialize log-odds matrices
    log_odds_prior = log(prob_prior_map./(ones(L)-prob_prior_map));
    log_odds_posterior=log_odds_prior;
    % set emprical probabilites
    p_occupied=0.8;
    p_free=1-p_occupied;
    
    % update each cell along the ray according to the Binary log-odds Bayes
    % filter and inverse sensor model, explained above.
    for i=1:length(x_crdnts)
        % if this is the last coordinate, update according to occupied cell.
        if i==length(x_crdnts)
            log_odds_posterior(y_crdnts(i),x_crdnts(i)) =  log_odds_prior(y_crdnts(i),x_crdnts(i)) + log(p_occupied/p_free);
        else
            log_odds_posterior(y_crdnts(i),x_crdnts(i)) =  log_odds_prior(y_crdnts(i),x_crdnts(i)) + log(p_free/p_occupied);
        end 
%         prob_posterior_map=ones(L) - ones(L)./(ones(L)+exp(log_odds_posterior));
%         imagesc(prob_posterior_map)
%         colormap(flipud(gray))
    end
    
    % we want to get back the probabilities of each cell and not the
    % log-odds ratios, so we an turn back each pixel according to:
    % p(cell_i)=1-1/(1+exp(l_t,i))
    prob_posterior_map=ones(L) - ones(L)./(ones(L)+exp(log_odds_posterior));

end

