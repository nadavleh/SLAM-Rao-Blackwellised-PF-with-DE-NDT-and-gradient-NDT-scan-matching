function [ prticle_array ] = move_prtcls( prticle_array,v,w,dt )
% This function is used in the PF simulations for localization only aswell as
% is the SLAM simulations.
%   it receives an array of particle's states where each row is the
%   corresponding particles state e.g. prticle_array(j,:)= [x_j y_j theta_j]
%   where j=1:N (N number of particles). We will move he particles state
%   according to the motion model and add Gaussian noise to the [v,w]
%   odometry readings.

    % There is no particular reason this array is transposed, this function
    % was written as a first try and worked well, so no ellegant
    % modificaions were made.
    prticle_array=prticle_array';
    % exctract number of particles N.
    [~, N]=size(prticle_array);
    
    std_v = 0.8;
    std_w = 0.1;
    for i=1:N
      % X:
      prticle_array(1,i)=prticle_array(1,i)+(v+normrnd(0,std_v))*dt*cos(prticle_array(3,i));
      % Y:
      prticle_array(2,i)=prticle_array(2,i)+(v+normrnd(0,std_v))*dt*sin(prticle_array(3,i));
      % Theta:
      prticle_array(3,i)=prticle_array(3,i)+(w+normrnd(0,std_w))*dt; 
    end 
    prticle_array=prticle_array';
end

