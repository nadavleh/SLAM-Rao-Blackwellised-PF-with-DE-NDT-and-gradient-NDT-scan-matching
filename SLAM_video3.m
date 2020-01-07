function [  ] = SLAM_video3( maps_over_time,maps_cell,real_obsticle_vector,X1,frame_rate,metadata_str)
%This version of SLAM_video (no. 3) basically differs from the previous
%versions, in the number of inputs which results in a more informatic
%output and nicer video.
% maps_over_time  - a 3D array of the best probability map at each timestep
% maps_cell  - a the corresponding cell from which the observations are
% assumed to be taken, i.e. this is the localized robot cell at each time,
% correspondind to Particle filter localization from the same time's
% probability map.
% real_obsticle_vector - is the obtsacle vector retrived from the genuine
% map, and is needed only to plot the robot's real position relative to the
% map
% X1 - is robot 1's actual state over time
% frame_rate - is the frame rate at which the video is outputed
% metadata_str - is a string that sates the number of maps Nm, the number
% of particles per map Np and the number 'L' of random scans we use from
% the scans at each time step (only to alleviate the computational load).


t=datetime('now');
DateString = datestr(t);
DateString(12)='_';DateString(3)='_';DateString(7)='_';DateString(15)='_';DateString(18)='_';
[~,~,time_steps]=size(maps_over_time);

    for t = 1:time_steps

        figure(1); clf;                                                 
        subplot(1,2,1) 
        imagesc(maps_over_time(:,:,t))                                        
        colormap(flipud(gray))                                          
        hold on                                                         
        % Usually in scatter() or plot() we provide the aruments plot(X,Y).
        % Here however we plot(Y_robot,X_robot) i.e. the opposit order.
        % This is due o the fact that we scatter the point againt the
        % imagesec() figure which somehow replaces the directions.
        scatter(maps_cell(t,2),maps_cell(t,1),'b')  
        xlabel(metadata_str)
        hold off                                                        
        axis square;                                                    
        camroll(180)                                                    
        drawnow      
        
        subplot(1,2,2) 
        % plot real map. scatter(~,~, 1)  - the 1 at the end referes to the
        % marker size.
        scatter(real_obsticle_vector(:,1),real_obsticle_vector(:,2), 1) 
        hold on                                                         
        % plot real robot location. we add [10 10 0] for correction to the
        % true location
        plot_robot(X1(t,:)+[10 10 0],0.3)                               
        axis square;                                                    
        hold off                                                        
        pause(0.02)                                                     
        F(t)=getframe(gcf); 

    end
%% Make Video
video = VideoWriter(['SLAM_',DateString]);%,'.mp4']);
video.FrameRate = frame_rate;
open(video)
writeVideo(video,F);
close(video)

end

