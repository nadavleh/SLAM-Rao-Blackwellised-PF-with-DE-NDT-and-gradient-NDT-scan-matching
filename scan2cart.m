function [ reading_position ] = scan2cart( scan_cell, robot_heading )
% This function calculates the end points of each laser beam in cartisian 
% coordinates, relative to the robot coordinate system and given its angle 
% i.e. the robot's coordinates system orientation relative to the horizon
% % This function gets a scan cell which contains 1080 range reading, and
% % and te robot's orientation


    ranges = scan_cell.Ranges; %convert the 1080 range reading to a vector
    angle_increment = scan_cell.AngleIncrement; %retrieve the angle incriment
    scan_number=length(ranges); % the number of scans ( should be 1080)

    reading_position = zeros(scan_number,2); 

    for k=1:scan_number
        % aor = angle of reading. angle_increment*(535.5-idx(k)) [rad]
        % calculates angle of rading numbered: idx(k), from the horizon, 
        % to that, we need to add the partical's heading.
        % index 535.5 in "ranges" corresponds to the heading 0 [rad] from
        % the horizon, becaue the reading range is -2.3562 to 2.3562 [rad] 
        % and so -2.3562+angle_increment*535.5 = 0
        aor=mod(robot_heading-angle_increment*(535.5-k),2*pi); 

        % The cartisian coordinae of the ens of each ray is simply: [x, y]=
        % [ray_length*cos(ray_angle), ray_length*sin(ray_angle)]
        reading_position(k,:)= ranges(k)*[ cos(aor) sin(aor)];
    end 
end
