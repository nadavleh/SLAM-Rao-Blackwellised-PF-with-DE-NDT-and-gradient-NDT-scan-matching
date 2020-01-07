function [X_cells,Y_cells,chek_first] = verify_cell_order(robot_cell_pose,X_cells,Y_cells)
%This function gets the robots cell indxs and a laser ray specifed by
%corresponding x and y crdnts along it, and verifies the first cell of the
%ray corresponds to the robot's cell

% because we use the downloaded Bresenham's line algorithm which works with
% images, we get a mixup between the robots cell horizontal index and X,Y
% crdnts the Bresenham1() function outputs (because it works with images
% and not x,y crdnts so it all mixes up). This is why we we compare the
% ray's x index X_cells to the robots y position - robot_cell_pose(2).
chek_firstx =  (X_cells(1)==robot_cell_pose(2));
chek_firsty =  (Y_cells(1)==robot_cell_pose(1));
if chek_firstx==0
    X_cells=fliplr(X_cells);
    if chek_firsty==0
        Y_cells=fliplr(Y_cells);
    end
    chek_firstx =  (X_cells(1)==robot_cell_pose(2));
    chek_firsty =  (Y_cells(1)==robot_cell_pose(1)); 

elseif chek_firsty==0
    Y_cells=fliplr(Y_cells);
    if chek_firstx==0
        X_cells=fliplr(X_cells);
    end
    chek_firstx=  (X_cells(1)==robot_cell_pose(2));
    chek_firsty =  (Y_cells(1)==robot_cell_pose(1)); 
end
    chek_first=[chek_firstx,chek_firsty];
end

