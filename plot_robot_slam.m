function [robot_cell]=robot_cell(pos,res,CellNum_x)         
    % robot's cell
    robot_cell=[round(CellNum_x/2+pos(1)/res), round(CellNum_x/2+pos(2)/res)];
 


end  