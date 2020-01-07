function [Occupancy_Grid] = OccGrid(grid_size, Np )
% This function gets the grid size as a vector of two integers [y_size, x_size] 
% and outputes a grid of complete uncertainty (p=0.5 probability for each cell) 
y_cells=grid_size(1);
x_cells= grid_size(2);

Occupancy_Grid = 0.5.*ones(y_cells,x_cells,Np);

end

