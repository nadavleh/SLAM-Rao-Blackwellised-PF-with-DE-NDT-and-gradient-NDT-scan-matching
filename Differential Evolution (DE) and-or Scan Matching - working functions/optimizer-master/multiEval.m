function fitness = multiEval(func, inds)
para =  num2cell(inds,2);
% fitness = cellfun(func, para,'uniformoutput',false);
% i changed : fitness = cellfun(func, para); because i got error "Error using cellfun
% Non-scalar in Uniform output, at index 1, output 1.  Set 'UniformOutput'
% to false" so i looked online and found
% "https://www.mathworks.com/matlabcentral/answers/104956-error-using-arrayfun-non-scalar-in-uniform-output"
fitness = cellfun(func, para);