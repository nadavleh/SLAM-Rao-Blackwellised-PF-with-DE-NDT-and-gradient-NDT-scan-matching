    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rot=[1 0; 0 -1];
%     x=-0.5*CellNum_x:0.5*CellNum_x;
    
    x=1:CellNum_x;
    y=x;
    [xx,yy]=meshgrid(x,y);
    A=[xx(:)';yy(:)'];
    B=(rot*A);
    XX_new=reshape(B(1,:),CellNum_x,CellNum_x);
    YY_new=reshape(B(2,:),CellNum_x,CellNum_x)+CellNum_x+1;
    
%     K=best_map(XX_new,YY_new,t);
for i=1:CellNum_x
    for j=1:CellNum_x
        K(i,j)=best_map(XX_new(i),YY_new(j),t-2);
    end
end

    imagesc(K)
%     imagesc(best_map(:,:,t-2))
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%
load wbarb
rot=[1 0; 0 -1];

im = X;

figure(1); clf;
imagesc(im)
colormap gray

[n_rows,n_col] = size(im);

x=1:n_col;
y=x;
im = rot*reshape(im , n_rows*n_col/2, 2)';

im=reshape(im , n_rows,n_col);

figure(2); clf;
imagesc(im)
colormap gray


%%
load wbarb
im = X;
[n_rows,n_col] = size(im);

    rot=[1 0; 0 -1];
%     x=-0.5*CellNum_x:0.5*CellNum_x;
    
    x=1:n_rows;
    y=x;
    [xx,yy]=meshgrid(x,y);
    A=[xx(:)';yy(:)'];
    B=(rot*A);
    XX_new=reshape(B(1,:),n_rows,n_rows);
    YY_new=reshape(B(2,:),n_rows,n_rows)+n_rows+1;
    
%     K=best_map(XX_new,YY_new,t);
for i=1:n_rows
    for j=1:n_rows
        K(i,j)=im(XX_new(i),YY_new(j));
    end
end

    imagesc(K)