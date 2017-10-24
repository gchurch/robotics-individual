xstart = 0;
xend = 5;
ystart = 0;
yend = 5;
xdiff = 1;
ydiff = 1;

coords = [];
for i=xstart:xdiff:xend
    for j=ystart:ydiff:yend
        fprintf("(%d,%d)\n",i,j);
    end
end