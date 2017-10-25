xstart = 0;
xend = 5;
ystart = 0;
yend = 5;
xnum = 100;
ynum = 100;

coords = [];
inmap = zeros(xnum,ynum);
for i=1:xnum
    for j=1:ynum
        x = xstart + ((i-1)/xnum) * (xend - xstart);
        y = ystart + ((j-1)/ynum) * (yend - ystart);
        coords = [coords; [x,y]];
        r = rand;
        if r > 0.5
            inmap(i,j) = 1;
        else
            inmap(i,j) = 0;
        end
    end
end

disp(coords);
disp(inmap);
