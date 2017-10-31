function discreteMap = discretize(botSim, xnum, ynum)

    %% define map
    map=botSim.getMap();  %default map

    %% discretization
    t = cputime;
 
    %the min and max axis values
    xstart = map(1,1);
    xend = map(1,1);
    ystart = map(1,2);
    yend = map(1,2);

    dim = size(map);
    for i=1:dim(1)
        if map(i,1) < xstart
            xstart = map(i,1);
        end
        if map(i,1) > xend
            xend = map(i,1);
        end
        if map(i,2) < ystart
            ystart = map(i,2);
        end
        if map(i,2) > yend
            yend = map(i,2);
        end
    end

    %the real distance between each node
    xstep = (xend - xstart) / xnum;
    ystep = (yend - ystart) / ynum;

    %array of objects holding information for each node
    nodes(xnum,ynum) = Node;
    %getting map coordinates and determining if coordinate is inside the map
    for i=1:xnum
        for j=1:ynum
            % we subtract 0.5 so that we reside in the middle of the squares
            x = xstart + (i-0.5) * xstep;
            y = ystart + (j-0.5) * ystep;
            % set node pos property
            pos = [x,y];
            nodes(i,j).pos = pos;
            % set node index property
            index = [i,j];
            nodes(i,j).index = index;
            %set node inmap property
            if botSim.pointInsideMap(pos)
                nodes(i,j).inmap = 1;
            else
                nodes(i,j).inmap = 0;
            end
        end
    end
    
    discreteMap = DiscreteMap(xnum, ynum, xstart, ystart, xend, yend, xstep, ystep, nodes);

    e = cputime - t;
    if botSim.debug()
        fprintf("discretization time: %f\n", e);
    end

    %{
    %print inmap info
    for i=xnum:-1:1
        for j=1:ynum
            fprintf("%d ", nodes(j,i).inmap);
        end
        fprintf("\n");
    end
    %}
end
