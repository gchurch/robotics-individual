function path = astartest(botSim, xnum, ynum, start, target)

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

    %% algorithm
    %set the starting and target positions
    width = xend - xstart;
    height = yend - ystart;
    startPos = start;
    targetPos = target;
    %fprintf("start position: (%.1f,%.1f)\n", startPos(1), startPos(2));
    %fprintf("target position: (%.1f,%.1f)\n", targetPos(1), targetPos(2));
   

    %find the closest nodes to the start and target positions
    startNode = findClosestNode(xnum, ynum, xstart, xstep, ystart, ystep, startPos);
    targetNode = findClosestNode(xnum, ynum, xstart,xstep, ystart, ystep, targetPos);
    startNodePos = nodes(startNode(1), startNode(2)).pos;
    targetNodePos = nodes(targetNode(1), targetNode(2)).pos;
    if botSim.debug()
        fprintf("start node: (%d,%d)\n", startNode(1), startNode(2));
        fprintf("start node pos: (%.1f, %.1f)\n", startNodePos(1), startNodePos(2));
        fprintf("target node: (%d,%d)\n", targetNode(1), targetNode(2));
        fprintf("target node pos: (%.1f, %.1f)\n", targetNodePos(1), targetNodePos(2));
    end
    %run the A* search algorithm to find the best path from the start node to
    %the target node
    path = astarSearch(botSim, xnum, ynum, nodes, startNode, targetNode);

    %% draw map, bot, and path
    dims = size(path);
    for i=1:dims(1)
        pos = path(i,:);
        plot(pos(1),pos(2),'*');
    end
end

%% functions
%Performs A* search on the graph nodes from startNode to targetNode
function path = astarSearch(botSim, xnum, ynum, nodes, startNode, targetNode)
    t = cputime;

    %calculate all node heuristic values
    for i=1:xnum
        for j=1:ynum
            nodes(i,j).calculateHeuristic(targetNode);
        end
    end
    
    closedList = [];
    openList = [];

    %nodes conatain the index followed by the g and f cost
    currentNode = [startNode 0 0];
    closedList = [closedList; currentNode];

    %iterate
    its = 0;
    while ~(currentNode(1) == targetNode(1) && currentNode(2) == targetNode(2))
        its = its + 1;
        % get new nodes
        n = newNodeCosts(closedList, openList, nodes, xnum, ynum, currentNode);
        ndim = size(n);

        % add the nodes to the open list
        for i=1:ndim(1)
            openList = addToOpenList(openList, n(i,:));
        end
        %not sorting by column 3 properly
        openList = sortrows(openList,[4,3],{'ascend','descend'});
    
        % get new current node
        currentNode = openList(1,:);
        openList(1,:) = [];
        closedList = [closedList; currentNode];
    end
    
    % the path that the algorithm finds
    path = constructPath(nodes, closedList);
    finalNode = path(end,:);
    
    e = cputime - t;
    
    if botSim.debug()
        fprintf("path length: %d\n", closedList(end,4));
        fprintf("iterations: %d\n", its);
        fprintf("A* search algorithm time: %f\n", e);
    end
end

% Return new nodes along with their corresponding cost
function newNodes = newNodeCosts(closedList, openList, nodes, xnum, ynum, currentNode)
    newNodes = [];
    %iterate through all neighbouring nodes
    for i=-1:1
        for j=-1:1
            offset = [i,j];
            index = [currentNode(1),currentNode(2)];
            %the index of the neighbouring node
            newIndex = index + offset;
            %check that the node is not in the closed list
            if ~(i == 0 && j == 0) && ~inClosedList(closedList, newIndex)
                %check that the node indexes are in bounds
                if newIndex(1) > 0 && newIndex(2) > 0 && newIndex(1) <= xnum && newIndex(2) <= ynum
                    %check that the node is in the map
                    if nodes(newIndex(1),newIndex(2)).inmap
                        %calculate the g cost
                        if sum(abs(offset)) == 2
                            g = currentNode(3) + 14;
                        else
                            g = currentNode(3) + 10;
                        end
                        %f = g + h
                        f = g + nodes(newIndex(1),newIndex(2)).h;
                        %add node the list of new nodes
                        newEntry = [(index + [i,j]) g f];
                        newNodes = [newNodes; newEntry];
                    end
                end
            end
        end
    end
end

%function to check is a given node is already in the closed list
function bool = inClosedList(closedList, node)
    bool = 0;
    dims = size(closedList);
    for i=1:dims(1)
        row = closedList(i,:);
        if node(1) == row(1) && node(2) == row(2)
            bool = 1;
        end
    end
end

%function to return the new open list by adding an entry to the old one
function newOpenList = addToOpenList(openList, newEntry)
    newOpenList = openList;
    dims = size(newOpenList);
    c = dims(1);
    bool = 0;
    %check if the new entries indexes are already in the openlist
    for i=1:c
        row = newOpenList(i,:);
        %if already in the open list update the cost if it is less
        if newEntry(1) == row(1) && newEntry(2) == row(2)
            bool = 1;
            if newEntry(4) < row(4)
                newOpenList(i,3) = newEntry(3);
                newOpenList(i,4) = newEntry(4);
            end
        end
    end
    %if not in the open list then add to the end
    if bool == 0
        newOpenList = [newOpenList; newEntry];
    end
end

function path = constructPath(nodes, closedPath)
    path = [];
    last = closedPath(end,:);
    pos = nodes(last(1),last(2)).pos;
    path = [pos;path];
    dims = size(closedPath);
    for i=dims(1):-1:1
        row = closedPath(i,:);
        if (row(3) == (last(3) - 10)) || (row(3) == (last(3) - 14))
            for j=-1:1
                for k=-1:1
                    if ~(j == 0 && k == 0) && (row(1) == last(1) + j) && (row(2) == last(2) + k)
                        last = row;
                        pos = nodes(last(1),last(2)).pos;
                        path = [pos;path];
                    end
                end
            end
        end
    end
end

function indexes = findClosestNode(xnum, ynum, xstart, xstep, ystart, ystep, pos)
    xrelative = pos(1) - xstart + (xstep / 2);
    yrelative = pos(2) - ystart + (ystep / 2);
    closesti = round(xrelative / xstep);
    closestj = round(yrelative / ystep);
    if closesti <= 0
        closesti = 1;
    end
    if closesti > xnum
        closesti = xnum;
    end
    if closestj <= 0
        closestj = 1;
    end
    if closestj > ynum
        closestj = ynum;
    end
    indexes = [closesti, closestj];
end
