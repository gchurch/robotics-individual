function path = findPath(botSim, discreteMap, start, target)
 

    %% algorithm
    %set the starting and target positions
    startPos = start;
    targetPos = target;
    %fprintf("start position: (%.1f,%.1f)\n", startPos(1), startPos(2));
    %fprintf("target position: (%.1f,%.1f)\n", targetPos(1), targetPos(2));
   
    %find the closest nodes to the start and target positions
    startNode = discreteMap.findClosestNode(startPos);
    targetNode = discreteMap.findClosestNode(targetPos);
    %find the actual coordinates associated with those nodes
    startNodePos = discreteMap.nodes(startNode(1), startNode(2)).pos;
    targetNodePos = discreteMap.nodes(targetNode(1), targetNode(2)).pos;
    %if in debug mode then print information
    if botSim.debug()
        fprintf("start node: (%d,%d)\n", startNode(1), startNode(2));
        fprintf("start node pos: (%.1f, %.1f)\n", startNodePos(1), startNodePos(2));
        fprintf("target node: (%d,%d)\n", targetNode(1), targetNode(2));
        fprintf("target node pos: (%.1f, %.1f)\n", targetNodePos(1), targetNodePos(2));
    end
    
    %run the A* search algorithm to find the best path from the start node to the target node
    path = astarSearch(botSim, discreteMap, startNode, targetNode);

    %% draw map, bot, and path
    if botSim.debug()
        dims = size(path);
        for i=1:dims(1)
            nodeInfo = path(i,:);
            node = [nodeInfo(1),nodeInfo(2)];
            pos = discreteMap.nodes(node(1),node(2)).pos;
            plot(pos(1),pos(2),'*');
        end
    end
end

%% functions
%Performs A* search on the graph nodes from startNode to targetNode
function path = astarSearch(botSim, discreteMap, startNode, targetNode)
    t = cputime;

    %calculate all node heuristic values
    for i=1:discreteMap.xnum
        for j=1:discreteMap.ynum
            discreteMap.nodes(i,j).calculateHeuristic(targetNode);
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
        n = newNodeCosts(closedList, openList, discreteMap, currentNode);
        ndim = size(n);

        % add the nodes to the open list
        for i=1:ndim(1)
            openList = addToOpenList(openList, n(i,:));
        end
        %not sorting by column 3 properly
        dimOpenList = size(openList);
        if dimOpenList(1) == 0
            path = [];
            return;
        end
        openList = sortrows(openList,[4,3],{'ascend','descend'});
    
        % get new current node
        currentNode = openList(1,:);
        openList(1,:) = [];
        closedList = [closedList; currentNode];
    end
    
    % the path that the algorithm finds
    path = constructPath(discreteMap.nodes, closedList);
    finalNode = path(end,:);
    
    e = cputime - t;
    
    if botSim.debug()
        fprintf("path length: %d\n", closedList(end,4));
        fprintf("iterations: %d\n", its);
        fprintf("A* search algorithm time: %f\n", e);
    end
end

% Return new nodes along with their corresponding cost
function newNodes = newNodeCosts(closedList, openList, discreteMap, currentNode)
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
                if newIndex(1) > 0 && newIndex(2) > 0 && newIndex(1) <= discreteMap.xnum && newIndex(2) <= discreteMap.ynum
                    %check that the node is in the map
                    if discreteMap.nodes(newIndex(1),newIndex(2)).inmap
                        %calculate the g cost
                        if sum(abs(offset)) == 2
                            g = currentNode(3) + 14;
                        else
                            g = currentNode(3) + 10;
                        end
                        %f = g + h
                        f = g + discreteMap.nodes(newIndex(1),newIndex(2)).h;
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
    path = [last;path];
    dims = size(closedPath);
    for i=dims(1):-1:1
        row = closedPath(i,:);
        if (row(3) == (last(3) - 10)) || (row(3) == (last(3) - 14))
            for j=-1:1
                for k=-1:1
                    if ~(j == 0 && k == 0) && (row(1) == last(1) + j) && (row(2) == last(2) + k)
                        last = row;
                        path = [last;path];
                    end
                end
            end
        end
    end
end
