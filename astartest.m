%% define map
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
inpolygonMapformatX = cat(1,map(:,1), map(1,1));
inpolygonMapformatY = cat(1,map(:,2), map(1,2));

%% discretization

%how many steps in the x and y axis
xnum = 20;
ynum = 20;
fprintf("grid size: %d x %d\n", xnum, ynum);

dim = size(map);

xstart = map(1,1);
xend = map(1,1);
ystart = map(1,2);
yend = map(1,2);

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

%{
fprintf("TL: (%f,%f), BR: (%f,%f)\n", xstart, ystart, xend, yend);
xstep = (xend - xstart) / xnum;
ystep = (yend - ystart) / ynum;
fprintf("x step: %f\n", xstep);
fprintf("y step: %f\n", ystep);
%}

nodes(xnum,ynum) = Node;
inmap = zeros(xnum,ynum);
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
        if inpolygon(pos(1),pos(2),inpolygonMapformatX,inpolygonMapformatY) == 1
            nodes(i,j).inmap = 1;
        else
            nodes(i,j).inmap = 0;
        end
    end
end

%% displaying grip info
%{
%printing the coordinates
for i=1:xnum
    for j=1:ynum
        pos = nodes(i,j).pos;
        disp(pos);
    end
end

%print inmap info
for i=1:xnum
    for j=1:ynum
        fprintf("%d ", nodes(i,j).inmap);
    end
    fprintf("\n");
end
%}

%% create bot and draw map
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
%set the bots position to the first coordinate
botSim.setBotPos(nodes(1,1).pos);
%set the target position to the last coordinate
target = [nodes(xnum,ynum).pos];
%draw map
botSim.drawMap();
botSim.drawBot(10,'g');
plot(target(1),target(2),'*');
drawnow;

%% algorithm
startPos = botSim.getBotPos();
fprintf("start pos: (%f,%f)\n", startPos(1), startPos(2));
fprintf("target pos: (%f,%f)\n", target(1), target(2));

%get the start node
startNode = [1,1];
targetNode = [xnum,ynum];
fprintf("start node: (%d,%d)\n", startNode(1), startNode(2));
fprintf("target node: (%d,%d)\n", targetNode(1), targetNode(2));

%calculate all node heuristic values
for i=1:xnum
    for j=1:ynum
        nodes(i,j).calculateHeuristic(targetNode);
    end
end

closedList = [];
openList = [];

%nodes conatain the index followed by the f cost
currentNode = [startNode 0];
closedList = [closedList; currentNode];

%iterate
for it=1:3
    % get new nodes
    n = newNodeCosts(closedList, openList, nodes, xnum, ynum, currentNode);
    ndim = size(n);

    % add the nodes to the open list
    for i=1:ndim(1)
        openList = addToOpenList(openList, n(i,:));
    end
    openList = sortrows(openList,3);

    % get new current node
    currentNode = openList(1,:);
    openList(1,:) = [];
    closedList = [closedList; [currentNode(1), currentNode(2), nodes(bestNode(1),bestNode(2)).h]];

    disp("closed list:");
    disp(closedList);
    disp("open list:");
    disp(openList);
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
                if newIndex(1) > 0 && newIndex(2) > 0 && newIndex(1) < xnum && newIndex(2) < ynum
                    %check that the node is in the map
                    if nodes(newIndex(1),newIndex(2)).inmap
                        %calculate the g cost
                        if sum(abs(offset)) == 2
                            g = currentNode(3) + 1.4;
                        else
                            g = currentNode(3) + 1;
                        end
                        %f = g + h
                        f = g + nodes(newIndex(1),newIndex(2)).h;
                        %add node the list of new nodes
                        newEntry = [(index + [i,j]) f];
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
            if newEntry(3) < row(3)
                newOpenList(i,3) = newEntry(3);
            end
        end
    end
    %if not in the open list then add to the end
    if bool == 0
        newOpenList = [newOpenList; newEntry];
    end
end

