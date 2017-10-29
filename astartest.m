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