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

coords = zeros(xnum, ynum, 2);
inmap = zeros(xnum,ynum);
%getting map coordinates and determining if coordinate in inside the map
for i=1:xnum
    for j=1:ynum
        % we subtract 0.5 so that we reside in the middle of the squares
        x = xstart + (i-0.5) * xstep;
        y = ystart + (j-0.5) * ystep;
        pos = [x,y];
        coords(i,j,1) = pos(1);
        coords(i,j,2) = pos(2);
        %coords = [coords; pos];
        if inpolygon(pos(1),pos(2),inpolygonMapformatX,inpolygonMapformatY) == 1
            inmap(i,j) = 1;
        else
            inmap(i,j) = 0;
        end
    end
end

%{
%printing the coords array
for i=1:xnum
    for j=1:ynum
        coord = [coords(i,j,1), coords(i,j,2)];
        disp(coord);
    end
end
%map is rotated clockwise
disp(inmap);
%}

%% create bot and draw map
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
%set the bots position to the first coordinate
botSim.setBotPos([coords(1,1,1),coords(1,1,2)]);
%set the target position to the last coordinate
target = [coords(xnum,ynum,1), coords(xnum,ynum,2)];
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
startNode = findnode(coords, botSim.getBotPos());
fprintf("start node: (%d,%d)\n", startNode(1), startNode(2));
targetNode = findnode(coords, target);
fprintf("target node: (%d,%d)\n", targetNode(1), targetNode(2));
