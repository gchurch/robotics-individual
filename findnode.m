function node = findnode(nodes, pos)
%Find the node that corresponds to a given position
node = 1;
dims = size(nodes);
xmax = dims(1);
ymax = dims(2);
for i=1:xmax
    for j=1:ymax
        coords = [nodes(i,j,1), nodes(i,j,2)];
        if coords == pos
            node = [i,j];
        end
     end
end