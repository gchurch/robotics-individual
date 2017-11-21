classdef DiscreteMap < handle
    properties
        map
        xnum
        ynum
        xstart
        ystart
        xend
        yend
        xstep
        ystep
        nodes
    end
    
    methods
        function obj = DiscreteMap(botSim, xnum, ynum)
            if nargin > 0
                obj.map = botSim.getMap();
                obj.xnum = xnum;
                obj.ynum = ynum;
                discretize(obj, botSim);
                padding(obj);
            end
        end
        
        %turn our map into a grid
        function discretize(obj, botSim)
            
            %% discretization
            t = cputime;
 
            %the min and max axis values
            obj.xstart = obj.map(1,1);
            obj.xend = obj.map(1,1);
            obj.ystart = obj.map(1,2);
            obj.yend = obj.map(1,2);

            dim = size(obj.map);
            for i=1:dim(1)
                if obj.map(i,1) < obj.xstart
                    obj.xstart = obj.map(i,1);
                end
                if obj.map(i,1) > obj.xend
                    obj.xend = obj.map(i,1);
                end
                if obj.map(i,2) < obj.ystart
                    obj.ystart = obj.map(i,2);
                end
                if obj.map(i,2) > obj.yend
                    obj.yend = obj.map(i,2);
                end
            end

            %the real distance between each node
            obj.xstep = (obj.xend - obj.xstart) / obj.xnum;
            obj.ystep = (obj.yend - obj.ystart) / obj.ynum;

            %array of objects holding information for each node
            nodes(obj.xnum,obj.ynum) = Node;
            obj.nodes = nodes;
            %getting map coordinates and determining if coordinate is inside the map
            for i=1:obj.xnum
                for j=1:obj.ynum
                    % we subtract 0.5 so that we reside in the middle of the squares
                    x = obj.xstart + (i-0.5) * obj.xstep;
                    y = obj.ystart + (j-0.5) * obj.ystep;
                    % set node pos property
                    pos = [x,y];
                    obj.nodes(i,j).pos = pos;
                    % set node index property
                    index = [i,j];
                    obj.nodes(i,j).index = index;
                    %set node inmap property
                    if botSim.pointInsideMap(pos)
                        obj.nodes(i,j).inmap = 1;
                    else
                        obj.nodes(i,j).inmap = 0;
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
        end
        
        %find the closest node in the map to a given position
        function indexes = findClosestNode(obj, pos)
            xrelative = pos(1) - obj.xstart + (obj.xstep / 2);
            yrelative = pos(2) - obj.ystart + (obj.ystep / 2);
            closesti = round(xrelative / obj.xstep);
            closestj = round(yrelative / obj.ystep);
            if closesti <= 0
                closesti = 1;
            end
            if closesti > obj.xnum
                closesti = obj.xnum;
            end
            if closestj <= 0
                closestj = 1;
            end
            if closestj > obj.ynum
                closestj = obj.ynum;
            end
            indexes = [closesti, closestj];
        end
        
        %add configuration space to the map
        function padding(obj)
             newNodes(obj.xnum,obj.ynum) = Node;
             for i=1:obj.xnum
                 for j=1:obj.ynum
                     newNodes(i,j).pos = obj.nodes(i,j).pos;
                     newNodes(i,j).index = obj.nodes(i,j).index;
                     newNodes(i,j).inmap = obj.nodes(i,j).inmap;
                     newNodes(i,j).h = obj.nodes(i,j).h;
                 end
             end
             for i=1:obj.xnum
                 for j=1:obj.ynum
                     out = 0;
                     for a=-3:3
                         for b=-3:3
                             newIndex = [i+a,j+b];
                             if newIndex(1) > 0 && newIndex(1) <= obj.xnum && newIndex(2) > 0 && newIndex(2) <= obj.ynum
                                 if obj.nodes(newIndex(1), newIndex(2)).inmap == 0
                                     out = 1;
                                 end
                             end
                         end
                     end
                     if out == 1
                         newNodes(i,j).inmap = 0;
                     end
                 end
             end
             obj.nodes = newNodes;
        end
    end
end
        