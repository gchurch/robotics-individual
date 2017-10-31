classdef DiscreteMap < handle
    properties
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
        function obj = DiscreteMap(xnum, ynum, xstart, ystart, xend, yend, xstep, ystep, nodes)
            if nargin > 0
                obj.xnum = xnum;
                obj.ynum = ynum;
                obj.xstart = xstart;
                obj.ystart = ystart;
                obj.xend = xend;
                obj.yend = yend;
                obj.xstep = xstep;
                obj.ystep = ystep;
                obj.nodes = nodes;
            end
        end
        
        
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
    end
end
        