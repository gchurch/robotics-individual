classdef Node < handle
    properties
        pos
        index
        inmap
        h
    end
    
    methods
        function node = Node(pos,index,inmap)
            if nargin > 0
                node.pos = pos;
                node.index = index;
                node.inmap = inmap;
                node.h = 0;
            end
        end
        
        function calculateHeuristic(node, targetNode)
            diagonal = min(abs(targetNode - node.index));
            straight = max(abs(targetNode - node.index)) - diagonal;
            value = diagonal + straight;
            node.h = value;
        end
    end
end
        