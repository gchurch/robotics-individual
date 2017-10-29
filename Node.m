classdef Node
    properties
        pos
        index
        inmap
    end
    
    methods
        function node = Node(pos,index,inmap)
            if nargin > 0
                node.pos = pos;
                node.index = index;
                node.inmap = inmap;
            end
        end
    end
end
        