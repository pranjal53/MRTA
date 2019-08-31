classdef allpath < handle
    properties
        path_value=[];
        wait_path=[];
        wait_node_index=0;
        wait_time=0;
    end
    properties(Dependent)
        length_path
    end
    methods
        function newallpath=allpath(path_value)
            if nargin == 1
                newallpath.path_value=path_value;
                newallpath.wait_path=path_value;
            end
        end
        function length_path=get.length_path(newallpath)
            length_path=length(newallpath.path_value);
            disp('get.length_path called')
        end
    end
end