classdef allbusypath < handle
%   The path occupied by all vehicles at each moment
    properties
        %Path occupancy point at this moment and the next moment
        path_value=[];
    end
    properties(Dependent)
        length_path
    end
    methods
        function newallbusypath=allbusypath(path_value)
            if nargin == 1
                newallbusypath.path_value=path_value;
%                 newallbusypath.wait_path=path_value;
            end
        end
        function length_path=get.length_path(newallbusypath)
            length_path=length(newallbusypath.path_value);
            disp('get.length_path called')
        end
    end
end