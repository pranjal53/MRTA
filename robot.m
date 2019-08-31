
classdef robot
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Nrow
        location=1;
        path=[];
        loading=0;
        runing_mission=[];
        next_mission=[];
        %Shelf address
        Shelve1=[];
        %Shelf status: 1, idle; 2, take the shelf; 3, send the shelf to the sorting station; 4, return the shelf to the original position
        loading_state=1;
        x_lab1;y_lab1;
         x_filled1;y_filled1;
         wait_node_index=0;
         wait_time=0;
         waiting_state=0;
         wait_path;
         sorting_table_path=[];
        runningtime=1;
    end
     properties(Dependent)
%      path;
     length_path;
     shelve_index;
      time_series;
      xlab;
      ylab;
     end
    
    methods
        %Constructor
        function newrobot=robot(path,Nrow,wait_node_index,wait_time)
            if nargin ==4
            newrobot.path=path;
            newrobot.Nrow=Nrow;
           newrobot.wait_node_index=wait_node_index;newrobot.wait_time=wait_time;
           newrobot.wait_path=path;
            end
        end
%         function identify(thisrobot)
% %              disp(['I am ' num2str(thisrobot.location)]);
%           [temp thisrobot.shelve_index]=ismember(thisrobot.location,thisrobot.path);
%           [thisrobot.xlab,thisrobot.ylab]=ind2sub([thisrobot.Nrow,thisrobot.Nrow],thisrobot.location);
%         end
        function length_path=get.length_path(newrobot)
            length_path=length(newrobot.path);
%             disp('get.length_path called')
        end
        function time_series=get.time_series(newrobot)
%             length_path=length(newrobot.path);
            if (newrobot.wait_node_index>0)&(newrobot.wait_time>0)
                time_series=[1:newrobot.wait_node_index,...
                    newrobot.wait_node_index*ones(1,newrobot.wait_time-1),newrobot.wait_node_index:length(newrobot.path)];
            else
                time_series=1:length(newrobot.path);
            end
         end
         function shelve_index=get.shelve_index(newrobot)
            length_path=length(newrobot.path);
            [a1 shelve_index]=ismember(newrobot.next_mission,newrobot.path);
         end
         function xlab=get.xlab(newrobot)
             [xlab,ylab]=ind2sub([newrobot.Nrow,newrobot.Nrow],newrobot.location);
         end
         function ylab=get.ylab(newrobot)
             [xlab,ylab]=ind2sub([newrobot.Nrow,newrobot.Nrow],newrobot.location);
         end
    end  
end
