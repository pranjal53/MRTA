% Reference program Archive in asardemo.m
%Store feasible nodes towards 1, 2, 3, 4
function [path]=Astar_method(start_node,end_node,loading,map,stop_car)
%     Nrow=20;start_node=115;shelf_node=225;end_node=143;loading=1;
    setOpen=[start_node];setOpenCosts=[0];setOpenHeuristics=[Inf];
    setClosed=[];setClosedCosts=[];
  %load /Users/jindi/Desktop/map.txt
    Nrow=length(map(1,:));
    Ncol=length(map(:,1));
%     if 
    %The coordinates of the top right corner of the grid
    [xlabID_end,ylabID_end]=ind2sub([Nrow,Ncol],end_node);
    %Change the starting and ending points to no obstacles
    map(Ncol-ylabID_end+1,xlabID_end)=0;
    if loading==0
        for i=1:(Nrow*Ncol)
            connect_node_name=[0,0,0,0];
            %The coordinates of the top right corner of the grid
            [xlabID,ylabID]=ind2sub([Nrow,Ncol],i);
            if (xlabID<Nrow)
                if (~ismember(map(Ncol-ylabID+1,xlabID+1),[2,99]))
                    connect_node_name(1)=i+1;
                end
            end
            if (ylabID<Ncol)
                %The coordinate relationship of the map is opposite to the number of columns in the grid for the y-axis and the number of columns.
                if(~ismember(map(Ncol-ylabID,xlabID),[2,99]))
                    connect_node_name(2)=i+Nrow;
                end    
            end
            if (xlabID>1)
                if (~ismember(map(Ncol-ylabID+1,xlabID-1),[2,99]))
                    connect_node_name(3)=i-1;
                end
            end
            if (ylabID>1)
                if(~ismember(map(Ncol-ylabID+2,xlabID),[2,99]))
                    connect_node_name(4)=i-Nrow;
                end
            end
            connect_node_ID(i)=connect_node([connect_node_name,connect_node_name]);
        end
    else
        for i=1:(Nrow*Ncol)
            connect_node_name=[0,0,0,0];
            %The coordinates of the top right corner of the grid
            [xlabID,ylabID]=ind2sub([Nrow,Ncol],i);
            if (xlabID<Nrow)
                if(~ismember(map(Ncol-ylabID+1,xlabID+1),[1,2,3,99]))
                    connect_node_name(1)=i+1;
                end 
            end
            if (ylabID<Ncol)
    %             [xlabID1,ylabID1]=ind2sub([Nrow,Nrow],i+Nrow)
                if(~ismember(map(Ncol-ylabID,xlabID),[1,2,3,99]))
                    connect_node_name(2)=i+Nrow;
                end
            end
            if (xlabID>1)
                if(~ismember(map(Ncol-ylabID+1,xlabID-1),[1,2,3,99]))
                    connect_node_name(3)=i-1;
                end
            end
            if (ylabID>1)
                if(~ismember(map(Ncol-ylabID+2,xlabID),[1,2,3,99]))
                    connect_node_name(4)=i-Nrow;
                end
            end
            %For the right side priority, set the repeating path
            connect_node_ID(i)=connect_node([connect_node_name connect_node_name]);
        end
    end
    [xlabID_start,ylabID_start]=ind2sub([Nrow,Ncol],start_node);
    if xlabID_start-xlabID_end<=0
        %Path direction
        if ylabID_start-ylabID_end<=0
            path_direction=1;
        else
            path_direction=2;
        end
    else
        if ylabID_start-ylabID_end>0
            path_direction=3;
        else
            path_direction=4;
        end
    end

    for k1=1:Nrow*Ncol
        heuristics(k1)=Hamilton_distance(k1,end_node,Nrow,Ncol);
    end 
    while (~isempty(setOpen))&&(~max(ismember(setOpen,end_node)))
        [temp, ii] = min(setOpenCosts + setOpenHeuristics);
        setClosed = [setClosed, setOpen(ii)];
        setClosedCosts = [setClosedCosts, setOpenCosts(ii)];  
        %Parent node
        posinds=setOpen(ii);costs=setOpenCosts(ii);
        % update OPEN and their associated costs
        if (ii > 1 && ii < length(setOpen))
            setOpen = [setOpen(1:ii-1), setOpen(ii+1:end)];
            setOpenCosts = [setOpenCosts(1:ii-1), setOpenCosts(ii+1:end)];
            setOpenHeuristics = [setOpenHeuristics(1:ii-1), setOpenHeuristics(ii+1:end)];
        elseif (ii == 1)
            setOpen = setOpen(2:end);
            setOpenCosts = setOpenCosts(2:end);
            setOpenHeuristics = setOpenHeuristics(2:end);
        else
            setOpen = setOpen(1:end-1);
            setOpenCosts = setOpenCosts(1:end-1);
            setOpenHeuristics = setOpenHeuristics(1:end-1);
        end
        for jj=1:4
            next_node=connect_node_ID(posinds).connect_node_name(jj+path_direction-1);
            % if it's a wall, so ignore
            if (next_node~=0)
              % if node is not in OPEN or CLOSED then insert into costchart and 
              % movement pointers, and put node in OPEN
              if ~max([setClosed, setOpen] == next_node)
                connect_node_ID(next_node).father_node=posinds;
                setOpen = [setOpen, next_node];
                setOpenCosts = [setOpenCosts, costs+1];
                setOpenHeuristics = [setOpenHeuristics, heuristics(next_node)];
              % else node has already been seen, so check to see if we have
              % found a better route to it.
              elseif max(setOpen == next_node)
                I = find(setOpen == next_node);
                % update if we have a better route
                if setOpenCosts(I) > costs+1
                  connect_node_ID(next_node).father_node=posinds;
                  setOpenCosts(I) = costs+1;
                  setOpenHeuristics(I) = heuristics(next_node);
                end
              % else node has already been CLOSED, so check to see if we have
              % found a better route to it.
              else
                % find relevant node in CLOSED
                I = find(setClosed == next_node);
                % update if we have a better route
                if setClosedCosts(I) > costs+1
                  setClosedCosts(I) = costs+1;
                  connect_node_ID(next_node).father_node=posinds;
                end
              end
            end
        end
        if ismember(end_node,setClosed)|(length(setOpen)==0)

            break;
        end
    end

    path_node=end_node;
    path=[end_node];
    while length(path_node)~=0
        previous_node=connect_node_ID(path_node).father_node;
        path=[path,previous_node];
        path_node=previous_node;
    end
    path=fliplr(path);
end




    
    
    
    
    