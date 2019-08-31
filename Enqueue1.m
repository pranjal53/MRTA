%Queuing mechanism
% path1=[9,10,6,5,4,3,11,12,13];
% path2=[1,2,3,4,5,6,7,8];
% fixed_index=1;
% path1_wait_node=0;
% path1_wait_time=0;
function [wait_path1,wait_path2,result,need_Astar]=Enqueue1(path1,path2,fixed_index,map,sorting_table)
Nrow=length(map(1,:));Ncol=length(map(:,1));
    if path1(1)~=path2(1)
        %result=10 The result is correct and initialized to 10.
        result=10;
        %fixed_index==1,Indicates that the former path is fixed.
        if fixed_index==1
            i=1;
            wait_path1=path1;
            wait_path2=path2;
            if length(wait_path1)==1
                need_Astar=1;
                if wait_path2(2)==wait_path1
                  %if robot.mission==[],Then need to re-plan the path 
                  wait_path2=[[wait_path2(1),wait_path2(1)],wait_path2(2:length(wait_path2))];
                  disp('Since there is only one vehicle path point in front, the vehicle needs to wait or re-plan the path.');
                end
            else
                while i<min(length(wait_path1),length(wait_path2))
                    %Make sure that the point is not the next point of another car.
                    if ((wait_path2(i)~=wait_path1(i+1))&(wait_path2(i+1)==wait_path1(i+1)))|((wait_path2(i+1)==wait_path1(i))&(wait_path1(i+1)~=wait_path2(i)))
                        wait_path2=[wait_path2(1:i),[wait_path2(i)],wait_path2((i+1):length(wait_path2))];
%                         wait_path1
%                         wait_path2
                    %%Modified, or changed to
                    else
                        %January 14 good modification last is not equal
                        if ((wait_path2(i+1)==wait_path1(i))&(wait_path1(i+1)==wait_path2(i)))|((wait_path2(i)==wait_path1(i+1))&(wait_path2(i+1)~=wait_path1(i+1)))
                            if i-1>0
                                %Find the previous node of the path and wait
                                j1=i;
                                while j1>1
                                    %Avoid a little conflict
                                    if (wait_path2(j1-1)==wait_path2(j1))|(wait_path2(j1-1)==wait_path1(i))
                                       j1=j1-1;
                                    else
                                       break;
                                    end
                                end
                                if j1==1
%                                     map1=map;
                                    
                                    wait_path2=path2(1);
                                    
                                    disp('enqueue1 Line 39 has returned to the starting point')
                                    result=11;
                                    break;
                                end
                                if j1<i
                                    for j2=j1:(i-1)
                                       wait_path2(j2) = wait_path2(j1-1);
                                    end
                                %elseif (wait_path2(i)==wait_path1(i+1))&(wait_path2(i+1)==wait_path1(i+1))

                                end
                                wait_path2=[wait_path2(1:(i-1)),[wait_path2((i-1))],wait_path2(i:length(wait_path2))];
                            else
                                disp('path1,No avoidance point, re-planned path')
                                map1=map;
                                [ia,ib]=ind2sub([Nrow,Ncol],wait_path1(1));
                                map1(Ncol-ib+1,ia)=2;
                                end_node=wait_path2(length(wait_path2));start_node=wait_path2(1);
                                [sorting_table_index]= assign_sorting_table(end_node,sorting_table);
                                wait_path2=[Astar_method(start_node,end_node,0,map1),...
                                    Astar_method(end_node,sorting_table(sorting_table_index,1),1,map1),sorting_table(sorting_table_index,2:4),...
                                    Astar_method(sorting_table(sorting_table_index,5),end_node,1,map1)];
                                result=12;
                                wait_path1=path1;
                                break;
                            end
                        else
                            i=i+1;
                        end
                    end
                end
                need_Astar=0;
            end       
        else
            disp('Fixed does not enter correctly')
        end
    else
        wait_path1=path1(1);
        wait_path2=path2(1);
        result=11;
        need_Astar=1;
        disp('Input error, two paths with the same starting point')
    end
end



