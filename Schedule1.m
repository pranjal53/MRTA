%Task assignment function, because of the empty car, directly using the Hamiltonian distance
function [scedule_table,undistributed_mission]=Schedule1(robot_ID,mission,Nrow,Ncol)
robot_ID1=[];
for i=1:length(robot_ID)
    if (robot_ID(i).loading_state==1)|(robot_ID(i).loading_state==4)
        robot_ID1=[robot_ID1,robot_ID(i)];
    end
end
if ~isempty(robot_ID1)
    if length(mission)>length(robot_ID1)
        mission1=mission(1:length(robot_ID1));
        undistributed_mission=mission((length(robot_ID1)+1):length(mission));
    else
        mission1=mission;
        undistributed_mission=[];
    end
    costs=[];
    for k1 = 1:length(robot_ID1)
        if robot_ID1(k1).loading_state==1
            for k2 = 1:length(mission1)
                costs(k1,k2)=Hamilton_distance(robot_ID1(k1).location,mission1(k2),Nrow,Ncol);
            end
        else
            for k2 = 1:length(mission1)
                %To add the remaining distance of the vehicle
                costs(k1,k2)=Hamilton_distance(robot_ID1(k1).location,mission1(k2),Nrow,Ncol)+...
                    length(robot_ID1(k1).path);
            end
        end
    end
    if length(mission1)<=4
        AA=nchoosek(1:length(robot_ID1),length(mission1));all_iter=[];
        for i=1:length(AA(:,1))
            %All possible assignments
            all_iter=[all_iter;perms(AA(i,:))];
        end
        [x_row,y_col]= size(all_iter);
        accumulative_cost=[];
        for jj=1:x_row
         accumulative_cost(jj)=0;
         for jj1=1:y_col
            accumulative_cost(jj)=accumulative_cost(jj)+costs(all_iter(jj,jj1),jj1);
         end
        end
        [temp,cost_index1]=min(accumulative_cost);
        %1:Y_col is the index of the task
        scedule_table=[1:y_col;all_iter(cost_index1,:)];
    else
        row_index1=1:length(costs(:,1));
        scedule_table=[1:length(mission1);1:length(mission1)];
        for i=1:length(mission1)
            [temp,min_index11]=min(costs(row_index1,i));
            scedule_table(2,i)=row_index1(min_index11);
            row_index1(min_index11)=[];
        end
    end
else
    disp('No vehicle available')
    scedule_table=[];
end
end