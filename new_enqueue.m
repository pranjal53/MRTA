%new_wait_path=1,The result is valid, new_wait_path=0, the conflict is not resolved.
function [result,new_wait_path]=new_enqueue(robot_ID,robot_index,map,sorting_table)
    % All two-two path conflict check
    if robot_index==1
        all_conflict=[2:length(robot_ID)];
    elseif robot_index==length(robot_ID)
        all_conflict=[1:(length(robot_ID)-1)];
    else
        all_conflict=[1:(robot_index-1),(robot_index+1):length(robot_ID)];
    end
    iters=3;
    while iters>0
        length_path1=length(robot_ID(robot_index).wait_path);
        for k1 = 1:length(all_conflict)
            [wait_path1,wait_path2,result1]=Enqueue1(robot_ID(all_conflict(k1)).wait_path,...
                robot_ID(robot_index).wait_path,1,map,sorting_table);
            if result1==11
                result=robot_ID(robot_index).location;
                disp('The two cars are numbered:')
                all_conflict(k1)
                robot_index
                disp('New_enqueue Two cars can't be avoided, need to re-plan the path, currently parked at the origin')
                new_wait_path=[robot_ID(robot_index).location,robot_ID(robot_index).location];
                break;
            else
                robot_ID(robot_index).wait_path=wait_path2;
                new_wait_path=1;
            end
        end
        if length(robot_ID(robot_index).wait_path)==length_path1
            break;
        else
            iters=iters-1;
        end
        if iters==0;
            result=robot_ID(robot_index).location;
            new_wait_path=0;
            disp('Exceeding the maximum number of iterations, you need to re-plan the path')
        end
    end
    result=robot_ID(robot_index).wait_path;
end