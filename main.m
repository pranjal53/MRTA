%The main function, the number of robot_number currently supports 2~6;
clear all;
close all;
clear variables;
clc;
robot_number=3;
[map,grid1,Nrow,Ncol,sorting_table,Obstacle]=initial_input();
% Obstacle_index=max(round(rand(1,30)*length(Obstacle)),1);
%    Obstacle_index=randperm(length(Obstacle),10);
   Obstacle_index=[48,24,70,43,34,15,59,12,58,35,46,61, 4,79,30,63,60,31,67,37, 6,20,50,73,72,56,47,14,78,62];
mission_list=[];start_node=[];
for i=1:length(Obstacle_index)
  mission_list=[mission_list,Obstacle(Obstacle_index(i))];
end
mission=mission_list(1:robot_number);
%Select some numbers from N numbers that are not repeated
%   start_node=Obstacle(randperm(length(Obstacle),robot_number));
   start_node=[166,352,5,12,120,220,240,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40];
for k1=1:robot_number    
    robot_ID(k1)=robot();
    robot_ID(k1).location=start_node(k1);
end

[scedule_table,undistributed_mission]=Schedule1(robot_ID,mission,Nrow,Ncol);
%The index of the first behavior task of the scedule_table result, the format is mostly 1:length(mission), and the second behavior is the car assigned to the task.
for i=1:length(scedule_table(1,:))
    end_node(scedule_table(2,i))=mission(scedule_table(1,i));
end
for k2=1:robot_number
    map3=map;
    for map_i = 1:robot_number
        if map_i~=k2
            [ia,ib]=ind2sub([Nrow,Ncol],start_node(map_i));
            map3(Ncol-ib+1,ia)=2;
            [ia1,ib1]=ind2sub([Nrow,Ncol],end_node(map_i));
            map3(Ncol-ib1+1,ia1)=2;
        end
    end
    [sorting_table_index]= assign_sorting_table(end_node(k2),sorting_table);
    allpath_ID(k2)=allpath([Astar_method(start_node(k2),end_node(k2),0,map3),Astar_method(end_node(k2),sorting_table(sorting_table_index,1),1,map3),...
    sorting_table(sorting_table_index,2:4),Astar_method(sorting_table(sorting_table_index,5),end_node(k2),1,map3)]);
end

if length(robot_ID)>1
    % All two-two path conflict check
    all_conflict=nchoosek(1:length(robot_ID), 2);
    k1=1;iters=length(all_conflict(:,1))*2;
    while k1<=length(all_conflict(:,1))&iters>0;
        if k1==1
            [wait_path1,wait_path2,result1]=Enqueue1(allpath_ID(all_conflict(k1,1)).wait_path,...
                allpath_ID(all_conflict(k1,2)).wait_path,1,map);
            [wait_path4,wait_path3,result2]=Enqueue1(allpath_ID(all_conflict(k1,2)).wait_path,...
                allpath_ID(all_conflict(k1,1)).wait_path,1,map);
            if (result1==11)&(result2==11)
                disp('The first two cars cannot be avoided and need to re-plan the path')
            else
                if (result2==11)|((result1==10)&(length(wait_path1)+length(wait_path2)<=length(wait_path3)+length(wait_path4)))
                    allpath_ID(all_conflict(k1,1)).wait_path=wait_path1;
                    allpath_ID(all_conflict(k1,2)).wait_path=wait_path2;
                else
                    allpath_ID(all_conflict(k1,1)).wait_path=wait_path3;
                    allpath_ID(all_conflict(k1,2)).wait_path=wait_path4;
                end
            end
            k1=k1+1;
        else
            [wait_path5,wait_path6,result3]=Enqueue1(allpath_ID(all_conflict(k1,1)).wait_path,...
                allpath_ID(all_conflict(k1,2)).wait_path,1,map,sorting_table);
            if result3==10
                allpath_ID(all_conflict(k1,1)).wait_path=wait_path5;
                allpath_ID(all_conflict(k1,2)).wait_path=wait_path6;
                k1=k1+1;
            else 
                [wait_path6,wait_path5,result3]=Enqueue1(allpath_ID(all_conflict(k1,2)).wait_path,...
                allpath_ID(all_conflict(k1,2)).wait_path,1,map);
                if result3==10
                    allpath_ID(all_conflict(k1,1)).wait_path=wait_path5;
                    allpath_ID(all_conflict(k1,2)).wait_path=wait_path6;
                    k1=1;
                else
                    if result3==12
                        allpath_ID(all_conflict(k1,1)).wait_path=wait_path5;
                        allpath_ID(all_conflict(k1,2)).wait_path=wait_path6;
                        k1=1;
%                     disp('A two-volume car can't be avoided, and the path needs to be re-planned.')
                    end
                end
            end
        end
        iters=iters-1;
        if iters==0;
            disp('Exceeding the maximum number of iterations, you need to re-plan the path')
        end
    end
    %Initialize the car
    for k3=1:robot_number
        robot_ID(k3)=robot(allpath_ID(k3).wait_path,Nrow,0,0);
        robot_ID(k3).location=start_node(k3);robot_ID(k3).loading_state=1;
    end
    for i=1:length(scedule_table(1,:))
        robot_ID(scedule_table(2,i)).next_mission=mission(scedule_table(1,i));
    end
    
else
    robot_ID(1)=robot(allpath_ID(1).wait_path,Nrow,0,0);
    robot_ID(1).location=start_node(1);robot_ID(1).loading_state=1;
    robot_ID(scedule_table(2,1)).next_mission=mission(scedule_table(1,1));
end


% Simulation demonstration
% diary('log_07.txt')
result=draw_map(map,grid1,robot_ID,Nrow,Ncol,mission_list,sorting_table)







