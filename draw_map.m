%Draw a storage project simulation animation, 0, 1, 2, 3 in the map represent various nodes, grid1 represents the grid number
function [all_distance]=draw_map(map,grid1,robot_ID,Nrow,Ncol,mission_list,sorting_table)
    all_distance=0;
    % figure(2)
    if Ncol==20||Nrow==20
        robot_size=24;
    elseif Ncol==11
        robot_size=24;
    else
        robot_size=10;
    end
    axis([0,Nrow,0,Ncol])
%     plot(1,2,'ro')
%     hold on
    for i= 1:Ncol
        for j=1:Nrow
            switch(map(i,j))
                case 1
                    x1=j-1;y1=Ncol-i;
                    x2=j;y2=Ncol-i;
                    x3=j;y3=Ncol-i+1;
                    x4=j-1;y4=Ncol-i+1;
                    fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.4,0.4,0.4]);
                    hold on
                case 2 
                    x1=j-1;y1=Ncol-i;
                    x2=j;y2=Ncol-i;
                    x3=j;y3=Ncol-i+1;
                    x4=j-1;y4=Ncol-i+1;
                    fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.6,0.6,0.6]);
                    hold on
                case 3
                    x1=j-1;y1=Ncol-i;
                    x2=j;y2=Ncol-i;
                    x3=j;y3=Ncol-i+1;
                    x4=j-1;y4=Ncol-i+1;
                    fill([x1,x2,x3,x4],[y1,y2,y3,y4],'g');
                    hold on
                case 99
                    x1=j-1;y1=Ncol-i;
                    x2=j;y2=Ncol-i;
                    x3=j;y3=Ncol-i+1;
                    x4=j-1;y4=Ncol-i+1;
                    fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0,0,0]);
                    hold on
            end
        end                   
    end
    axis([0,Nrow,0,Nrow])
    grid on
    set(gca,'xtick',[0:1:Nrow],'ytick',[1:1:Ncol])
    for i=1:Nrow
        for j=1:Ncol
            text(i-0.5,j-0.5,num2str(grid1(i,j)))
        end
    end
    for robot_i=1:length(robot_ID)
        p(robot_i)=plot(robot_ID(robot_i).xlab-0.5,robot_ID(robot_i).ylab-0.5,'bs','MarkerSize',robot_size);
        [x_filled2,y_filled2] = generate_filled(robot_ID(robot_i).location);
        p_fill(robot_i)=fill(x_filled2,y_filled2,'y','FaceAlpha',0.4);
    end
    iter=2;mission_index=4;
    max_wait_length=3;
    while max_wait_length>=iter
        wait_length1=[];
        for robot_i=1:length(robot_ID)
            if length(robot_ID(robot_i).wait_path)>1
                [robot_ID(robot_i).x_lab1,robot_ID(robot_i).y_lab1]= generate_lab(robot_ID(robot_i).wait_path(1:2));
                [robot_ID(robot_i).x_filled1,robot_ID(robot_i).y_filled1] = generate_filled(robot_ID(robot_i).wait_path);
                %%'FaceAlpha':Set transparency
                set(p_fill(robot_i),'XData',(robot_ID(robot_i).x_filled1)',...
                    'Ydata',(robot_ID(robot_i).y_filled1)','FaceAlpha',0.2);
            else
                [robot_ID(robot_i).x_filled1,robot_ID(robot_i).y_filled1] = generate_filled(robot_ID(robot_i).location);
                %%'FaceAlpha':Set transparency
                set(p_fill(robot_i),'XData',(robot_ID(robot_i).x_filled1)',...
                    'Ydata',(robot_ID(robot_i).y_filled1)','FaceAlpha',0.05);
                if mission_index<=length(mission_list)
                    robot_ID(robot_i).next_mission=mission_list(mission_index);
                    [sorting_table_index]= assign_sorting_table(robot_ID(robot_i).next_mission,sorting_table);
                 % It is thought that the car may stop at the mission point, so the task points of other cars are set as obstacles.
                    map2=map;
                    for map_i = 1:length(robot_ID)
                        if map_i~=robot_i
                        [ia,ib]=ind2sub([Nrow,Ncol],robot_ID(map_i).next_mission);
                            map2(Ncol-ib+1,ia)=2;
                        end
                    end
                    
        robot_ID(robot_i).path=[Astar_method(robot_ID(robot_i).location,robot_ID(robot_i).next_mission,0,map2),...
                Astar_method(robot_ID(robot_i).next_mission,sorting_table(sorting_table_index,1),1,map2),...
                sorting_table(sorting_table_index,2:4),Astar_method(sorting_table(sorting_table_index,5),...
                robot_ID(robot_i).next_mission,1,map2)];
                robot_ID(robot_i).loading_state=1;
                    %Update the path to a conflict-free waiting path
                    robot_ID(robot_i).wait_path=robot_ID(robot_i).path;
                    [robot_ID(robot_i).wait_path,new_wait_path]=new_enqueue(robot_ID,robot_i,map,sorting_table);
                    if length(robot_ID(robot_i).wait_path)>1
                        [robot_ID(robot_i).x_lab1,robot_ID(robot_i).y_lab1]= generate_lab(robot_ID(robot_i).wait_path(1:2));
                    end
                    mission_index=mission_index+1;
                end
            end
        end
        %Draw an image of each frame
        for j=2:6
            for robot_i=1:length(robot_ID)
                if length(robot_ID(robot_i).wait_path)>1
                    if robot_ID(robot_i).loading_state<=2
                        set(p(robot_i),'XData',robot_ID(robot_i).x_lab1(j),'Ydata',robot_ID(robot_i).y_lab1(j),...
                            'MarkerSize',robot_size,'MarkerFaceColor','blue')
                    else
                        set(p(robot_i),'XData',robot_ID(robot_i).x_lab1(j),'Ydata',robot_ID(robot_i).y_lab1(j),...
                                'MarkerSize',robot_size,'MarkerFaceColor','red')
                    end
                else
                    set(p(robot_i),'XData',robot_ID(robot_i).xlab-0.5,'Ydata',robot_ID(robot_i).ylab-0.5,...
                            'MarkerSize',robot_size,'MarkerFaceColor','blue')
                end
            end
            drawnow;
            axis([0 Nrow 0 Ncol]);
            pause(0.05);
        end
        %Remove the path that each robot has traveled
        for robot_i=1:length(robot_ID)
            wait_length1=[wait_length1,length(robot_ID(robot_i).wait_path)];
            if length(robot_ID(robot_i).wait_path)>1
                if robot_ID(robot_i).wait_path(1)==robot_ID(robot_i).next_mission
                        robot_ID(robot_i).loading_state=3;
                end
                robot_ID(robot_i).wait_path=robot_ID(robot_i).wait_path(2:length(robot_ID(robot_i).wait_path));
                all_distance=all_distance+1;
                robot_ID(robot_i).location=robot_ID(robot_i).wait_path(1);
            end           
        end
        max_wait_length=max(wait_length1);
        min(wait_length1);
    end
end
            

