function[x_lab,y_lab]= generate_lab(robot_path)
        global Nrow;
        %Mod is the remainder symbol
        for i=1:length(robot_path)
            [path_location_process1,path_location_process2]=ind2sub([Nrow,Nrow],robot_path(i));
            path_location1(i)=path_location_process1-0.5;path_location2(i)=path_location_process2-0.5;
        end
        x_lab=linspace(path_location1(1),path_location1(2), 6);
        y_lab=linspace(path_location2(1),path_location2(2), 6);
        for i=2:(length(path_location1)-1)
            %%Insert several points evenly into two points
            x_process=linspace(path_location1(i),path_location1(i+1), 6);
            x_lab=[x_lab x_process(2:6)];
            y_process=linspace(path_location2(i),path_location2(i+1), 6);
            y_lab=[y_lab y_process(2:6)];
        end
     end