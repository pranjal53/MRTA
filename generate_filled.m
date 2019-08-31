function[x_filled,y_filled]= generate_filled(robot_path)
    global Nrow;
    x_filled=zeros(length(robot_path),4);
    y_filled=zeros(length(robot_path),4);
    for i=1:length(robot_path)
        [x_filled_process,y_filled_process]=ind2sub([Nrow,Nrow],robot_path(i));
        x_filled(i,1)=x_filled_process-1;x_filled(i,4)=x_filled_process-1;
        x_filled(i,2)=x_filled_process;x_filled(i,3)=x_filled_process;
        y_filled(i,1)=y_filled_process-1;y_filled(i,2)=y_filled_process-1;
        y_filled(i,3)=y_filled_process;y_filled(i,4)=y_filled_process;
    end