mission_list;
all_distance=0;
for i=1:30 
    [sorting_table_index] = assign_sorting_table(mission_list(i));
    AA1=sorting_table(sorting_table_index,1);
    AA2=sorting_table(sorting_table_index,5);
    [DISTANCE1] = Hamilton_distance(mission_list(i),AA1,20)+Hamilton_distance(mission_list(i),AA2,20);
    all_distance=all_distance+DISTANCE1;
end