function [sorting_table_index]= assign_sorting_table(end_node,sorting_table)
global Nrow Ncol;
for i =1:length(sorting_table(:,1))
    distance(i)=Hamilton_distance(end_node,sorting_table(i,1),Nrow,Ncol);
end
[temp,sorting_table_index]=min(distance);