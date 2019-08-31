%load map.txt
%initial_input
function [map,grid1,Nrow,Ncol,sorting_table,Obstacle]=initial_input()
load map.txt
global Nrow Ncol;
% Nrow Indicates the number of columns in the map file, which is the maximum value of the x-axis coordinates in the graph.
% Ncol Indicates the number of rows in the map file, which is the maximum value of the y-axis coordinates in the graph.
Nrow=length(map(1,:));Ncol=length(map(:,1));

Obstacle=[];
platform=[];
parking_space=[];
%when robot no cargo; 
grid1=zeros(Nrow,Ncol);
for i = 1:Nrow
	for j =1:Ncol
    	grid1(i,j)=(j-1)*Nrow+i;
    	switch(map(Ncol-j+1,i))
    		case 1
    			Obstacle=union(Obstacle,grid1(i,j));
    		case 2 
    			platform=union(platform,grid1(i,j));
    		case 3
    			parking_space=union(parking_space,grid1(i,j));
    	end
    end
end
if Ncol==20
    sorting_table1=[122,222,342];
elseif Ncol==50
    sorting_table1=[353,1053,1803];
elseif Ncol==11
    sorting_table1=platform;
else
%      sorting_table1=[fix(Ncol/4)*Nrow+3,fix(Ncol/2)*Nrow+3,fix(Ncol/4)*3*Nrow+3];
     sorting_table1=platform([4,10,16]);
end
sorting_table=[];
if Ncol==11
    for i=1:length(sorting_table1)
        [ia,ib]=ind2sub([Nrow,Ncol],sorting_table1(i));
        sorting_table(i,1)=(ib-1)*Nrow+ia-1;sorting_table(i,2)=(ib-1-1)*Nrow+ia-1;sorting_table(i,3)=(ib-1-1)*Nrow+ia;
        sorting_table(i,4)=(ib-1-1)*Nrow+ia+1;sorting_table(i,5)=(ib-1)*Nrow+ia+1;
    end
else
    for i=1:length(sorting_table1)
        [ia,ib]=ind2sub([Nrow,Ncol],sorting_table1(i));
        sorting_table(i,1)=(ib+1-1)*Nrow+ia;sorting_table(i,2)=(ib+1-1)*Nrow+ia-1;sorting_table(i,3)=(ib-1)*Nrow+ia-1;
        sorting_table(i,4)=(ib-1-1)*Nrow+ia-1;sorting_table(i,5)=(ib-1-1)*Nrow+ia;
    end
end
end