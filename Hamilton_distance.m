function [Hamilton]=Hamilton_distance(location1,location2,Nrow,Ncol)
    [xnew1 ,ynew1]=ind2sub([Nrow,Ncol],location1);
    [xnew2 ,ynew2]=ind2sub([Nrow,Ncol],location2);
    
    Hamilton=abs(xnew1-xnew2)+abs(ynew1-ynew2);
end