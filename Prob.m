function p = Prob(xp1,u,x)

A=[0 1; 1 0];
uriver=A*u+x;
uriver2=x-(A*u);

if (xp1==u+x)
    p=0.8;

elseif ((xp1(1,1)==uriver(1,1) && xp1(2,1)==uriver(2,1)) || (xp1(1,1)==uriver2(1,1) && xp1(2,1)==uriver2(2,1)))
        p=0.1;    
else 
        p=0;
    
end

