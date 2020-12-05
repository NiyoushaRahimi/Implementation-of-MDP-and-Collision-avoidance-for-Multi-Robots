function p = payF(x,u,xGoal,map)

xp1=x+u;
s=size(map);
if (xp1(1,1)<1 || xp1(2,1)<1 || xp1(1,1)> s(1,2) || xp1(2,1)>s(1,1) )
    p=2000000;
    return     
end


if (map(xp1(2,1),xp1(1,1))==1)
    payF=-1000;
else
    payF=3;
end

if (norm(xp1-xGoal)<norm(x-xGoal))
    alpha=1/(norm(xp1-xGoal)+2);
    payF=1000*alpha+payF;
end

p=payF;
