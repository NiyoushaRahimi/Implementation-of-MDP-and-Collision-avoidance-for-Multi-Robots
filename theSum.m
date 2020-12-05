function sigma = theSum(ValueFm1,x,u,s)

sigma=0;
direct=[1 -1 0 0; 0 0 1 -1];

for i=1:4
    xj=x+direct(:,i);
    if (xj(1,1)<1 || xj(2,1)<1 || xj(1,1)>s(1,2) || xj(2,1)>s(1,1))
        continue
    else
        sigma=sigma+ValueFm1(xj(2,1),xj(1,1))*Prob(xj,u,x);
    end
end

      
