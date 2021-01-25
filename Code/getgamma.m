function Gamma = getgamma(A,B,C)
p=size(B,2);
n=size(A,1);
Gamma=zeros(1,p);

for i=1:p
    for j=0:n
         if any(C(i,:)*A^j*B)~=0
         gamma= j+1;
         Gamma(1,i)=gamma;
         break 
         end    
    end
end


end



