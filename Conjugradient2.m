function [p,k,residual] = Conjugradient2(A,b,m0,max_k,tol,m_low,m_max)
global bounds;
% k=0;m=m0;rk=A'*(b-A*m0);residual=[];
% while k<max_k
%     k=k+1;
%     if k==1
%         p=rk;
%     else
%         bk=(rk'*rk)/(r'*r);
%         p=rk+bk*p;
%     end
%     q=A*p;
%     ak=(rk'*rk)/(q'*q);
%     m=m+ak*p;
%     if bounds==1
%         n=length(m);
%         for i=1:n
%             if  m(i,1)>m_max
%                 m(i,1)=m_max;
%             end
%             if  m(i,1)<m_low
%                 m(i,1)=m_low;
%             end
%         end
%     end
%     r=rk;
%    residual(1,k)=norm(r);
%    if norm(rk)<tol
%        break;
%    end
%     rk=rk-ak*A'*q;
%     
% end
% p=m;

end




