function [p] =Conjugradient(ForwardMatrix,dObs,Wd,Wm,m0,Nmax,sigma,tolorence,m_low,m_max)
global bounds;
global nx;
global ny;
global nz;
k=0;
n=length(m0);
% A=[ForwardMatrix;sqrt(mu)*Wm];
% b=[dObs;zeros(size(Wm,1),1)];
% [p,k,residual] = Conjugradient(A,b,m0,max_iteration,tol,m_low,m_max);
% m1=inv(ForwardMatrix'*ForwardMatrix)*ForwardMatrix'*dObs;
[x, flag, relres, iter, resvec] = conjugateGradient0(ForwardMatrix'*ForwardMatrix,ForwardMatrix'*dObs, 1e-21, 300);
m1=x;
mu=(ForwardMatrix*m1-dObs)'*(ForwardMatrix*m1-dObs)/((Wm*m1)'*(Wm*m1));
Mw=Wm*(zeros(n,1)-m0);
Aw=Wd*ForwardMatrix*inv(Wm);
xy=nx*ny*nz;
Q=Aw'*Aw+mu*eye(xy(1,1));
dw=Wd*(dObs-ForwardMatrix*m0);
q=Aw'*dw;
f=Q*Mw-q;
d=-f;
t=-(d'*f)/(d'*Q*d);
while k<Nmax
    k=k+1;
    mu=sigma*mu;
    Mw=Mw+t*d;
    m=inv(Wm)*Mw+m0;
    dd=(ForwardMatrix*m-dObs)'*(ForwardMatrix*m-dObs);
    if bounds==1
       for i=1:n
         if  m(i,1)>m_max
             m(i,1)=m_max;
         end
         if  m(i,1)<m_low
             m(i,1)=m_low;
         end
       end
    end
        Mw=Wm*(m-m0);a(1,k)=dd;
    if dd<tolorence
        break;
    end
    
    Q=Aw'*Aw+mu*eye(xy(1,1));
    fk=Q*Mw-q;
    d=-fk+((fk'*fk)/(f'*f))*d;
    t=-(d'*fk)/(d'*Q*d);
    f=fk;
end
 p=inv(Wm)*Mw+m0;
figure(1)
plot(1:k,a)
k
end

