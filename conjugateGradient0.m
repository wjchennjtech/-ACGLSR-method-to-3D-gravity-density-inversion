function [x, flag, relres, iter, resvec] = conjugateGradient0(A, b, tol, maxit)
% conjugateGradient: Solve a system of linear equations using the Conjugate Gradient Algorithm
%
% Inputs:
% A - coefficient matrix
% b - right-hand side vector
% tol - tolerance for stopping criterion
% maxit - maximum number of iterations
%
% Outputs:
% x - solution vector
% flag - convergence flag (0 if converged, 1 otherwise)
% relres - relative residual
% iter - number of iterations
% resvec - vector of residual norms at each iteration

n = size(A, 1);
x = zeros(n, 1);
r = b;
p = b;
rsold = r' * r;

for i = 1 : maxit
    Ap = A * p;
    alpha = rsold / (p' * Ap);
    x = x + alpha * p;
    r = r - alpha * Ap;
    rsnew = r' * r;
    if sqrt(rsnew) < tol
        flag = 0;
        relres = sqrt(rsnew) / norm(b);
        iter = i;
        resvec = sqrt(rsnew);
        break;
    end
    p = r + (rsnew / rsold) * p;
    rsold = rsnew;
end

if i == maxit
    flag = 1;
    relres = sqrt(rsnew) / norm(b);
    iter = i;
    resvec = sqrt(rsnew);
end