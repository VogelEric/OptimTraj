function [c, ceq] = collectConstraints(t,x,u,p,defects, pathCst, bndCst)
% [c, ceq] = collectConstraints(t,x,u,p,defects, pathCst, bndCst)
%
% OptimTraj utility function.
%
% Collects the defects, calls user-defined constraints, and then packs
% everything up into a form that is good for fmincon.
%
% INPUTS:
%   t = time vector
%   x = state matrix
%   u = control matrix
%   p = parameter matrix
%   defects = defects matrix
%   pathCst = user-defined path constraint function
%   bndCst = user-defined boundary constraint function
%
% OUTPUTS:
%   c = inequality constraint for fmincon
%   ceq = equality constraint for fmincon
%

ceq_dyn = reshape(defects,numel(defects),1);

%%%% Compute the user-defined constraints:
if isempty(pathCst)
    c_path = [];
    ceq_path = [];
else
    [c_pathRaw, ceq_pathRaw] = pathCst(t,x,u);
    c_path = reshape(c_pathRaw,numel(c_pathRaw),1);
    ceq_path = reshape(ceq_pathRaw,numel(ceq_pathRaw),1);
end
if isempty(bndCst)
    c_bnd = [];
    ceq_bnd = [];
else
    t0 = t(1);
    tF = t(end);
    x0 = x(:,1);
    xF = x(:,end);
    if nargin(bndObj)==4 %4 arguments without paramater(s)
        [c_bnd, ceq_bnd] = bndCst(t0,x0,tF,xF);
    elseif nargin(bndObj)==5 %5 arguments with paramater(s)
        [c_bnd, ceq_bnd] = bndCst(t0,x0,tF,xF,p);
    end
end

%%%% Pack everything up:
c = [c_path;c_bnd];
ceq = [ceq_dyn; ceq_path; ceq_bnd];

end


