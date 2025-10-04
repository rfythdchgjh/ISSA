%_________________________________________________________________________________
%  Salp Swarm Algorithm (SSA) source codes version 1.0
%
%  Developed in MATLAB R2016a
%
%  Author and programmer: Seyedali Mirjalili
%
%         e-Mail: ali.mirjalili@gmail.com
%                 seyedali.mirjalili@griffithuni.edu.au
%
%       Homepage: http://www.alimirjalili.com
%
%   Main paper:
%   S. Mirjalili, A.H. Gandomi, S.Z. Mirjalili, S. Saremi, H. Faris, S.M. Mirjalili,
%   Salp Swarm Algorithm: A bio-inspired optimizer for engineering design problems
%   Advances in Engineering Software
%   DOI: http://dx.doi.org/10.1016/j.advengsoft.2017.07.002
%____________________________________________________________________________________


% This function initialize the first population of search agents
function Positions=GISSAinitialization(SearchAgents_no,dim,ub,lb)

Boundary_no= size(ub,1); % numnber of boundaries

% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    Positions=rand(SearchAgents_no,dim).*(ub-lb)+lb;
end
tmp1 = [1: SearchAgents_no]'*ones(1, dim);
Ind = [1: dim];
prime1 = primes(100*dim);
[~,q]=find(prime1 >= (2*dim+3));
tmp2 = (2*pi.*Ind)/prime1(1,q(1));
tmp2 = 2*cos(tmp2);
tmp2 = ones(SearchAgents_no,1)*tmp2;
GD = tmp1.*tmp2;
GD = mod(GD,1);
% If each variable has a different lb and ub
if Boundary_no>1
    for i=1:dim
        ub_i=ub(i);
        lb_i=lb(i);
        Positions(:,i)=GD(:,i).*(ub_i-lb_i)+lb_i;
    end
%     for m=1:dim      %m为列 n为行
%         for n=1:SearchAgents_no
%         Positions1(n,m)=(ub(m)+lb(m))/2+(ub(m)+lb(m))/(2*0.9)-Positions(n,m)/0.9;
%         end
%     end
%     Positions=Positions1;
end