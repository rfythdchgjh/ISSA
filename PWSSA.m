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

function [FoodFitness,FoodPosition,Convergence_curve]=PWSSA(N,Max_iter,lb,ub,dim,fobj)

if size(ub, 1) == 1 % 如果上界或下界是一个标量
    ub = ones(1, dim) .* ub; % 将其转换为与维度相匹配的行向量
    lb = ones(1, dim) .* lb; % 同上
end

% Convergence_curve = zeros(1,Max_iter);

%Initialize the positions of salps
SalpPositions=initialization(N,dim,ub,lb);


FoodPosition=zeros(1,dim);
FoodFitness=inf;


%calculate the fitness of initial salps

for i=1:size(SalpPositions,1)
    SalpFitness(1,i)=fobj(SalpPositions(i,:));
end

[sorted_salps_fitness,sorted_indexes]=sort(SalpFitness);

for newindex=1:N
    Sorted_salps(newindex,:)=SalpPositions(sorted_indexes(newindex),:);
end

FoodPosition=Sorted_salps(1,:);
FoodFitness=sorted_salps_fitness(1);

%Main loop
l=2; % start from the second iteration since the first iteration was dedicated to calculating the fitness of salps
while l<Max_iter+1
    
%     c1 = 2*exp(-(4*l/Max_iter)^2); % Eq. (3.2) in the paper
    c1new=normrnd(0,1)*(1-l/Max_iter);c2new=normrnd(0,1)*(1-l/Max_iter);
    for i=1:size(SalpPositions,1)
        
        SalpPositions= SalpPositions';
        
         if i<=N/2
            for j=1:1:dim
%                 c2=rand();
                c3=rand();
                %%%%%%%%%%%%% % Eq. (3.1) in the paper %%%%%%%%%%%%%%
                if c3<0.5 
                    SalpPositions(j,i)=FoodPosition(j)+c1new*((ub(j)-lb(j))*c2new+lb(j));
                else
                    SalpPositions(j,i)=FoodPosition(j)-c1new*((ub(j)-lb(j))*c2new+lb(j));
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
          
        elseif i>N/2 && i<N+1
            w1=rand*2-1;w2=rand*2-1;w3=rand*2-1;
            point1=SalpPositions(:,i-1);
            point2=SalpPositions(:,i);
            
%             SalpPositions(:,i)=(point2+point1)/2; % % Eq. (3.4) in the paper
            SalpPositions(:,i)=w1*((w2*FoodPosition'-point2)+(w3*FoodPosition'-point1));
        end
        
        SalpPositions= SalpPositions';
    end
    
    for i = 1:size(SalpPositions, 1)
        Tp = SalpPositions(i, :) > ub; % 检查是否超出上界
        Tm = SalpPositions(i, :) < lb; % 检查是否低于下界
        
        % 生成一个标量随机数
        rand_scalar = rand;
        
        % 对超出边界的座头鲸进行位置调整
        SalpPositions(i, :) = (SalpPositions(i, :) .* ~(Tp + Tm)) + (rand_scalar * ub .* Tp) + (rand_scalar * lb .* Tm);
        
        % 重新计算适应度
        SalpFitness(1, i) = fobj(SalpPositions(i, :));
        
        if SalpFitness(1,i)<FoodFitness
            FoodPosition=SalpPositions(i,:);
            FoodFitness=SalpFitness(1,i);
            
        end
    end
    
    Convergence_curve(l-1)=FoodFitness;
    l = l + 1;
end



