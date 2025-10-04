    

function [FoodFitness,FoodPosition,Convergence_curve]=SSA(N,Max_iter,lb,ub,dim,fobj)

if size(ub, 1) == 1 % 如果上界或下界是一个标量
    ub = ones(1, dim) .* ub; % 将其转换为与维度相匹配的行向量
    lb = ones(1, dim) .* lb; % 同上
end

% Convergence_curve = zeros(1,Max_iter);  
% 收敛曲线

%Initialize the positions of salps
SalpPositions=initialization(N,dim,ub,lb);


FoodPosition=zeros(1,dim);
FoodFitness=inf;


%calculate the fitness of initial salps

for i=1:size(SalpPositions,1)                    %i从1到种群数
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
    
    c1 = 2*exp(-(4*l/Max_iter)^2); % Eq. (3.2) in the paper
    
    for i=1:size(SalpPositions,1)
        
        SalpPositions= SalpPositions';
        
        if i<=N/2
            for j=1:1:dim
                c2=rand();
                c3=rand();
                %%%%%%%%%%%%% % Eq. (3.1) in the paper %%%%%%%%%%%%%%
                if c3<0.5 
                    SalpPositions(j,i)=FoodPosition(j)+c1*((ub(j)-lb(j))*c2+lb(j));
                else
                    SalpPositions(j,i)=FoodPosition(j)-c1*((ub(j)-lb(j))*c2+lb(j));
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
            
        elseif i>N/2 && i<N+1
            point1=SalpPositions(:,i-1);
            point2=SalpPositions(:,i);
            
            SalpPositions(:,i)=(point2+point1)/2; % % Eq. (3.4) in the paper
        end
        
        SalpPositions= SalpPositions';
    end
    
 
 % 边界处理
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



