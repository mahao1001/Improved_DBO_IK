% -----------------------------------------------------------------------------------------------------------
% Sparrow Search algorithm (SSA) (demo)
% Programmed by Jian-kai Xue    
% Updated 21 Mar, 2020.                     
%
% This is a simple demo version only implemented the basic         
% idea of the SSA for solving the unconstrained problem.    
% The details about SSA are illustratred in the following paper.    
% (To cite this article):                                                
%  Jiankai Xue & Bo Shen (2020) A novel swarm intelligence optimization
% approach: sparrow search algorithm, Systems Science & Control Engineering, 8:1, 22-34, DOI:
% 10.1080/21642583.2019.1708830


clear all 
clc

SearchAgents_no=100; % Number of search agents

Function_name='F1'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper)

Max_iteration=1000; % Maximum numbef of iterations

% Load details of the selected benchmark function
[lb,ub,dim,fobj]=Get_Functions_details(Function_name);

[fMin,bestX,SSA_curve]=SSA(SearchAgents_no,Max_iteration,lb,ub,dim,fobj);  


%Draw objective space
 semilogy(SSA_curve,'Color','g')
  
axis ([0 1000 0 1 ])
title('Objective space')
xlabel('Iteration');
ylabel('Best score obtained so far');
%axis tight
grid on
box on
legend('SSA')
display(['The best solution obtained by SSA is : ', num2str(bestX)]);
display(['The best optimal value of the objective funciton found by SSA is : ', num2str(fMin)]);


