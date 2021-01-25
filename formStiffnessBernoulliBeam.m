function [Stiffness,Forces]=...
    formStiffnessBernoulliBeam(DOFs,elemNo,elemNodes,x,EI,P)

Forces =zeros(DOFs,1);
Stiffness=zeros(DOFs,DOFs);

% Calculations
for e=1:elemNo
    %DOFs of Element
    oneElemNodes=elemNodes(e,:);   %[1,2] for example first element;
    elementDof=[2*(oneElemNodes(1)-1)+1 2*(oneElemNodes(2)-1) ...
        2*(oneElemNodes(2)-1)+1 2*(oneElemNodes(2)-1)+2];  %[1,2,3,4] for ex;
    
    elemLength=x(oneElemNodes(2))-x(oneElemNodes(1));
    
    %Element Stiffness for each element
    k=(EI/(elemLength^3))*[12 6*elemLength -12 6*elemLength;...
        6*elemLength 4*elemLength^2 -6*elemLength 2*elemLength^2;...
        -12 -6*elemLength 12 -6*elemLength;...
        6*elemLength 2*elemLength^2 -6*elemLength 4*elemLength^2];
    
    % Equavelent Forces at each node  4 DOFs; for each element
    forcesEq=(elemLength*P/6)*[3; elemLength/2; 3; -elemLength/2];
    
    % Force Vector
    Forces(elementDof)=Forces(elementDof)+forcesEq;
    
    % Stiffness Matrix
    Stiffness(elementDof,elementDof)=Stiffness(elementDof,elementDof)+ k;
end
