% MATLAB CODE FOR BERNOULI BEAM ELEMENT
% CANTILIVER BEAM WITH DISTRIBUTED LOAD.

clear all, clc

%% CROSS SECTION PROPS
E=1;  I=1; L=1;  EI=E*I;  P=-1; %iInputs

%% COORDINATES AND CONNECTIVITIES
elemNo=100;   %Input, Number of elements
nodeCoord=linspace(0,L,elemNo+1)';  %Coordinates of Nodes vector  Nodes=elements+1;
nodeNo=size(nodeCoord,1);
x=nodeCoord(:,1);
% Define Elements Nodes
for i=1:elemNo
    
    elemNodes(i,1)=i;    %1,2
    elemNodes(i,2)=i+1;  %2,3  and so on
end

%% STRUCTURE
DOFs=nodeNo*2;  %Total DOF 2 per one node
displacements=zeros(nodeNo,1);

% For Stiffness Matrix and Forces;
[Stiffness,Forces]=formStiffnessBernoulliBeam(DOFs,elemNo,...
    elemNodes,x,EI,P);

%% BCs
% Cantliever Beam
fixedNodeU=[elemNodes(1)];  % only first DOF 1
fixedNodeV=[elemNodes(2)];  % only Second DOF 2
%Clamped-Clamped
%fixedNodeU=[elemNodes(1) DOFs-1];  %like 1,161
%fixedNodeV=[elemNodes(2) DOFs];  %like 1,162
zeroDOFs=[fixedNodeU; fixedNodeV];

%% Solution
% Displacements for Active DOFs
activeDOFs=setdiff([1:DOFs]',zeroDOFs);
U=Stiffness(activeDOFs,activeDOFs)\Forces(activeDOFs); %Ax=B x=A\B;

% Final Displacements for All DOFs
displacements(activeDOFs)=U;

[Reactions, Displacements]=Output...
   (displacements,Forces,zeroDOFs,[1:DOFs]');

disp('Max Vertical Displacement')
displacements(DOFs-1)

disp('Max Rotational Displacement')
displacements(DOFs)

% drawing deformed shape
D=displacements(1:2:2*nodeNo);
figure
plot(x,D,'-ob','LineWidth',1.2,'MarkerIndices',1:5:length(D))
title('Deformation of the Beam')
xlabel('x')
ylabel('w')
ylim([-0.3 0.3])
yticks(-0.3:0.05:0.3)
grid on
hold on;
y=zeros(1,length(x));
plot(x,y,'k','LineWidth',2)
