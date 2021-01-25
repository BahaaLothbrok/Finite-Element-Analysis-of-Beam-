function [Reactions, Displacements]=Output...
    (displacements,Forces,zeroDOFs,DOFs)

disp('Displacements are')
Displacements=[DOFs,displacements]

disp('Reactions are')
Reactions=[zeroDOFs,Forces(zeroDOFs)]
