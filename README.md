# CrSyPSTL
Controller Synthesis for Probabilistic Signal Temporal Logic Specifications

Dorsa Sadigh, 2015

1. Introduction:
This MATLAB package contains the implementations of the algorithm described in the paper "Safe Control under Uncertainty", Sadigh and Kapoor, Submitted to HSCC 2016. This file describes the content of the package and instructions on its use.
2. Installation:
This code was written for MATLAB R2014b. Earlier versions may be sufficient, but not tested.

In addition to MATLAB R2014b, and the Control System Toolbox, the code requires:
-Yalmip: http://users.isy.liu.se/johanl/yalmip/

In addition, Change the following Yalmip function. This needs to be done because depends() does not return the right indices of variables for implies in yalmip.

go to Yalmip/extras/getvariables.m and modify the file:

function vars = getvariables(X,var)
%GETVARIABLES (overloads sdpvar/getvariables on double)

vars = [];

if iscell(X) && strcmp(X{1}, 'implies')
    vars = uniquestripped([getvariables(X{2}) getvariables(X{3})]);
end


-Gurobi: http://www.gurobi.com/

3. Usage:
Make sure Yalmip is in your MATLAB path.


