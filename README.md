# CrSyPSTL
Controller Synthesis for Probabilistic Signal Temporal Logic Specifications

Dorsa Sadigh, 2015

##1. Introduction:
This MATLAB package contains the implementations of the algorithm described in the paper "Safe Control under Uncertainty", Sadigh and Kapoor, Submitted to HSCC 2016. This file describes the content of the package and instructions on its use.
##2. Installation:
This code was written for MATLAB R2014b. Earlier versions may be sufficient, but not tested.

In addition to MATLAB R2014b, and the Control System Toolbox, the toolbox requires the following:
###-[Yalmip](http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Main.Download)
YALMIP is a modeling language for advanced modeling and solution of convex and nonconvex optimization problems. It is implemented by Johan Löfberg, and can be downloaded for free.

You also need to modify the following Yalmip function. This needs to be done because depends() does not return the right indices of variables for implies in Yalmip.

go to Yalmip/extras/getvariables.m and modify the file:

>function vars = getvariables(X,var)
>%GETVARIABLES (overloads sdpvar/getvariables on double)
>
>vars = [];
>
>if iscell(X) && strcmp(X{1}, 'implies')
>    vars = uniquestripped([getvariables(X{2}) getvariables(X{3})]);
>end


###-[Gurobi Optimizer](http://www.gurobi.com/)
An optimization solver for LP, QP, QCP, MIP, provided for free for academic use.

##3. Usage:
Make sure Yalmip is in your MATLAB path.


