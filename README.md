---
Name: Daniel Canales
Topic: (Special Topics) Finite Element Methods. What is it? Why used? Applications in electrical engineering.*
Title: Background of the Finite Element Method for Partial Differential Equations and its Application in Analyzing Piezoelectricity
---
# Finite Element Method for Partial Differential Equations and its Application in Analyzing Piezoelectricity 

## Table of Contents
- [Overview](#Overview)
- [Background](#Background)
- [Finite Element Method Overview](#Finite-Element-Method-Overview)
- [Common Applications](#Common-Applications)
- [Formulation](#Formulation)
- [Penalty Function Options](#Penalty-Function-Options)
- [References](#References)

## Overview
The finite element method (FEM) is a numerical technique used to achieve finite element analysis. More specfiically, FEM computes approximate solutions to boundary and initial-value problems of partial differential equations (PDEs). These PDEs typically arise in engineering and mathematical modeling to model, simulate, and predict the behavior of a structure (or system) in a given physical phenomenon. Examples of such physical phenomenons include heat transfer, mass transport, fluid flow, electromagnetics, and more. Given the governing equations, initial and boundary conditions, material properties of the structure, and the behavior of the structure, FEM constructs a mesh of the structure and divides it into smaller and simpler subdomains, called finite elements, connected by nodes. The behavior of the finite elements are described with equations assembled into a larger system of equations to model the entire problem, which is then solved with numerical methods. In addition to a background, history, and overview of the steps involved in FEM, this article provides an example of applying FEM to analyze piezoelectricity, the electromechanical phenomenon in which certain materials generate an electric charge in response to mechanical stress or strain.

## Background
It is important to first define the following equations, principle, and method often discussed in FEM:
- Partial differential equations
- Classifications of boundary conditions (BCs) for continuous systems 
- Principle of Energy Minimization
- Discretization

### Partial Differential Equations (PDEs)
A PDE is an equation of partial derivatives of an unknown function with respect to more than one independent variable. Many of the basic laws of science are expressed as PDEs [link to example of PDEs of laws of science]. The order of a PDE is determined by the highest-order partial derivative appearing in the PDE. Once the order of a PDE has been determined, their classifications can be determined. It is important to understand the different types of PDEs and their suitability with the use of FEM. For example, second-order linear PDEs can be described as the following:
- Hyperbolic: PDEs that describe time-dependent, conservative physical processes (e.g. convection) that are not evovling toward a steady state. Their solutions neither grows nor decays over with time.
- Parabolic: PDEs that describe time-dependent, dissipative physical processes (e.g. diffusion) that are evolving toward a steady state. Their solutions exponentially decay over time.
- Elliptic: PDEs that describe systems that are time-dependent and have already reached a steady state. 

Such characteristics for a given PDE determine what inital and boundary conditions result in a well-posted problem. For example, below is the one-dimensional heat equation, a PDE which describtes the the variation of temperature with respective to time and space in a one-dimensional object:
$$
\frac{\partial u}{\partial t} = \alpha \frac{\partial^2 u}{\partial x^2}
$$

with given initial conditions
$$
u(0,x) = f(x)
u_t(0,x) = g(x)
0 \lt x \lt L,
\]$$


and boundary conditions

where: 
  - $\( u(x,t) \)$ is the temperature distribution in the material as a function of space $(\(x\))$ and time $(\(t\))$,
  - $\( \frac{\partial u}{\partial t} \)$ is the partial derivative of $u$ with respect to time, representing the rate of change of temperature with respect to time,
  - $\( \frac{\partial^2 u}{\partial x^2} \)$ is the second partial derivative of $u$ with respect to $x$, representing the spatial curvature of the temperature distribution,
  - $\( \alpha \)$ is the thermal diffusivity, which depends on the material's thermal conductivity, density, and specific heat.

### Classifications of boundary conditions (BCs) for continuous systems 
In general, BCs for continuous systems are classified into two types:
- Geometric (Essential) BCs: conditions which satisfy geometric constraints
- Force (Natural) BCs: conditions which satisfy constraints prescribes by forces and moments
For example, if the FEM was applied to describe the transverse displacement, **$u(x)$**, of a cantilever beam, then its PDE would have a geometric BC on the fixed end and a force BC on the free end. The transverse displacement and its derivative would be equal to zero at the fixed end (**$u(x=0)=0$** and **$u^'(x=0)=0$**. However, on the free end, the transverse displacement must satisfy free and moment balances


### Principle of Energy Minimization

### Discretization

- FEM solvers
- Examples of types of FEM


## Finite Element Method Overview
From a methmatical point of view, FEM can be divded into five essential steps:
1) Problem Formulation
2) Weak Formulation
3) Discretization
4) Element Equations
5) Assembly
6) Boundary Conditions
7) Solution of Linear Systems
  - FEM solvers
  - Examples of types of FEM
9) Post processing
10) Mesh Adaptive Refinement

## Application of FEM in Analyzing Piezoelectricity 
The exterior penalty function method has various applications that take advantage of its robust and convenient computation of optimization under constraints. One of the primary limitations on use cases for the exterior method is that the intermediate iterations present infeasible solutions. This can make the method unsuitable for applications such as optimal control, where intermediate results are incorporated into the system’s behavior, and violation of constraints would negatively impact the response [2]. The following are *some* of the most pertinent applications of the exterior penalty method.




## References

1. Choi, S.-K., Grandhi, R. V., &amp; Canfield, R. A. (2007). Chapter 5: Reliability-based Structural Optimization. In Reliability-based Structural Design (pp. 153–202), Springer-Verlag London.
2. Malisani, P., Chaplais, F., &amp; Petit, N. (2014). An interior penalty method for optimal control problems with state and input constraints of Nonlinear Systems. Optimal Control Applications and Methods, 37(1), 3–33. https://doi.org/10.1002/oca.2134 
3. Heath, M. T. (2009). Chapter 6: Optimization. In Scientific computing: An introductory survey (pp. 256–308), McGraw Hill.
4. Knight, J. (2018). Al-Kash. In Encyclopedia.com. Infonautics Corp.
5. Saad, Y. (2020). Iterative methods for linear systems of equations: A brief historical journey. 75 Years of Mathematics of Computation, 197–215. https://doi.org/10.1090/conm/754/15141 
6. Ebenau, C., Rottschäfer, J., &amp; Thierauf, G. (2005). An advanced evolutionary strategy with an adaptive penalty function for mixed-discrete structural optimisation. Advances in Engineering Software, 36(1), 29–38. https://doi.org/10.1016/j.advengsoft.2003.10.008 
7. Yeniay, Ö. (2005). Penalty function methods for constrained optimization with genetic algorithms. Mathematical and Computational Applications, 10(1), 45–56. https://doi.org/10.3390/mca10010045 
8. Grasmeyer, J., &amp; Grasmeyer, J. (1997). Application of a genetic algorithm with adaptive penalty functions to airfoil design. 35th Aerospace Sciences Meeting and Exhibit. https://doi.org/10.2514/6.1997-7
9. Reddy, J. N. (1982). On penalty function methods in the finite-element analysis of flow problems. International Journal for Numerical Methods in Fluids, 2(2), 151–171. https://doi.org/10.1002/fld.1650020204 
10. Kim, S. J., &amp; Kim, J. H. (1993). Finite element analysis of laminated composites with contact constraint by extended interior penalty methods. International Journal for Numerical Methods in Engineering, 36(20), 3421–3439. https://doi.org/10.1002/nme.1620362003 
11. Bäeck Thomas, Fogel, D., Michalewicz, Z., Coit, D. W., &amp; Smith, A. E. (1995). Section C 5.2: Penalty Functions. In Handbook of Evolutionary Computation, Oxford University Press and Institute of Physics Publishing.
12. Michalewicz, Z. (1995). Genetic Algorithms, Numerical Optimization, and Constraints. Retrieved December 7, 2022, from https://cs.adelaide.edu.au/~zbyszek/Papers/p16.pdf.
13. Nocedal, J., &amp; Wright, S. J. (2006). Chapter 17: Penalty and Augmented Lagrangian Methods. In Numerical optimization (pp. 497–528), Springer.
14. Luenberger, D., &amp; Ye, Y. (2008). Chapter 13: Penalty and Barrier Methods. In Linear and nonlinear programming (4th ed., Ser. Operations Research and Management Science, pp. 409–440), Springer. 
