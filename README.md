---
Name: Daniel Canales
Topic: (Special Topics) Finite Element Methods. What is it? Why used? Applications in electrical engineering.*
Title: Finite Element Method for Partial Differential Equations and its Application in Analyzing Energy Harvested from Piezoelectric Cantilever Beams
---
# Finite Element Method for Partial Differential Equations and its Application in Analyzing Energy Harvested from Piezoelectric Cantilever Beams

## Table of Contents
- [Overview](#Overview)
- [Background](#Background)
- [Finite Element Method Overview in FEA Workflow](#Finite-Element-Method-Overview-in-FEA-Workflow)
- [Common Applications](#Common-Applications)
- [Formulation](#Formulation)
- [Penalty Function Options](#Penalty-Function-Options)
- [References](#References)

## Overview
The finite element method (FEM) is a numerical technique used to achieve finite element analysis [FEA] (https://www.comsol.com/multiphysics/finite-element-method?parent=physics-pdes-numerical-042-62). More specfiically, FEM computes approximate solutions to boundary and initial-value problems of partial differential equations (PDEs). These PDEs typically arise in engineering and mathematical modeling to model, simulate, and predict the behavior of a structure (or system) in a given physical phenomenon. Examples of such physical phenomenons include heat transfer, mass transport, fluid flow, electromagnetics, and more. Given the governing equations, initial and boundary conditions, material properties of the structure, and the behavior of the structure, FEM constructs a mesh of the structure and divides it into smaller and simpler subdomains, called finite elements, connected by nodes. The behavior of the finite elements are described with equations assembled into a larger system of equations to model the entire problem, which is then solved with numerical methods. In addition to a background and overview of the steps involved in using FEM with FEA software packages, this article provides an example of using FEM to analayze energy harvested from piezoelectric cantilever beams, in which mechanical vibration energy of the beam is converted into electrical energy. [file:///Users/danielcanales/Desktop/Conference_4_ICME2019_BUETBangladesh.pdf].

## Background
It is important to first review the background of the following items as they are often discussed in FEM:
- Partial differential equations
- Classifications of boundary conditions (BCs) for continuous systems 
- Principle of Energy Minimization

### Partial Differential Equations (PDEs)
A PDE is an equation of partial derivatives of an unknown function with respect to more than one independent variable [txtbook chapter 11]. PDEs can be described by their order and classifications. The order of a PDE is determined by the highest-order partial derivative appearing in the PDE. The classifications of first-order PDEs are linear, non-linear, and quasi-linear. For PDEs of second-order and beyond, the classification of a PDE comes down to one of following terms:
- Hyperbolic: PDEs that describe time-dependent, conservative physical processes (e.g. convection) that are not evovling toward a steady state. Their solutions neither grows nor decays over with time.
- Parabolic: PDEs that describe time-dependent, dissipative physical processes (e.g. diffusion) that are evolving toward a steady state. Their solutions exponentially decay over time.
- Elliptic: PDEs that describe systems that are time-dependent and have already reached a steady state. 

For example, consider the Euler-Bernoulli Beam PDE representing the transverse displacement, **$u(x,t)$**, of a beam  over space and time (assume the PDE corresponds to small deflections of a beam that is subject to lateral loads only and ignore the effects of shear deformations and rotary intertia) [https://www.sciencedirect.com/science/article/abs/pii/B9780128185636000171]: 

$$
EI \dfrac{\partial^4 u}{\partial x^4}= 0
$$

where

- **$E$** is the Young's modulus of the beam material
- **$I$** is the moment of inertia of the beam's cross-sectional area
- **$u=u(x,t)$** is the transverse displacement

<!-- The PDE representing the electric potential induced across the thickness of the beam, **$v(t)$**, as a function of spatial coordinate **$x$** and time **$t$** is shown below:

$$
\varepsilon A_p \frac{\partial^2 v}{\partial t^2} + g v = d_{31} A_p \frac{\partial u}{\partial x}
$$

where 

- **$\varepsilon$** is the permittivity of the piezoelectric material,
- **$A_p$** is the cross-sectional area of the piezoelectric material
- **$g$** is the electrical conductance
- **$v(x,t)$** is the electricl potential induced across the thickness of the beam
- **$d_{31}$** is the piezoelectric coupling coefficient.
-->

In regards to order and classifications, the above PDE is a fourth order, hyperbolic PDE. Note, the PDE is characterized as a hyperbolic function since the beam expresses vibrational behavior that can be analyzed using the wave equation (a hyperbolic PDE) [https://www.sciencedirect.com/science/article/abs/pii/0022460X91904015]. 


### Classifications of boundary conditions (BCs) for continuous systems 
In general, BCs for continuous systems are classified into two types [https://www.jousefmurad.com/fem/the-finite-element-method-beginners-guide/]:
- Geometric (Essential) BCs: conditions which satisfy geometric constraints
- Force (Natural) BCs: conditions which satisfy constraints prescribed by forces and moments

Consider the BCs for the above Euler-Bernoulli Beam PDE for a clamped (fixed) cantilever beam with a proof mass at the free end. Given this specific case, its PDE would have two geometric BCs on the fixed end and two force BCs on the free end with the proof mass [https://www.sciencedirect.com/science/article/abs/pii/B9780128185636000171]. The BCs are: 

$$
\left.u\right|_{x=0}=0
$$

$$
\left.\dfrac{\partial u}{\partial x}\right|_{x=0}=0
$$

$$
\left.\dfrac{\partial^2 u}{\partial x^2}\right|_{x=L}=0
$$

$$
\left.EI\dfrac{\partial^3 u}{\partial x^3}\right|_{x=L} = m\dfrac{\partial^2 u}{\partial t^2}
$$

where 
- **$L$** is the length of the beam
- **$m$** is the mass of the proof mass
- **$\dfrac{\partial^2 u}{\partial t^2}$** is the acceleration of the proof mass

The transverse displacement and its derivative would be equal to zero at the fixed end. However, the transverse displacement would satisfy the load applied by the weight of the proof mass at the free end.

### Principle of Energy Minimization
The primary driving force for FEM related to physical phenomenon is the principle of minimization of energy. When BCs are applied to the PDE of structure, the structure can tehcnically result in many configurations. However, the configuration where the total energy of the structure is at its minimum is typically the chosen configuration [https://www.simscale.com/blog/what-is-finite-element-method/]. Regarding the above example of the Euler-Bernoulli Beam PDE for a clamped cantilever beam with a proof mass at the free end, assume the structure has inital conditions such that the beam is not bent. Now, considering the BCs mentioned above (clamped at one end, and proof mass at the other end), the example's FEM should result in a solution which demonstrates that the beam will transversley vibrate due to the acceleration of the proof mass (e.g. due to gravity) and eventually come to a rest. This is because coming to a rest achieves the minimum total energy of the beam. 

## Finite Element Method Overview in FEA Workflow
From a mathmatical point of view, the below five steps overview how the FEM is working in FEA workflow:
1) Model preparation
2) Element formulation
3) Assembly
4) Solving systems of linear equations
5) Post-processing

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
10) Mesh Adaptive Refinement'

### 1. Problem Formulation
Problem formulation consists of creating the geometry of your structure, definiting material properties, creating initial and boundary conditions, and defining other conditions such as contact behaviour. 

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
