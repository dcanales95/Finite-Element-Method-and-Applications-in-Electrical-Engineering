---
Name: Daniel Canales
Topic: (Special Topics) Finite Element Methods. What is it? Why used? Applications in electrical engineering.*
Title: Finite Element Method in Finite Element Analysis Workflow and its Application in Analyzing Energy Harvested from a Piezoelectric Cantilever Beam
---
# Introduction to Finite Element Method of Partial Differential Equations and an Example of Its Application in Analyzing Energy Harversting from a Vibrating Piezoelectric Cantilever Beam

## Table of Contents

## Overview
The finite element method (FEM) is a numerical technique used to approximate the solution of a boundary value partidal differential equation (PDE) by solving an algebraic system of equations. FEM discretizes the domain of the partial differential equation into a mesh of smaller and simpler subdomains, called elements, connected by nodes[https://www.simscale.com/blog/what-is-finite-element-method/]. Considering boundary conditions at the nodes, a system of basis functions are calculated to model these elements. These functions are then assembled into a larger system of equations over the entire domain and solved using numerical methods

## Purpose of FEM
The purpose of FEM is to create a simulation, also known as a finite element analysis (FEA). This simulation is used to provide a structural analysis of how a particular product or design would react under physical phenomenons in the real world. FEA breaks down the entire model into smaller elements within a mesh, which engineers use to test how the different elements of a design interact and perform under physical effects (e.g. vibration, heat, fluid flow, electromagnetic forces, and others) [https://www.autodesk.com/solutions/simulation/finite-element-analysis]. Such simulations are done with software packages that implement FEM to solve the PDEs governing these physical effects. Some common FEA software packages include ANSYS [https://www.ansys.com/], SIMSCALE [https://www.simscale.com/], Abaqus FEA [https://www.3ds.com/products-services/simulia/], and COMSOL Multiphysics [https://www.comsol.com/]. Shown below is an example of FEA analysis of a cantilever beam under pressure loading solid, shell, and beam elements in Abaqus FEA software. 

<img src="AbaqusExample.jpg" width="40%" height="30%">

## Background: 
In discussing FEM it is important to first review the background of PDEs, boundary conditions, and numerial integration.

### Partial Differential Equations and Boundary Conditions
A PDE is an equation of partial derivatives of an unknown function with respect to more than one independent variable [txtbook chapter 11]. The order of a PDE is determined by the highest-order partial derivative appearing in the PDE. For example, consider the Euler-Bernoulli Beam PDE, a fourth-order PDE, representing the transverse displacement, **$u(x,t)$**, of a beam over space and time (with assumptions mentioned in [https://www.sciencedirect.com/science/article/abs/pii/B9780128185636000171]). : 

$$
EI \dfrac{\partial^4 u}{\partial x^4}= 0
$$

where **$E$** is the Young's modulus of the beam material and **$I$** is the moment of inertia of the beam's cross-sectional area. Now, to limit the possible solutions of the PDE, and any PDE in general, boundary conditions (BCs) must be provided. In doing so, the PDE becomes a boundary-value problem. Consider the BCs for the above Euler-Bernoulli Beam PDE for a clamped (fixed) piezoelectric cantilever beam with a proof mass at the free end [file:///Users/danielcanales/Desktop/Conference_4_ICME2019_BUETBangladesh.pdf]. In such a case, the proof mass is approximated as a shear force applied at the free end.

<img src="PzCantileverBeam.png" width="40%" height="30%">

The BCs are: 

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

where **$L$** is the length of the beam, **$m$** is the mass of the proof mass, and **$\dfrac{\partial^2 u}{\partial t^2}$** is the acceleration of the proof mass. Note, with FEM, there are not only BCs for the domain of the problem, but also boundary conditions at the nodes of the problem's mesh. 

### Numerical integration
Numerical integration is an approximation of the definite integral of a function over a specific interval [notes Chapter 8]. As shown below, numerical integration assumes the the definitive integral, $I(f)$, over an interval, $[a,b]$, can be approximated as a quadrature rule, $Q_n(f)$, which is a weighted sum of $n$-number of sample values of the integrand function. 

$$
I(f) = \int_{a}^{b} f(x)dx
$$

$$
I(f) \approx Q_n(f) = \sum_{i=1}^n w_i f\left(x_i\right)
$$

where $x_i$ are called nodes and multipliers $w_i$ are called weights. There are several methods to determine the nodes and weights of the quadrature rule, such as equally spacing the nodes in the given interval (Newton-Cotes quadrature method) or using polynomials to interpolate between the given interval (Gaussian Quadtrature method). In the latter approach, the weights $w_i$ become polynomial functions $w_i(x)$, and they are chosen to maximize the degree of freedom of the resulting rule. In FEM, the Gaussian Quadrature method is used to to obtain the weighted functions, later referred to as the system of basis functions that models the elements of the problem's mesh.

## Steps of the Finite Element Method 
The FEM of a boundary-value problem can be generalized into the following steps [https://davis.wpi.edu/~matt/courses/fem/fem.htm, https://www.wias-berlin.de/people/john/LEHRE/NUM_PDE_FUB/num_pde_fub_4.pdf]:
- Step 1: Discretization of the problem's domain
- Step 2: Selection of interpolation functions
- Step 3: Assembly of interpolation functions into a larger system of equations over the entire domain 
- Step 4: Solution of the system of equations

There are two main approaches for achieving the above steps: the Ritz method and Galerkin method [https://www.wias-berlin.de/people/john/LEHRE/NUM_PDE_FUB/num_pde_fub_4.pdf]. The above steps will be explained with a 1D problem using Galerkin's method [https://www.iue.tuwien.ac.at/phd/orio/node45.html, https://web.iitd.ac.in/~hegde/fem/lecture/lecture5.pdf]. Consider the below problem:

$$
\begin{equation}
\tag{1}
  L[u(\vec{r})]=f(\vec{r})
\end{equation}
$$

defined in the problem domain **$\Omega$**, where **$L[\cdot]$** represents a linear differential operator, **$u(\vec{r})$** is the unknown function to be determined, and **$f(\vec{r})$** is a given source function.

### Discretization of the problem's domain
The discretization process will turn the problem into a linear system of equations over the entire problem domain. The process begins with transforming the PDE into an integral form, known as its weak form. [https://www.simscale.com/blog/what-is-finite-element-method/]. In this specific example, the weak form of the problem will not be explicitly derived, but left as a variational formulation. Multiplying a test function **$v(\vec{r})$** to (1) and integrating over the problem domain gives the variational fomrulation [https://vtechworks.lib.vt.edu/server/api/core/bitstreams/a792e408-2fb1-44ef-b862-ad32fb66ec45/content]:

$$
\begin{equation}
\tag{2}
  \int_{\Omega} v(\vec{r}) L[u(\vec{r})] d \Omega=\int_{\Omega} v(\vec{r}) f(\vec{r}) d \Omega
\end{equation}
$$

Using the notation $(a, b)=\int_{\Omega} a(\vec{r}) b(\vec{r}) d \Omega$, (2) can be written as

$$
\begin{equation}
\tag{3}
  (L[u], v)=(f, v)
\end{equation}
$$

Now that the problem is represented in an integral form, $\Omega$, can be divided into a set of $m$ elements, $T_{1}, T_{2}, \ldots, T_{m}$, which do not overlap. The mesh obtained by such a domain discretization is represented by

$$
\begin{equation}
\tag{4}
  T_{h}(\Omega)=\bigcup_{i=1}^{m} T_{i}
\end{equation}
$$

with a set of $N$ grid points. These grid points are the nodes of the problem's mesh. By using numerical integration, the approximate solution, $u_{h}(\vec{r})$, for $u(\vec{r})$ in (3) is given by the following weighted function [C. Johnson, Numerical Solution of Partial Differential Equations by The Finite Element Method.Cambridge University Press, 1987.]:

$$
\begin{equation}
\tag{5}
  u_{h}(\vec{r})=\sum_{i=1}^{N} u_{i} N_{i}(\vec{r})
\end{equation}
$$

where $N_{i}(\vec{r})$ are the basis functions. They are often referred to as the shape functions or interpolation functions of the FEM, and will be the focus of Step 2. The approximate solution of (3) is determined by the coefficients $u_{i}$. At node $i$, where the point is given by the coordinates $\vec{r}_{i}$, the basis functions must satisfy the below condition

$$
\begin{equation}
\tag{6}
  N_{j} \left(\vec{r}_{i}\right) = \delta _{ij}, \quad i, j=1, \ldots, N
\end{equation}
$$

For reasons that will be discussed in Step 2, the basis functions are typically chosen to be low order polynomials.

By substituting (5) in (4), and choosing $v=N_{j}(\vec{r})$, one obtains

$$
\begin{equation}
\tag{7}
  \left(L\left[\sum_{i=1}^{N} u_{i} N_{i}\right], N_{j}\right)=\left(f, N_{j}\right), \quad j=1, \ldots, N
\end{equation} 
$$

and since $L[\cdot]$ is a linear operator and the coefficients $u_{i}$ are constants, one can use the product rule in $L[\cdot]$ and then use numerical integration to write

$$
\begin{equation}
\tag{8}
  \sum_{i=1}^{N} u_{i}\left(L\left[N_{i}\right], N_{j}\right)=\left(f, N_{j}\right), \quad j=1, \ldots, N
\end{equation}
$$

Notice that Equation (8) is a linear system of $N$ equations with $N$ unknowns, $u_{1}, u_{2}, \ldots, u_{N}$. Therefore, (7) can be written in matrix notation as

$$
\begin{equation}
\tag{8}
  \mathbf{A x}=\mathbf{b}
\end{equation}
$$

where $\mathbf{A}=\left(a_{i j}\right)$ is called stiffness matrix, given by the elements

$$
\begin{equation}
\tag{9}
  a_{i j}=\left(L\left[N_{i}\right], N_{j}\right)=\int_{\Omega} L\left[N_{i}(\vec{r})\right] N_{j}(\vec{r}) d \Omega, \quad i, j=1, \ldots, N,
\end{equation}
$$

$\mathbf{x}=\left(u_{1}, \ldots, u_{N}\right)^{T}$ is the vector of unknown coefficients, and $\mathbf{b}=\left(b_{1}, \ldots, b_{N}\right)^{T}$ is the load vector, given by

$$
\begin{equation}
\tag{10}
  b_{j}=\left(f, N_{j}\right)=\int_{\Omega} f(\vec{r}) N_{j}(\vec{r}) d \Omega, \quad j=1, \ldots, N
\end{equation}
$$

As a result, by implementing the Galerkin Method, the problem in (1) has been discretized into a system of elements connected at nodes and can be expressed as (8); a system of equations over the entire problem domain. 

### Selection of interpolation functions
As mentioned earlier, the shape function $N_i$ is the function which interpolates the solution between the discrete values $u_i$ obtained at the mesh nodes. Therefore, appropriate functions have to be used and, as already mentioned, low order polynomials are typically chosen as shape functions (also mentioned in earlier discuss of numerical integration). 


In this work linear shape functions are used.

For three-dimensional finite element simulations it is convenient to discretize the simulation domain using tetrahedrons, as depicted in Figure 4.1. Thus, linear shape functions must be defined for each tetrahedron of the mesh, in order to apply the Galerkin method described in Section 4.1.1.

![](https://cdn.mathpix.com/cropped/2023_12_12_c034fdc06d3e6ebd37c6g-1.jpg?height=1063&width=1908&top_left_y=962&top_left_x=106)

Figure 4.1: Finite element mesh of a three-dimensional interconnect structure discretized with tetrahedrons.

Consider a tetrahedron in a cartesian system as depicted in Figure 4.2(a). The linear shape function of the node $i$ has the form [153]

$$
N_{i}(x, y, z)=a_{i}+b_{i} x+c_{i} y+d_{i} z,
$$

where $i=1, \ldots, 4$. The coefficients, $a_{i}, b_{i}, c_{i}$, and $d_{i}$ for each nodal basis function of the tetrahedral element can be calculated considering the condition [152]

$$
N_{j}\left(\vec{r}_{i}\right)=\delta_{i j}, \quad i, j=1, \ldots, 4
$$

As a result, a system of 4 equations for the 4 unknown coefficients is obtained. This procedure has to be repeated for all tetrahedrons of the mesh, so that the basis functions of all grid nodes are determined. Furthermore, in order to obtain the discrete system of equations (․9), the shape functions have to be derived and integrated, as shown by (4.11) and (4.12).

![](https://cdn.mathpix.com/cropped/2023_12_12_c034fdc06d3e6ebd37c6g-2.jpg?height=796&width=898&top_left_y=548&top_left_x=169)

(a)

![](https://cdn.mathpix.com/cropped/2023_12_12_c034fdc06d3e6ebd37c6g-2.jpg?height=800&width=898&top_left_y=543&top_left_x=1058)

(b)

Figure: Tetrahedral finite element. (a) Original coordinate system. (b) Transformed coordinate system.

The calculations can be significantly simplified by carring out a coordinate transformation. A tetrahedron in a transformed coordinate system is shown in Figure $4.2(\underline{b})$. Each point $(x, y, z)$ of the tetrahedron in the original coordinate system can be mapped to a corresponding point $(\xi, \eta, \zeta)$ in the transformed coordinate system [155]

$$
\begin{gathered}
x=x_{1}+\left(x_{2}-x_{1}\right) \xi+\left(x_{3}-x_{1}\right) \eta+\left(x_{4}-x_{1}\right) \zeta \\
y=y_{1}+\left(y_{2}-y_{1}\right) \xi+\left(y_{3}-y_{1}\right) \eta+\left(y_{4}-y_{1}\right) \zeta \\
z=z_{1}+\left(z_{2}-z_{1}\right) \xi+\left(z_{3}-z_{1}\right) \eta+\left(z_{4}-z_{1}\right) \zeta
\end{gathered}
$$

which in matrix form leads to the Jacobian matrix

$$
\mathbf{J}=\left[\begin{array}{ccc}
x_{2}-x_{1} & x_{3}-x_{1} & x_{4}-x_{1} \\
y_{2}-y_{1} & y_{3}-y_{1} & y_{4}-y_{1} \\
z_{2}-z_{1} & z_{3}-z_{1} & z_{4}-z_{1}
\end{array}\right]
$$

In this way, the nodal basis functions for the tetrahedron in the transformed coordinate system are given by [155]

$$
\begin{aligned}
& N_{1}^{t}(\xi, \eta, \zeta)=1-\xi-\eta-\zeta, \\
& N_{2}^{t}(\xi, \eta, \zeta)=\xi \\
& N_{3}^{t}(\xi, \eta, \zeta)=\eta \\
& N_{4}^{t}(\xi, \eta, \zeta)=\zeta
\end{aligned}
$$

These shape functions are rather simple, so that the derivatives and integrals required for the finite element formulation can be readily evaluated in the transformed coordinate system. Given a function $f(x, y, z)$, the gradient in the transformed coordinates is of the form

$$
\nabla^{t} f=\left[\begin{array}{lll}
\frac{\partial f}{\partial \xi} & \frac{\partial f}{\partial \eta} & \frac{\partial f}{\partial \zeta}
\end{array}\right]^{T}
$$

where the derivatives are calculated via the chain rule by

$$
\begin{aligned}
& \frac{\partial f}{\partial \xi}=\frac{\partial f}{\partial x} \frac{\partial x}{\partial \xi}+\frac{\partial f}{\partial y} \frac{\partial y}{\partial \xi}+\frac{\partial f}{\partial z} \frac{\partial z}{\partial \xi} \\
& \frac{\partial f}{\partial \eta}=\frac{\partial f}{\partial x} \frac{\partial x}{\partial \eta}+\frac{\partial f}{\partial y} \frac{\partial y}{\partial \eta}+\frac{\partial f}{\partial z} \frac{\partial z}{\partial \eta} \\
& \frac{\partial f}{\partial \zeta}=\frac{\partial f}{\partial x} \frac{\partial x}{\partial \zeta}+\frac{\partial f}{\partial y} \frac{\partial y}{\partial \zeta}+\frac{\partial f}{\partial z} \frac{\partial z}{\partial \zeta}
\end{aligned}
$$

These equations can be expressed in matrix notation as

$$
\left[\begin{array}{l}
\frac{\partial f}{\partial \xi} \\
\frac{\partial f}{\partial \eta} \\
\frac{\partial f}{\partial \zeta}
\end{array}\right]=\left[\begin{array}{lll}
\frac{\partial x}{\partial \xi} & \frac{\partial y}{\partial \xi} & \frac{\partial z}{\partial \xi} \\
\frac{\partial x}{\partial \eta} & \frac{\partial y}{\partial \eta} & \frac{\partial z}{\partial \eta} \\
\frac{\partial x}{\partial \zeta} & \frac{\partial y}{\partial \zeta} & \frac{\partial z}{\partial \zeta}
\end{array}\right] \cdot\left[\begin{array}{l}
\frac{\partial f}{\partial x} \\
\frac{\partial f}{\partial y} \\
\frac{\partial f}{\partial z}
\end{array}\right]
$$

or

$$
\nabla^{t} f=\mathbf{J}^{T} \nabla f
$$

where $\mathbf{J}^{T}$ is the transpose of the Jacobian matrix. Thus, the gradient in the original coordinate system can be calculated using the transformed coordinate gradient by

$$
\nabla f=\left(\mathbf{J}^{T}\right)^{-1} \nabla^{t} f=\boldsymbol{\Lambda} \nabla^{t} f
$$

where $\boldsymbol{\Lambda}=\left(\mathbf{J}^{T}\right)^{-1}$.

Performing such a coordinate transformation significantly simplifies the practical implementation of the FEM. The nodal shape functions in the transformed coordinates are fixed and known in advance, thus, it is not necessary to solve the system of equations formed by (4.15) and (4.16) for each element of the mesh. Only the Jacobian matrix has to be determined, and the required calculations for the finite element formulation can be easily evaluated.


<!-- 
# Finite Element Method in Finite Element Analysis Workflow and its Application in Analyzing Energy Harvested from a Piezoelectric Cantilever Beam

## Table of Contents
- [Overview](#Overview)
- [Background](#Background)
- [Finite Element Method Overview in FEA Workflow](#Finite-Element-Method-Overview-in-FEA-Workflow)
- [Common Applications](#Common-Applications)
- [Formulation](#Formulation)
- [Penalty Function Options](#Penalty-Function-Options)
- [References](#References)

## Overview
The finite element method (FEM) is a numerical technique used to achieve finite element analysis [FEA] (https://www.comsol.com/multiphysics/finite-element-method?parent=physics-pdes-numerical-042-62). More specfiically, FEM computes approximate solutions to boundary and initial-value problems of partial differential equations (PDEs). These PDEs typically arise in engineering and mathematical modeling to model, simulate, and predict the behavior of a structure (or system) in a given physical phenomenon. Examples of such physical phenomenons include heat transfer, mass transport, fluid flow, electromagnetics, and more. Given the governing equations, initial and boundary conditions, material properties of the structure, and the behavior of the structure, FEM constructs a mesh of the structure and divides it into smaller and simpler subdomains, called finite elements, connected by nodes. The behavior of the finite elements are described with equations assembled into a larger system of equations to model the entire problem, which is then solved with numerical methods. In addition to a background and overview of the steps involved in using FEM with FEA software packages, this article provides an example of using FEM to analayze energy harvested from a vibrating piezoelectric cantilever beam.

## Background
In discussing FEM and its application towards FEA of a piezoelectric cantilever beam, it is important to first review the background of the following items:
- Partial differential equations
- Classifications of boundary conditions (BCs) for continuous systems 
- Principle of Energy Minimization
- Piezoelectric Effect 

### Partial Differential Equations (PDEs)
A PDE is an equation of partial derivatives of an unknown function with respect to more than one independent variable [txtbook chapter 11]. PDEs can be described by their order and classifications. The order of a PDE is determined by the highest-order partial derivative appearing in the PDE. The classifications of first-order PDEs are linear, non-linear, and quasi-linear. For PDEs of second-order and beyond, their classification comes down to one of following terms:
- Hyperbolic: PDEs that describe time-dependent, conservative physical processes (e.g. convection) that are not evovling toward a steady state. Their solutions neither grows nor decays over with time.
- Parabolic: PDEs that describe time-dependent, dissipative physical processes (e.g. diffusion) that are evolving toward a steady state. Their solutions exponentially decay over time.
- Elliptic: PDEs that describe systems that are time-dependent and have already reached a steady state. 

For example, consider the Euler-Bernoulli Beam PDE representing the transverse displacement, **$u(x,t)$**, of a beam over space and time (assume the PDE corresponds to small deflections of a beam that is subject to lateral loads only and ignore the effects of shear deformations and rotary intertia) [https://www.sciencedirect.com/science/article/abs/pii/B9780128185636000171]: 

$$
EI \dfrac{\partial^4 u}{\partial x^4}= 0
$$

where **$E$** is the Young's modulus of the beam material and **$I$** is the moment of inertia of the beam's cross-sectional area.

In regards to order and classifications, the above PDE is a fourth order, hyperbolic PDE. Note, the PDE is characterized as a hyperbolic function since the beam expresses vibrational behavior that can be analyzed using the wave equation (a hyperbolic PDE) [https://www.sciencedirect.com/science/article/abs/pii/0022460X91904015]. 


### Classifications of boundary conditions (BCs) for continuous systems 
In general, BCs for continuous systems are classified into two types [https://www.jousefmurad.com/fem/the-finite-element-method-beginners-guide/]:
- Geometric (Essential) BCs: conditions which satisfy geometric constraints
- Force (Natural) BCs: conditions which satisfy constraints prescribed by forces and moments

Consider the BCs for the above Euler-Bernoulli Beam PDE for a clamped (fixed) piezoelectric cantilever beam with a proof mass at the free end [file:///Users/danielcanales/Desktop/Conference_4_ICME2019_BUETBangladesh.pdf]. Given this specific case, its PDE would have two geometric BCs on the fixed end and two force BCs on the free end with the proof mass [https://www.sciencedirect.com/science/article/abs/pii/B9780128185636000171]. 

<img src="PzCantileverBeam.png" width="40%" height="30%">

The BCs are: 

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

where **$L$** is the length of the beam, **$m$** is the mass of the proof mass, and **$\dfrac{\partial^2 u}{\partial t^2}$** is the acceleration of the proof mass. The transverse displacement and its derivative are equal to zero at the fixed end. However, the transverse displacement is equal to the load applied by the weight of the proof mass at the free end.

### Principle of Energy Minimization
The primary driving force for FEM related to physical phenomenon is the principle of minimization of energy. When BCs are applied to the PDE of structure, the structure can tehcnically result in many configurations. However, the configuration where the total energy of the structure is at its minimum is typically the chosen configuration [https://www.simscale.com/blog/what-is-finite-element-method/]. Regarding the above example of the Euler-Bernoulli Beam PDE for a clamped cantilever beam with a proof mass at the free end, assume the structure has inital conditions such that the beam is not bent. Now, considering the BCs mentioned above (clamped at one end, and proof mass at the other end), the example's FEM should result in a solution which demonstrates that the beam will transversley vibrate due to the acceleration of the proof mass (e.g. due to gravity) and eventually come to a rest. This is because coming to a rest achieves the minimum total energy of the beam. 

### Piezoelectric Effect
The piezoelectric effect is the ability for a piezoelectric material to generate electric charge density in response to a mechanical struss, such  as pressure, vibration, or force [file:///Users/danielcanales/Desktop/Conference_4_ICME2019_BUETBangladesh.pdf]. In the field of electrical engineering, this piezoelectric mechanism is a method used to build energy harvesting systems [Bhuyan MS, Majlis BY, Othman M, Ali SH, Islam MS. Development of a Fluid Actuated Piezoelectric Micro
Energy Harvester: Finite Element Modeling Simulation and Analysis. Asian Journal of Scientific Research,
2013; 6(4): pp. 691-702.]. An example of such methods can be seen with the the electric chrage density generated by a piezoelectric cantilever beam. Its mechanism is described by the following piezoelectric constitutive equation [Gong LJ, Shen X, Li JQ. Experimental investigation of energy harvesting from triple-layer piezoelectric
bender. In 18th IEEE International Symposium on the Applications of Ferroelectrics 2009 Aug 23 (pp. 1-6).IEEE.] :

$$
D_3=d_{31} T_1+\varepsilon_{33}^T E_3
$$

where **$D_3$** is the electrical charge density, **$d_{31}$** is the piezoelectric strain constant, **$T_1$** is the stress generated in the length direction of the piezoelectric layer, **$\varepsilon_{33}$** is the dielectric constant of the piezoelectric material under constant stress conditions, and **$E_3$** is the electric field developed in the “3” direction. From the above equation, it is clear that the charge density is proportional to the developed stress. 

## Finite Element Method Overview in FEA Workflow
FEA software packages are computer programs that use FEM to analyze how a material or design responds to certain forces, such as vibration, heat, fluid flow, electromagnetic forces, and other physical effects [https://www.autodesk.com/solutions/simulation/finite-element-analysis]. Some common FEA software packages include ANSYS [https://www.ansys.com/], Abaqus FEA [https://www.3ds.com/products-services/simulia/], COMSOL Multiphysics [https://www.comsol.com/], and SIMSCALE [https://www.simscale.com/]. From a mathmatical point of view, the below five steps overview how the FEM works within these software tools [https://www.jousefmurad.com/fem/the-finite-element-method-beginners-guide/#feaworkflowFFA]:
1) Model preparation
2) Element formulation
3) Assembly
4) Solving systems of linear equations
5) Post-processing

### 1. Model preparation
Problem formulation consists of creating the geometry of your structure, defining material properties, creating initial and boundary conditions, defining other conditions such as contact behaviour, and discretisation of the geometry of your structure. It is important to note that discretization of the geometry is done behind the scenes. 

To achieve discretization of the sturcture, its PDE needs to be represented in its integral form, also known as its weak form [https://www.simscale.com/blog/what-is-finite-element-method/, https://www.comsol.com/multiphysics/finite-element-method?parent=physics-pdes-numerical-042-62]. Once the weak form has been set up, it can be discretized. In simplist terms, numerical integration schemes are used on the integrations in the weak form to divide its integration domain over the structure into non-overlapping elements with primitive shapes. Here's a simplified example of the process: the integral functions from the weak form can be computed approximately as a sum of integrals over all the elements:

$$
\mathrm{I}(\varrho)=\int_{\Omega} \varrho(\mathbf{x}) \mathrm{d} \Omega \approx \int_{\Omega^{\mathrm{h}}} \varrho(\mathbf{x}) \mathrm{d} \Omega=\sum_{\mathrm{e}=0}^{\mathrm{N}-1} \int_{\square^{\mathrm{e}}} \varrho(\mathbf{x}) \mathrm{d} \Omega=\sum_{\mathrm{e}=0}^{\mathrm{N}-1} \mathrm{I}_{\square^e}
$$

where **$\varrho(\mathbf{x})$** is the function to be integrated, **$\Omega$** is the integration domain, **$\Omega^{\mathrm{h}}$** is the approximated discretization of **$\Omega$**, and **$\mathrm{I}_{\square^e}$** is the integral over the element **$\square^e$**. The integrals over the elements can be achieved by first performing a transformation of coordinates. By changing the coordinates of each element from the global coordinate system **$\lbrace \mathbf{x}^0, \mathbf{x}^1, \mathbf{x}^2 \rbrace$** to that of a local coordinate system **$\lbrace \xi^0, \xi^1, \xi^2 \rbrace$**, the integration can be done on a reference element which will be the same for all physical elements. For example, the following is the integral of a triangle element after a change of coordinates via the Jacobian method:

$$
\mathrm{I}_ {\square} = \int_{\square} \varrho(\mathbf{x}) \mathrm{dx}=\int_0^1 \int_0^{1-\xi^0} \varrho(\mathbf{x}(\xi))\|\mathbf{J}\| \mathrm{d} \xi^0 \mathrm{~d} \xi^1
$$


### 2. Element formulation
Element formulation is the integral over a single element. The first steps to achieve this is to change coordinates for earch element from the original global coordinate system to a local coordinate system using the Jacobian of transformation of cordinates. For example, the following is the integral of a triangle element after a change of coordinates via the Jacobian method:


### 3. Assembly
Assembly is obtaining equations for the entire system from the equations for one element.

### 4. Solving systems of linear equations
The system of linear equations are solved via direct or iterative numerical methods. 

### 5. Post-processing
Quantities of interst are determined and visualizations of their response are obtained. 

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

-->
