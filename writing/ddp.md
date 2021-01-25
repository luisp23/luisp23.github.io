---
layout: default
---

### Differential Dynamic Programming and Model Predictive Control

Differential Dynamic Programming is an optimal control algorithm that can be used to solve trajectory optimization problems of dynamic systems. In this post I want to share a mathematical first principles derivation for this algorithm and a MATLAB simulated implementation on an inverted pendulum and a cart-pole system. After this, we implement this algorithm in a Receding Horizon/Model Predicive Control fashion. This post is based on my assignments/notes completed for my [Robotics and Autonomy](http://www.ae.gatech.edu/sites/default/files/images/ae-4803-roboticsandautonomy_0.pdf?_ga=2.58749550.297452889.1611523712-996569902.1606676489) course taught by Dr. Evangelos Theodorou.

(work in progress)

---
#### Linearization and time discretization

First we respresent the dynamics of a system with a set of differential equations denoted by $ \frac{d\vec{x}}{dt} = f(\vec{x}, \vec{u}, t) $ where $\vec{x}$ and $\vec{u}$ are state-control pairs of a trajectory under consideration. For certain (more interesting) systems, these dynamics can be nonlinear. DDP requires us to linearize these dynamics and we do so in the following:

$$  
    \begin{align*}
        & \frac{d\vec{x}}{dt} = f(\vec{x}, \vec{u}, t) \\
        & = f(\vec{x} + (\bar{\vec{x}} - \bar{\vec{x}}), \vec{u} + (\bar{\vec{u}} - \bar{\vec{u}}), t) \\
        & = f(\bar{\vec{x}} + (\vec{x} - \bar{\vec{x}}), \bar{\vec{u}} + (\vec{u} - \bar{\vec{u}}), t) \\
        & = f(\bar{\vec{x}} + \delta \vec{x}, \bar{\vec{u}} + \delta \vec{u}, t) \\
        & = f(\bar{\vec{x}}, \bar{\vec{u}}, t) + \nabla_{\vec{x}}f\delta\vec{x} +  \nabla_{\vec{u}}f\delta\vec{u} \\
        & \frac{d\vec{x}}{dt} - f(\bar{\vec{x}}, \bar{\vec{u}}, t) = \nabla_{\vec{x}}f\delta\vec{x} +  \nabla_{\vec{u}}f\delta\vec{u} \\
        & \\
        & \boxed{\frac{d\delta\vec{x}}{dt} = \nabla_{\vec{x}}f\delta\vec{x} +  \nabla_{\vec{u}}f\delta\vec{u}}
    \end{align*}    

$$

Our next step is to time discretize the linearized dynamics: 

$$
    \begin{align*}
        & \frac{\delta \vec{x}(t_{k+1}) - \delta\vec{x}(t_{k})}{t_{k+1} - t_{k}} = \nabla_{\vec{x}}f\delta\vec{x}(t_{k}) +  \nabla_{\vec{u}}f\delta\vec{u}(t_{k}) \\
        & \frac{\delta \vec{x}(t_{k+1}) - \delta\vec{x}(t_{k})}{dt} = \nabla_{\vec{x}}f\delta\vec{x}(t_{k}) +  \nabla_{\vec{u}}f\delta\vec{u}(t_{k}) \\
        & \delta \vec{x}(t_{k+1}) - \delta\vec{x}(t_{k}) = \nabla_{\vec{x}}fdt\delta\vec{x}(t_{k}) +  \nabla_{\vec{u}}fdt\delta\vec{u}(t_{k}) \\
        & \delta \vec{x}(t_{k+1}) = \nabla_{\vec{x}}fdt\delta\vec{x}(t_{k}) + \delta\vec{x}(t_{k}) +  \nabla_{\vec{u}}fdt\delta\vec{u}(t_{k}) \\
        & \delta \vec{x}(t_{k+1}) =  \left(\vec{I}_{n \times n} + \nabla_{\vec{x}}fdt\right)\delta\vec{x}(t_{k}) + (\nabla_{\vec{u}}fdt)\delta\vec{u}(t_{k}) \\
        & \\
        & \boxed{\delta \vec{x}(t_{k+1}) =  \vec{\Phi}(t_{k})\delta\vec{x}(t_{k}) + \vec{B}(t_{k})\delta\vec{u}(t_{k})} \\
    \end{align*}
$$

Where $\vec{\Phi}(t_{k})$ and $\vec{B}(t_{k})$ represent the Jacobians with respect to the states and controls, respectively: 

$$
    \begin{align*}
        & \vec{\Phi}(t_{k}) = \vec{I}_{n \times n} + \nabla_{\vec{x}}f(t_{k})(t_{k+1} - t_{k}) \\
        & \vec{B}(t_{k}) = \nabla_{\vec{u}}f(t_{k})(t_{k+1} - t_{k})
    \end{align*}
$$

---
#### Bellman Principle and the state-action value function Q

The derivation of DDP stems from the Value function $V(\vec{x}(t),t)$ from Bellman's Principle of optimality using dynamic programming for trajectory optimization. In continous time this is denoted as: 

$$
    \begin{align*}  
        V(\vec{x}(t), t) = \underset{\vec{u}}{\text{min}} ~ \Bigg[ \int_{t}^{t + \delta t} \ell (\vec{x},\vec{u},t)dt + V(x(t+\delta t)) \Bigg]
    \end{align*}
    
$$

As we are working in discrete time, this is denoted as: 

$$
    \begin{align*}  
        V(\vec{x}(t_{k}), t_{k}) = \underset{\vec{u}}{\text{min}} ~ \Bigg[\ell (\vec{x}(t_k),\vec{u}(t_k),t_{k})dt + V(x(t_{k+1})) \Bigg]
    \end{align*}  
$$

In the notation of this derivation we consider the state-action value function $Q$ defined as

$$
    \begin{align*}
        & \boxed{Q(\vec{x}(t_{k}), \vec{u}(t_{k})) = \mathcal{L}(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k}) + V(\vec{x}(t_{k+1}), t_{k+1})}
    \end{align*}
$$

where the running cost is denoted by.

$$
    \begin{align*}
        & \mathcal{L}(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k}) = \ell (\vec{x}(t_k),\vec{u}(t_k),t_{k})dt 
    \end{align*}
$$ 

Therefore our optimization problem under consideration is: 

$$
    \begin{align*}
        & \boxed{\underset{\vec{u}}{\text{min}} ~ \Bigg[ Q(\vec{x}(t_{k}), \vec{u}(t_{k})) \Bigg]}
    \end{align*}
$$ 

---
#### Quadratic expansions

In this algorithm we consider quadratic models of our cost under minimization, allowing our algorithm to achieve quadratic convergence. To do this we must consider a quadratic approximation of our running cost and our value function by using the Second-Order Taylor Series expansion. We recall this general formula for a multivaribale function along the (x,y = a,b): 

$$
    \begin{align*}
        & f(x,y) \approx f(a,b) + f_{x}(a,b)(x-a) + f_{y}(a,b)(y-b) + \frac{1}{2}f_{xx}(a,b)(x-a)^2 + f_{xy}(a,b)(x-a)(y-b) + \frac{1}{2}f_{yy}(a,b)(y-b)^2 \\
    \end{align*}
$$

#### Quadratic expansion of the running cost

First we do the second order expansion of the runnning cost along the nominal trajectory $\bar{\vec{x}}$ and $\bar{\vec{u}}$: 

$$
    \begin{align*}
        & \mathcal{L}(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k}) \\
        & = \mathcal{L}(\vec{x}(t_{k}) + \left(\bar{\vec{x}}(t_{k}) - \bar{\vec{x}}(t_{k})\right), \vec{u}(t_{k}) + \left(\bar{\vec{u}}(t_{k}) - \bar{\vec{u}}(t_{k})\right), t_{k}) \\
        & = \mathcal{L}( \bar{\vec{x}}(t_{k}) + \left(\vec{x}(t_{k}) - \bar{\vec{x}}(t_{k})\right), \bar{\vec{u}}(t_{k}) + \left(\vec{u}(t_{k}) - \bar{\vec{u}}(t_{k})\right), t_{k}) \\
        & = \mathcal{L}(\bar{\vec{x}}(t_{k}) + \delta \vec{x}(t_{k}), \bar{\vec{u}}(t_{k}) + \delta \vec{u}(t_{k}), t_{k}) \\
        & = \underbrace{\ell(\vec{\bar{x}}(t_{k}), \vec{\bar{u}}(t_{k}), t_{k})dt}_{\mathcal{L}} + \underbrace{\Big(\nabla_{\vec{x}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)^{T}}_{\mathcal{L}_{\vec{x}}}\delta\vec{x}(t_{k}) + \underbrace{\Big(\nabla_{\vec{u}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)^{T}}_{\mathcal{L}_{\vec{u}}}\delta\vec{u}(t_{k}) \\
        & + \frac{1}{2}\delta\vec{x}(t_{k})^{T}\underbrace{\Big(\nabla_{\vec{xx}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)}_{\mathcal{L}_{\vec{xx}}}\delta\vec{x}(t_{k}) + \frac{1}{2}\delta\vec{u}(t_{k})^{T}\underbrace{\Big(\nabla_{\vec{uu}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)}_{\mathcal{L}_{\vec{uu}}}\delta\vec{u}(t_{k}) \\
        & + \frac{1}{2}\delta\vec{u}(t_{k})^{T}\underbrace{\Big(\nabla_{\vec{xu}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)}_{\mathcal{L}_{\vec{xu}}}\delta\vec{x}(t_{k}) + \frac{1}{2}\delta\vec{x}(t_{k})^{T}\underbrace{\Big(\nabla_{\vec{ux}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt\Big)}_{\mathcal{L}_{\vec{ux}}}\delta\vec{u}(t_{k})\\
    \end{align*}
$$

We then denote: 

$$
    \begin{align*}
        & \mathcal{L} = \ell(\vec{\bar{x}}(t_{k}), \vec{\bar{u}}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{x}} = \nabla_{\vec{x}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{u}} = \nabla_{\vec{u}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{xx}} = \nabla_{\vec{xx}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{uu}} = \nabla_{\vec{uu}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{xu}} = \nabla_{\vec{xu}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
        & \mathcal{L}_{\vec{ux}} = \nabla_{\vec{ux}}\ell(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k})dt \\
    \end{align*}
$$

And rewrite our second order expansion of the runnning cost to be: 

$$
    \boxed{\begin{align*}
        & \mathcal{L}(\vec{x}(t_{k}), \vec{u}(t_{k}), t_{k}) = \mathcal{L} + \mathcal{L}_{\vec{x}}^{T}\delta\vec{x}(t_{k}) + \mathcal{L}_{\vec{u}}^{T}\delta\vec{u}(t_{k}) + \frac{1}{2}\delta\vec{x}(t_{k})^{T}\mathcal{L}_{\vec{xx}}\delta\vec{x}(t_{k}) + \frac{1}{2}\delta\vec{u}(t_{k})^{T}\mathcal{L}_{\vec{uu}}\delta\vec{u}(t_{k}) + \frac{1}{2}\delta\vec{u}(t_{k})^{T}\mathcal{L}_{\vec{xu}}\delta\vec{x}(t_{k}) + \frac{1}{2}\delta\vec{x}(t_{k})^{T}\mathcal{L}_{\vec{ux}}\delta\vec{u}(t_{k}) \\
    \end{align*}}
$$ 

#### Quadratic expansion of the value function

#### Quadratic expansion of the state-action value function


---
#### Control optimization


#### Computing the optimal control corrections

#### Computing the optimal control open-loop feedforward and closed-loop feedback gains




---
#### Backward pass value functions 

#### Forward pass 


---
#### Implementation on an inverted pendulum

#### Implementation on a cart-pole system 

#### Robustness against stochastic forces



---
#### Formulation as a model predictive control problem


---
#### Credits

1) AE 4803 Robotics and Autonomy, Dr. Evangelos Theodorou, Georgia Institute of Technlogy Fall 2020 

2) [Differential Dynamic Programming, Wikipedia](https://en.wikipedia.org/wiki/Differential_dynamic_programming)