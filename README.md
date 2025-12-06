# JAGUAR

## Abstract 

The control of both cooperative and non-cooperative multi-agent systems have applications across many different domains and can be modeled using differential games. However, computational cost, primarily due to the curse of dimensionality, has typically prohibited quickly solving these problems on moderate hardware. In this work, we propose a novel method for solving difficult, non-convex differential games through a hybridization of kinodynamic motion planning and solving iterative linear-quadratic game approximations. Game-informed motion planning is used to rapidly determine feasible game solution trajectories while iterative linear-quadratic game approximations are then used for further refinement and multi-agent coupling. We name this algorithm framework JAGUAR: Joint Agent Game-theoretic Update with Augmented Rapid-planning. For numerical examples, we study a problem between an evader and pursuer inspired by the homicidal chauffeur problem in the presence of convex polygon obstacles. 


## Installation and Demos

If pulling from git:

```
git clone https://github.com/BennetOutland/JAGUAR.git

cd JAGUAR

julia

using Pkg

Pkg.activate("venv/")

Pkg.resolve()

Pkg.instantiate()
```

You may need to "install" the DynamicPlanning.jl package in JAGUAR/ via:

```
git clone https://github.com/JuliaDifferentialGames/DynamicPlanning.jl.git
```

If using zip file, should be all good.


See ```\examples``` for the code used to make the figures.


## License

MIT License - see LICENSE file for details.

## Acknowledgments

This package follows the design principles of the [SciML](https://sciml.ai/) ecosystem and draws inspiration from:
- MotionPlanning.jl
- DifferentialEquations.jl
- Optimization.jl

## Disclosure of Generative AI Usage

Generative AI, Claude, was used in the creation of this library as a programming aid including guided code generation, assistance with performance optimization, and for assistance in writing documentation. All code and documentation included in this repository, whether written by the author(s) or generative AI, has been reviewed by the author(s) for accuracy and has completed a verification and validation process upon release.
