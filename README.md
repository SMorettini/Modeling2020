# Modeling a luna park carousel
This is a univesity project for the course of Modelling and Simulation of Mechatronics Systems of University of Trento. The objective is to model a luna park carousel.

<p align="center">
<img align="center" src="https://github.com/MesSem/Modeling2020/blob/master/carousel.jpeg">
</p>

## Software and external libraries

For the project the following software were used:

* Blender: to made a very simple model as reference for reference frames and lenghts;
* Maple: to model the carousel;
* MapleSim: to model the carousel using a different approach;
* Matlab: to optimize the model and to solve the equation of motion using a different aproach instead of the internal library of Maple. 

In both the code of Maple and Matlab some external function are used from this https://github.com/ebertolazzi/courseModellingAndSimulationOfMechatronicsSystem2020 repository. The toolbox present in the repository need to be installed in Matlab to run some of the scrips.

## Description of the repository content


* [0FinalKinematics.mw](0FinalKinematics.mw): Maple script for modeling the kinematic of the system using a global and a recursive approach;
* [0FinalDynamics.mw ](0FinalDynamics.mw): Maple script for modeling the dynamic of the system using the recursive kinematic model made in the previous file;
* [0Winx_model.msim](0Winx_model.msim): model made by MapleSim of the system;
* [Presentation.pdf](Presentation.pdf): final presentation with explained the approach used and the results;
* [Presentation_with_animation.pptx](Presentation_with_animation.pptx): final presentation with explained the approach used and the results with some animation of the carousel that explain better the results obtained;
* [WeightsEstimate.mw](WeightsEstimate.mw): code to estimate the weights of the different parts of the carouse;
* [SimpleModel.blend](3dModelBlender/SimpleModel.blend): simple blender model to show the name we give to the different lenghts and to the different reference frame;
* [Optimization.m](Optimization/Optimization.m): script for optimizing the variable of the system using a evolutionary algorithm (GA or PSO or DE);
* [Winx.m](Matlab/Winx.m): class representing the ODE of the system;
* [WinxDAE.m](Matlab/WinxDAE.m): class representing the DAE of the system;
* [WinxResult_projection.mlx](Matlab/WinxResult_projection.mlx): Matlab script containing the result of the profile obtained using the ODE plus the projection with static dynamic;
* [WinxResult_projection_profile.mlx](Matlab/WinxResult_projection_profile.mlx): Matlab script containing the result of the profile obtained using the ODE plus the projection with non zero motion profile to the motors;
* [WinxStabilized.mlx](Matlab/WinxStabilized.mlx): Matlab script containing the result of the profile obtained using the DAE plus the Baumgarte stabilization with static dynamic;
* [WinxStabilized_profile.mlx](Matlab/WinxStabilized_profile.mlx): Matlab script containing the result of the profile obtained using the DAE plus the Baumgarte stabilization with non zero motion profile to the motors.

The files not listed above contains tests or old implementation that had some error or not totally working but all is mantained for some of the interesting passages maybe usefull in future. Some othe files are instead in support of the main scripts. 

## Possible improvements

* Add a close loop when solving the dynamics to reduce the error during direct dynamic either in Matlab and in Maple;
* Adjust the MapleSim to give as input the motor profiles;
* Reduce the approximation in weight evaluation;

## Team members
* [Marco Boffo](https://github.com/boffomarco)
* [Giulia Brusadin](https://github.com/GiuliaBrusadin)
* [Maria Camporese](https://github.com/mariacamporese)
* [Simone Morettini](https://github.com/MesSem)

For more information loot at the Presentation file or feel free to ask us questions.
