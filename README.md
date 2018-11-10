backstep_qc

Backstepping controller for quadcotpers using ROS

Overview

This repository contains a backstepping controller for quadcopters. We propose a procedure based on conjugate gradient optimization for controller tuning when the dynamic model is nonlinear and the test signals are stochastic. The optimization procedure is performed with MATLAB scripts. To validate the findings, a bipartite ROS application was implemented. The first part corresponds to the orientation controller of the drone which runs on the on-board computer. The second part carries out the position controller and runs on a ground station computer. ROS Indigo Igloo is used for the code of this chapter.
