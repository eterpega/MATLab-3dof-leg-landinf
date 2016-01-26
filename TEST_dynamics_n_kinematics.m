%INVPEND_DYNAMICS wrapper to numerically compute 3-link invertered Pendulum's EOMs
% FILENAME: invPend_Dynamics.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 
%
% DESCRIPTION:
%
%% ----------------------------------------------------------
%   INITIALIZE WORKSPACE
% -----------------------------------------------------------
clc; clear; close all;

addpath('wrapperFncs/');
addpath('autogenFncs/');
%% ----------------------------------------------------------
%   DEFINE ODE PROBLEM
% -----------------------------------------------------------
% Initialize model
params = params_3link_invPend;

% Integrator 
t0 = 0;
tF = 2;
tspan = [t0,tF];

% Initial Condition
q0 = [-2; -2; -1.5; 0; 0;];
dq0 = zeros(5,1);
z0 = [q0;dq0]';
u = [];
%% ----------------------------------------------------------
%   SOLVE
% -----------------------------------------------------------

[Tsol,Xsol] = ode45(@(t,z)invPend_DynamicsWrapper(t,z,u,params),tspan,z0);
Xsol = Xsol';

%% ----------------------------------------------------------
%   DISPLAY RESULTS
% -----------------------------------------------------------

% Plots
figure
subplot(2,2,1)
plot(Tsol,Xsol(1:3,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle [rad]');

subplot(2,2,2)
plot(Xsol(4,:),Xsol(5,:))
xlabel('X Pos [m]')
ylabel('Y Pos [m]')

subplot(2,2,3)
plot(Tsol,Xsol(6:8,:));
legend('joint 1','joint 2','joint 3')
xlabel('Time [sec]');
ylabel('Angle rate [rad/sec]');

subplot(2,2,4)
plot(Xsol(9,:),Xsol(10,:))
xlabel('X Vel  [m/s]')
ylabel('Y Vel [m/s]')

%%%%TODO
% Animate the results:
A.plotFunc = @(t,z)( drawInvPend3(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
animate(Tsol,Xsol,A)

