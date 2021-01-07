% MAIN.m -- Simple Car Parameter Optimization
%
% This script runs a trajectory optimization to find the minimus time to
% accelerate from a standstill and stop  at 400 meters
%
%
% NOTES:
%   This
%

clc; clear;
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Problem Bounds                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
mass = 1000;

mLow = 600; %(kg)
mUpp = 2000; %(kg)

m0 = 1000; %Starting mass parameter
d0 = 0; %Starting position
v0 = 0; %Starting stationary

dF = 400; % reach 400m
vF = 0; % stop there

dLow = 0;
dUpp = inf;

vLow = 0;
vUpp = inf;

uLow = -25000; %Newtons of force for braking
uUpp = 15000; %Newtons of force for acceleration

P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;

P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 1000;

P.bounds.state.low = [dLow; vLow];
P.bounds.state.upp = [dUpp; vUpp];

P.bounds.initialState.low = [d0;v0];
P.bounds.initialState.upp = [d0;v0];

P.bounds.finalState.low = [dF;vF];
P.bounds.finalState.upp = [dF;vF];

P.bounds.control.low = uLow;
P.bounds.control.upp = uUpp;

P.bounds.parameter.low = mLow;
P.bounds.parameter.upp = mUpp;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

P.guess.time = [0, 6, 12]; %(s)
P.guess.state = [ [d0;v0], [d0;v0], [dF;vF] ];
P.guess.control = [uUpp, uUpp, uLow];
P.guess.parameter = [1000];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Dynamics function:
P.func.dynamics = @(t,x,u)([x(2,:);u/mass]); %simple acceleration due to force u

% Objective function:
P.func.bndObj = @(t0,x0,tF,xF)( tF ); %minimize time

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Options and Method selection                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

method = 'trapezoid';

P.options(1).method = 'trapezoid';
P.options(1).defaultAccuracy = 'low';

P.options(2).method = 'trapezoid';
P.options(2).defaultAccuracy = 'medium';
P.options(2).nlpOpt.MaxFunEvals = 5e4;
P.options(2).nlpOpt.MaxIter = 1e5;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Solve!                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(P);

t = linspace(soln(end).grid.time(1),soln(end).grid.time(end),250);
x = soln(end).interp.state(t);
u = soln(end).interp.control(t);



figure;
subplot(3,1,1);
plot(t, x(1,:));
xlabel('time (s)');
ylabel('distance (m)');
title('Fastest Time to drive 400m');

subplot(3,1,2);
plot(t, x(2,:));
xlabel('time (s)');
ylabel('velocity (m/s)');
title('Fastest Time to drive 400m');

subplot(3,1,3);
plot(t, u/1000);
xlabel('time (s)');
ylabel('acceleration forve (kN)');
title('Fastest Time to drive 400m');