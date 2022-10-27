#! /usr/bin/env python
# coding: utf-8

## @file
# Contains the CatmullSpline class. This code was adapted from the code i wrote during my bachelor's thesis.

## @author Martin 
## @maintainer Jiangnan

import numpy as np


## Class containing a two-dimensional Catmull-Spline with methods for evaluation. 
# Needs 4 control points for spline calculation.
class CatmullSpline():

    ## Standard CatmullSpline constructor.
    # @param p0 First control point of the spline. Default np.zeros(2). Type np.array(2, dtype='float').
    # @param p1 Second control point of the spline. Default np.zeros(2). Type np.array(2, dtype='float').
    # @param p2 Third control point of the spline. Default np.zeros(2). Type np.array(2, dtype='float').
    # @param p3 Fourth control point of the spline. Default np.zeros(2). Type np.array(2, dtype='float').
    def __init__(self, p0 = np.zeros(2), p1 = np.zeros(2), p2 = np.zeros(2), p3 = np.zeros(2)):
        self._p0 = p0
        self._p1 = p1
        self._p2 = p2
        self._p3 = p3
        self._pos = np.zeros(2)
        self._vel = np.zeros(2)
        self._agl = 0
        self._agl_vel = 0

    ## Sets the control points for the spline.
    # @param p_lst 2D array holding four points. Type np.array((4,2), dtype='float')
    def set_pts(self, p_lst):
        self._p0, self._p1, self._p2, self._p3 = p_lst

    ## Evaluates the spline at a certain parameterization.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the position as np.array(2,dtype='float'), the angle in rad as float, the velocity as np.array(2,dtype='float') and the angular velocity in rad/s as float.
    def get_spl(self, u):
        self._pos = self._calc_spline(u)
        self._vel = self._calc_spline_vel(u)
        self._agl = self._calc_agl(u)
        self._agl_vel = self._calc_agl_vel(u)
        return self._pos, self._agl, self._vel, self._agl_vel

    ## Calculates the Catmull-Spline position.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the position on the spline at u. Type np.array(2,dtype='float').
    def _calc_spline(self, u):
        pos = (0.5) * ((self._p1*2.0) + (-self._p0 + self._p2)*u + ((self._p0*2.0) - (self._p1*5.0) + 
              (self._p2*4.0) - self._p3)*(u**2) + (-self._p0 + (self._p1*3.0) - (self._p2*3.0) + self._p3)*(u**3))
        return pos

    ## Calculate the linear velocity while executing the spline.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the velocity on the spline at u. Type np.array(2,dtype='float').
    def _calc_spline_vel(self, u):
        # Calculate the spline derivative to get the velocity.
        vel = (0.5) * ((-self._p0 + self._p2) + ((self._p0*2.0) - (self._p1*5.0) + 
              (self._p2*4.0) - self._p3)*(2*u) + (-self._p0 + (self._p1*3.0) - (self._p2*3.0) + self._p3)*(3*u**2))
        return vel

    ## Calculate the yaw of the robot on the trajectory.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the angle in rads on the spline at u. Type float.
    def _calc_agl(self, u):
        vel = self._calc_spline_vel(u)
        return np.arctan2(vel[1],vel[0])

    ## Calculate the angular velocity of the spline.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the angular velocity in rads/s on the spline at u. Type float.
    def _calc_agl_vel(self, u):
        vel = self._calc_spline_vel(u)
        acc = self._calc_spline_acc(u)
        if acc.any() == 0 or vel.all() == 0:
            return 0.0
        ang_vel = -(vel[1]) / (np.sum(vel**2)) * acc[0] + (vel[0]) / (np.sum(vel**2)) * acc[1]
        return ang_vel

    ## Calculate the linear acceleration while executing the spline.
    # @param u Parameter for the spline evaluation. Should be in range 0 to 1. Type float.
    # @return Returns the acceleration on the spline at u. Type np.array(2,dtype='float').
    def _calc_spline_acc(self, u):
        # Calculate second derivative of the spline.
        acc = (self._p0*2.0) - (self._p1*5.0) + (self._p2*4.0) - self._p3 + (-self._p0 + (self._p1*3.0) - (self._p2*3.0) + self._p3)*(3*u)
        return acc
    
    ## @var _p0 
    # First control point for the Catmull-Rom spline. Type np.array(2,dtype='float').
    
    ## @var _p1 
    # Second control point for the Catmull-Rom spline. Type np.array(2,dtype='float').
    
    ## @var _p2 
    # Third control point for the Catmull-Rom spline. Type np.array(2,dtype='float').
    
    ## @var _p3 
    # Fourth control point for the Catmull-Rom spline. Type np.array(2,dtype='float').
    
    ## @var _pos 
    # Last position on the Catmull-Rom spline calculated by get_spline. Type np.array(2,dtype='float').
    
    ## @var _vel 
    # Last velocity on the Catmull-Rom spline calculated by get_spline. Type np.array(2,dtype='float').
    
    ## @var _agl 
    # Last yaw angle in radians on the Catmull-Rom spline calculated by get_spline. Type float.
    
    ## @var _agl_vel 
    # Last yaw angular speed in radians/s on the Catmull-Rom spline calculated by get_spline. Type float.
    
