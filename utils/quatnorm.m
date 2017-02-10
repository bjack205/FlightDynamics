function [q,n] = quatnorm(q)
%QUATNORM Converts a quaternion to a unit quaternion
%   quaternions of the form [r, ai, bj, ck]
%
%   INPUTS:
%       q must an MxN 2D numeric array. Calculates the norm across each row
%       of an array and divides each element of the row by its norm.
%
%   OUTPUTS:
%       q is a MxN array with normalized rows
%       n is an Mx1 array of the norms of each row
%       
% Brian Jackson July 2016
% Brigham Young University
n = sqrt(sum(q.*q,2));
q = q ./ repmat(n,1,size(q,2));