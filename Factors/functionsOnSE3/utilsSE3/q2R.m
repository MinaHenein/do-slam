function A= q2R(q)
% Q_TO_DCM converts a quaternion [w x y z ]' to a DCM and Q
    I = eye(3);
    q_ = q(2:4);
    w = q(1);
    Q = skew_symmetric(q_);
%     Q = [0 -q_(3) q_(2);
%          q_(3) 0 -q_(1);
%          -q_(2) q_(1) 0];
    A = (w^2 - q_'*q_) * I + 2 * (q_ * q_') + 2 * w * Q; 
end