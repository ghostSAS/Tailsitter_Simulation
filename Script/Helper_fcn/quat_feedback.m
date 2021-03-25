function axis_err = quat_feedback(quat,q_ref)

q_err = quatmultiply(q_ref,quatconj(quat));

axis_err = -q_err(2:4)*sign(q_err(1));
end
