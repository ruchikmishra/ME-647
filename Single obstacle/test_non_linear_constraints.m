function[c,ceq]= test_non_linear_constraints(u,v_robot,v_obstacle,robot_abs_pos,obstacle_abs_position,r_robot,r_obstacle)
c= (dot(transpose(obstacle_abs_position-robot_abs_pos),(v_obstacle-(v_robot+u)))/norm(v_obstacle-(v_robot+u)))^2 + (r_robot+r_obstacle)^2 - (norm(obstacle_abs_position-robot_abs_pos))^2;
ceq=[];
end