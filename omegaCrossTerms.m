function w_dot = omegaCrossTerms(w, J)

l1 = J(1,1);
l2 = J(2,2);
l3 = J(3,3);
w32 = w(3,:).*w(2,:);
w31 = w(3,:).*w(1,:);
w12 = w(1,:).*w(2,:);

w_dot = J^-1*[(l3-l2)*w32; (l1-l3)*w31; (l2-l1)*w12];