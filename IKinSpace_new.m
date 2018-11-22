function [thetalist_act, cycle_hist, Jac_hist, dJac_hist, p_act, success] = IKinSpace_new(Tsd, thetalist_init, t, algorithm)

thetalist_act = zeros(length(thetalist_init),length(t));
Jac_hist      = zeros(6,6,length(t));
p_act         = zeros(3,length(t));
success       = zeros(length(t),1);
cycle_hist    = zeros(length(t),1);
dJac_hist     = zeros(length(t),1);
%
maxiterations = 50;
eomg          = 1e-3; % rad/s
ev            = 1e-5; % m/s
thetalist    = thetalist_init;
% initial guess is equal to very first desire Tsd

if algorithm == 'Numerical Inverse Kinematics'
    
    [Slist,M,tlist,Jf] = construct(thetalist);
    Tsb             = FKinSpace_new(M,Slist,tlist);
    J               = JacobianSpace_new(Slist,tlist,Jf);
    for i =1:1:length(t)
        Vb  = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd(:,:,i)));
        Vb(1)=0;
        Vs  = Adjoint(Tsb) * Vb;
%         Vs(4)  = 0 ; 
        err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
        cycle = 0;
        
        while err && cycle < maxiterations
            J           = JacobianSpace_new(Slist, tlist,Jf);
            J_inv       = pinv(J);
% %               J_inv (4,:)=J_inv (4,:)/10;
%             J_w         = diag([1 1 1 0.1 1 1]);
            J_w         = eye(6);
            thetalist_delta     = J_w*J_inv * Vs;
         
%              thetalist_delta(4)  = 0;
%              thetalist_delta(5)  = 0;
%              thetalist_delta(2)  = 0;
            thetalist   = thetalist + thetalist_delta;
            
            [Slist,M,tlist,Jf] = construct(thetalist);
            Tsb         = FKinSpace_new(M, Slist, tlist);
            Vb  = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd(:,:,i)));
%             Vs          = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd(:,:,i)));
            Vs          = Adjoint(Tsb) * Vb;

            err         = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
            cycle       = cycle + 1;
        end
        Vs 
        %         [thetalist, success] = IKinSpace(Slist, M, Tsd(:,:,i), thetalist0, eomg, ev)
        
        thetalist_act(:,i)  = thetalist
        cycle_hist(i)       = cycle;
        Jac_hist(:,:,i)     = J;
        dJac_hist(i)        = det(J);
        p_act(:,i)          = Tsb(1:3,4);
        success(i)          = ~ err;

    end
    
else
end

end