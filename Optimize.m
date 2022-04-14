function Uopt = Optimize(F, G, Q, R, SP, Inp, del_Y)
    Uopt = zeros(2, 1);
    Y_pred = F * del_Y + (G*Inp);
    E = SP - Y_pred;
    f = -G'*Q*E;
    H = G'*Q*G + R;
    func = @(U) (E'*Q*E) + U'*H*U + 2*f'*U;

    Acon = [];
    bcon = [];

    options = optimoptions('fmincon','Display', 'off',...
        'Algorithm','sqp', 'MaxIterations', 10);
%     lb =  zeros(2,1) - Inf;
%     ub =  zeros(2,1) + Inf;
    lb = [-0.523599; -0.523599];
    ub = [0.523599; 0.523599];
    Uopt = fmincon(func, Inp, Acon, bcon, Acon, bcon, lb, ub, [], options);
end