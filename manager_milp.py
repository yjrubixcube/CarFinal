import gurobipy as gp
from gurobipy import GRB

def find_trajectory_conflict(L, N, target_l):
    '''
    Return a list of vehicle pairs ((l, i), (lp, ip)) 
    that have no trajectory conflict.
    '''

    '''
    conflict zone:

            |       |
            | lane1 |
    --------         --------
    lane2    'B' 'A'  lane0
             'C' 'D'
    --------         --------
            | lane3 |
            |       |
    '''

    trajectory = {}
    for l in range(L):
        for i in range(N[l]):
            # entering lane: 0
            if l == 0:
                if target_l[l][i] == 1:
                    trajectory[(l, i)] = {'A'}

                elif target_l[l][i] == 2:
                    trajectory[(l, i)] = {'A', 'B'}

                elif target_l[l][i] == 3:
                    trajectory[(l, i)] = {'A', 'B', 'C'}

            # entering lane: 1
            elif l == 1:
                if target_l[l][i] == 0:
                    trajectory[(l, i)] = {'B', 'C', 'D'}

                elif target_l[l][i] == 2:
                    trajectory[(l, i)] = {'B'}

                elif target_l[l][i] == 3:
                    trajectory[(l, i)] = {'B', 'C'}

            # entering lane: 2
            elif l == 2:
                if target_l[l][i] == 0:
                    trajectory[(l, i)] = {'C', 'D'}

                elif target_l[l][i] == 1:
                    trajectory[(l, i)] = {'A', 'C', 'D'}

                elif target_l[l][i] == 3:
                    trajectory[(l, i)] = {'C'}

            # entering lane: 3
            elif l == 3:
                if target_l[l][i] == 0:
                    trajectory[(l, i)] = {'D'}

                elif target_l[l][i] == 1:
                    trajectory[(l, i)] = {'A', 'D'}

                elif target_l[l][i] == 2:
                    trajectory[(l, i)] = {'A', 'B', 'D'}


    # Gamma: a list of vehicle pairs ((l, i), (lp, ip)) that have no trajectory conflict
    Gamma = []
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                for ip in range(N[lp]):
                    if not trajectory[(l, i)] & trajectory[(lp, ip)]:
                        Gamma.append(((l, i), (lp, ip)))
    
    return Gamma


def solve(A, H, target_l, G, Gp):
    '''
    Return the entering time of each vehicle
    
    ------------ problem parameters ------------
    (v[l][i]: vehicle i on lane l)
    A[l][i]: estimated arrival time of v[l][i]
    H[l][i]: whether v[l][i] is HV
    target_l[l][i]: target lane of v[l][i]
    G: time gap for AV
    Gp: time gap for HV
    --------------------------------------------
    '''

    # N[l]: number of vehicles on lane l
    N = []
    for i in range(len(A)):
        N.append(len(A[i]))

    # L: number of lane
    L = len(A) 

    # determine trajectory conflict
    Gamma = find_trajectory_conflict(L, N, target_l)

    # create model
    m = gp.Model("milp")
    
    # disable solver log output
    m.Params.OutputFlag = 0

    # --------------------  create variables  -------------------- #
    # t[l, i]: entering time of v[l, i]
    t_indices = []
    for l in range(L):
        for i in range(N[l]):
            t_indices.append((l, i))

    t = m.addVars(t_indices, name="t")

    # h[l, i, l', i']: whether v[l', i'] is at the head on its lane
    #                  when v[l, i] is passing through the intersection
    # -- 
    # h[l, i, l', Nl']: whether all vehicles on l' have passed through the intersection
    #                   when v[l, i] is passing through the intersection
    h_indices = []
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                for ip in range(N[lp]+1):
                    h_indices.append((l, i, lp, ip))

    h = m.addVars(h_indices, vtype=GRB.BINARY, name="h")


    # o[l, i, l', i']: whether v[l, i] passes before v[l', i']
    o_indices = h_indices
    o = m.addVars(o_indices, vtype=GRB.BINARY, name="o")

    # r[l, i]: whether there exists an HV (including v[l, i] itself) 
    #          at the head on at least one lane when v[l, i] is passing through 
    #          the intersection
    r_indices = t_indices
    r = m.addVars(r_indices, vtype=GRB.BINARY, name="r")

    # z[l, i]: whether v[l, i] is the last passing vehicle
    z_indices = t_indices
    z = m.addVars(z_indices, vtype=GRB.BINARY, name="z")

    # Z: max(t[l, i])
    Z = m.addVar(name="Z")


    # --------------------   add contraints   -------------------- #
    # M: a sufficient large number
    M = 10000
            
    # 2.2
    for l in range(L):
        for i in range(N[l]):
            for ip in range(i):
                m.addConstr(t[l, i] >= t[l, ip])

    # 3.5
    for l in range(L):
        for i in range(N[l]):
            m.addConstr(h[l, i, l, i] == 1)

    # 3.6
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                if lp == l: continue
                
                expr = gp.LinExpr(0)
                for ip in range(N[lp]+1):
                    expr.addTerms(1, h[l, i, lp, ip])
                
                m.addConstr(expr == 1)

    # 3.7
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                if lp == l: continue
                
                expr1 = gp.LinExpr(1)
                expr2 = gp.LinExpr(0)
                for ip in range(N[lp]):
                    expr1.addTerms(1, o[lp, ip, l, i])

                for ip in range(N[lp]+1):
                    expr2.addTerms(ip+1, h[l, i, lp, ip])

                m.addConstr(expr1 == expr2)

    # 3.8
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                for ip in range(N[lp]):
                    if (lp, ip) == (l, i): continue
                    m.addConstr(o[l, i, lp, ip] + o[lp, ip, l, i] == 1)

                    for lpp in range(L):
                        for ipp in range(N[lpp]):
                            if (lpp, ipp) == (lp, ip) or (lpp, ipp) == (l, i): continue
                            m.addConstr(o[l, i, lp, ip] + o[lp, ip, lpp, ipp] - 1 <= o[l, i, lpp, ipp])

    # 3.9 (ip range??)
    for l in range(L):
        for i in range(N[l]):
            expr = gp.LinExpr(0)
            for lp in range(L):
                for ip in range(N[lp]):
                    expr.addTerms(H[lp][ip], h[l, i, lp, ip])

            m.addConstr(expr <= M*r[l, i])

    # 3.10
    expr = gp.LinExpr(0)
    for l in range(L):
        for i in range(N[l]):
            expr.addTerms(1, z[l, i])

    m.addConstr(expr == 1)

    ##### add by myself
    expr = gp.LinExpr(0)
    for l in range(L):
        if N[l] > 0:
            expr.addTerms(1, z[l, N[l]-1])

    m.addConstr(expr == 1)
    ##### add by myself

    for l in range(L):
        for i in range(N[l]):
            expr = gp.LinExpr(M*z[l, i]-M)
            for lp in range(L):
                for ip in range(N[lp]):
                    expr.addTerms(1, h[l, i, lp, ip])

            m.addConstr(expr <= 1)

    # 3.12
    for l in range(L):
        for i in range(N[l]):
            for ip in range(i):
                m.addConstr(o[l, ip, l, i] == 1)
            
            m.addConstr(o[l, i, l, i] == 0)

    # 3.13
    for l in range(L):
        for i in range(N[l]):
            m.addConstr(t[l, i] >= A[l][i])

    # 3.14, 3.15
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                for ip in range(N[lp]):
                    if ((l, i), (lp, ip)) in Gamma:
                        m.addConstr(t[lp, ip] - t[l, i] + M*(1-o[l, i, lp, ip]+z[l, i]) >= 0)
                    
                    else:
                        m.addConstr(t[lp, ip] - t[l, i] + M*(1-o[l, i, lp, ip]+z[l, i]) >= G)

    # 3.16
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                for ip in range(N[lp]):
                    if ((l, i), (lp, ip)) not in Gamma:
                        m.addConstr(t[lp, ip] - t[l, i] + M*(1-o[l, i, lp, ip]+z[l, i]) >= Gp + M*(r[l, i]-1))

    # 3.17
    for l in range(L):
        for i in range(N[l]):
            for lp in range(L):
                if lp == l: continue
                for ip in range(N[lp]):
                    m.addConstr(M*H[lp][ip]*h[l, i, lp, ip] + A[l][i]-A[lp][ip] <= M)


    # objective
    m.addConstr(Z == gp.max_(t))
    m.setObjective(Z, GRB.MINIMIZE)

    # solve
    m.optimize()

    '''
    # show result
    for v in m.getVars():
        print(f"{v.VarName} {v.X:g}")

    print(f"Obj: {m.ObjVal:g}")
    '''

    t_result = []
    for l in range(L):
        t_result.append([ _ for _ in range(N[l])])
        for i in range(N[l]):
            t_result[l][i] = t[l, i].X

    return t_result


def fast_solve(A, H, target_l, group_size, G, Gp):
    # group_size = 10
    
    L = len(A)
    N = []
    t_result = []
    for l in range(L):
        N.append(len(A[l])) 
        t_result.append([0 for _ in range(N[l])])

    # directly return if total number of vehicles is small
    if sum(N) <= group_size:
        return solve(A, H, target_l, G, Gp)

    # collect vehicle indices (with increasing estimated time)
    indices = [((l, i), A[l][i]) for l in range(L) for i in range(N[l])]
    indices.sort(key=lambda x: x[1])
    

    # find minimum estimated time
    t_min_temp = []
    for l in range(L):
        if len(A[l]) > 0:
            t_min_temp.append(A[l][0])
    
    t_start = min(t_min_temp)

    # solve milp group by group
    v_count = 0
    while v_count < sum(N):
        A_p = [[] for _ in range(L)]
        H_p = [[] for _ in range(L)]
        target_l_p = [[] for _ in range(L)]
        id_mapping = {} # id_mapping[new_id] = old_id

        # the last group
        if v_count + group_size > sum(N):
            group_size = sum(N) - v_count

        for id in range(v_count, v_count+group_size):
            l, i = indices[id][0]
            A_p[l].append(A[l][i])
            H_p[l].append(H[l][i])
            target_l_p[l].append(target_l[l][i])

            id_mapping[(l, len(A_p[l])-1)] = (l, i)

        '''
        print(A_p)
        print(H_p)
        print(target_l)
        print(id_mapping)
        '''

        t_group_result =  solve(A_p, H_p, target_l_p, G, Gp)
        t_min_temp = []
        t_max_temp = []
        for l in range(L):
            if len(t_group_result[l]) > 0:
                t_min_temp.append(t_group_result[l][0])
                t_max_temp.append(t_group_result[l][-1])

        t_min = min(t_min_temp)
        t_max = max(t_max_temp)
        for new_l in range(len(t_group_result)):
            for new_i in range(len(t_group_result[new_l])):
                l, i = id_mapping[(new_l, new_i)]
                t_result[l][i] = t_group_result[new_l][new_i] - t_min + t_start

        t_start += t_max - t_min + Gp
        v_count += group_size

    return t_result