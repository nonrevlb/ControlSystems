

def step_function(t,T,T2,A):
    # t is the sim time.
    # T is the sim time when the step
    # function assumes the value of A
    # A is the amplitude of the step function
    u = 0;
    if t > T:
        if t < T2:
            u = A
    return u
