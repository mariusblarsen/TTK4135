function w = computeLagrangeMultipliers(nAllC, A, g)
    w = linsolve(A,g)