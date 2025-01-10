
from scipy.optimize import minimize_scalar

def f(x, y) :
    return (x**2) + (y**2) + 4


res = minimize_scalar(f)

print(res.fun)
print(res.x)