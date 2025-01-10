from scipy.optimize import minimize, rosen, rosen_der

x0 = [1.3, 0.7, 0.8, 1.9, 1.2]

def rosen(x):
    return sum(100.0*(x[1:] - x[:-1]**2.0)**2.0 + (1 - x[:-1])**2.0)

res = minimize(rosen, x0, method='Nelder-Mead', tol=1e-6)

print(res.x)