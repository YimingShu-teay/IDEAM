import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import numpy as np


def find_nearest_intersection(a, b, center, external_point):
    h, k = center
    x1, y1 = external_point

    if x1 - h == 0:  
        nearest_x = x1
        if y1 > k:  
            nearest_y = k + b
        else:  
            nearest_y = k - b
        return nearest_x, nearest_y

    m = (y1 - k) / (x1 - h)
    c = y1 - m * x1

    # (x - h)^2/a^2 + (m*x + c - k)^2/b^2 = 1
    coeff_x2 = (1/a**2 + m**2/b**2)
    coeff_x = (2*m*(c - k)/b**2 - 2*h/a**2)
    coeff_const = (h**2/a**2 + (c - k)**2/b**2 - 1)

    delta = coeff_x**2 - 4*coeff_x2*coeff_const
    if delta < 0:  # No intersection
        return None

    x_sol1 = (-coeff_x + np.sqrt(delta)) / (2*coeff_x2)
    x_sol2 = (-coeff_x - np.sqrt(delta)) / (2*coeff_x2)

    y_sol1 = m*x_sol1 + c
    y_sol2 = m*x_sol2 + c

    d1 = (x1 - x_sol1)**2 + (y1 - y_sol1)**2
    d2 = (x1 - x_sol2)**2 + (y1 - y_sol2)**2
    if d1 < d2:
        return x_sol1, y_sol1
    else:
        return x_sol2, y_sol2

def projection_on_ellipse(a, b, center, external_point):

    h, k = center
    x1, y1 = external_point
    
    def equations(p):
        x, y, lam = p
        eq1 = 2*(x - x1) + 2*lam * (x - h)/a**2
        eq2 = 2*(y - y1) + 2*lam * (y - k)/b**2
        eq3 = eq3 = (x - h)**2/a**2 + (y - k)**2/b**2 - 1
        return [eq1, eq2, eq3]
    
    # Finding a better initial guess
    init_x, init_y = find_nearest_intersection(a, b, center, external_point)
    initial_guess = (init_x, init_y, 4)
    x, y, _ = fsolve(equations, initial_guess, xtol=1e-13, maxfev=10000)
    
    return x, y

def tangent_to_ellipse(a, b, center, external_point):
    point_on_ellipse = projection_on_ellipse(a, b, center, external_point)
    h, k = center
    X, Y = point_on_ellipse

    coeff_a = b**2 * (X - h)
    coeff_b = a**2 * (Y - k)
    coeff_c = b**2 * (h**2 - h*X) + a**2 * (k**2 - k*Y) - a**2 * b**2 * 1

    return coeff_a, coeff_b, coeff_c


def plot_ellipse_and_external_point(center, a, b, external_point, projection_point):
    h, k = center
    x1, y1 = external_point
    x2, y2 = projection_point

    theta = np.linspace(0, 2 * np.pi, 1000)
    x = h + a * np.cos(theta)
    y = k + b * np.sin(theta)

    plt.plot(x, y, label='Ellipse', color='blue')

    plt.scatter(x1, y1, color='green', label='External Point')
    
    plt.scatter(x2, y2, color='red', label='Projection Point')
    
    plt.legend()

    plt.axis('equal')  
    plt.grid(True)
    plt.show()
    
    
def plot_ellipse_and_tangent(a, b, center, external_point):
    x_proj, y_proj = projection_on_ellipse(a, b, center, external_point)

    coeff_a, coeff_b, coeff_c = tangent_to_ellipse(a, b, center, (x_proj, y_proj))

    # 计算切线方程 y = (-coeff_a * x - coeff_c) / coeff_b
    x_vals = np.linspace(center[0] - a - 1, center[0] + a + 1, 400)
    y_vals = (-coeff_a * x_vals - coeff_c) / coeff_b

    theta = np.linspace(0, 2 * np.pi, 1000)
    x = center[0] + a * np.cos(theta)
    y = center[1] + b * np.sin(theta)
    plt.plot(x, y, label='Ellipse', color='blue')

    plt.plot(x_vals, y_vals, label='Tangent', color='red')

    plt.plot(external_point[0], external_point[1], 'go', label='External Point')

    plt.plot(x_proj, y_proj, 'ro', label='Projection Point')


    