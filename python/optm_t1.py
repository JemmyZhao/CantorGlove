import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import time
import datetime


def f(x):
    return x**4 - 14 * x**3 + 60 * x**2 - 70 * x


def golden_section_optm(function, x_range, iterations):
    rho = (3-5**0.5)/2
    a = x_range[0]
    b = x_range[1]
    pair = []
    for i in range(iterations):
        a_c = a + (b-a) * rho
        b_c = b - (b-a) * rho
        fa_c = function(a_c)
        fb_c = function(b_c)
        if fa_c < fb_c:
            b = b_c
        elif fa_c > fb_c:
            a = a_c
        pair.append([a_c, fa_c, b_c, fb_c])
    return pair


# Golden Section
def golden_section_optm_fast(function, x_range, max_err):
    rho = (3-5**0.5)/2
    a = x_range[0]
    b = x_range[1]
    iterations = int(np.log(max_err/(b-a)) / np.log(1 - rho)) + 1
    print('iteration:', iterations)
    a_c = a + (b - a) * rho
    b_c = b - (b - a) * rho
    fa_c = function(a_c)
    fb_c = function(b_c)
    pair = []
    for i in range(iterations):
        pair.append([a_c, fa_c, b_c, fb_c])
        if b == b_c:
            b_c = a_c
            fb_c = fa_c
            a_c = a + (b-a) * rho
            fa_c = function(a_c)
        if a == a_c:
            a_c = b_c
            fa_c = fb_c
            b_c = b - (b-a) * rho
            fb_c = function(b_c)
        if fa_c < fb_c:
            b = b_c
        elif fa_c > fb_c:
            a = a_c
    return pair


# Fibonacci optimization
def cal_fib_table(num):
    fib_table = [0, 1]
    for i in range(num):
        if i > 1:
            fib = fib_table[i-1] + fib_table[i-2]
            fib_table.append(fib)
    return fib_table


def fibonacci_optm(function, x_range, max_err):
    return 0

print(cal_fib_table(10))
print()
X = np.linspace(0, 2, 1000)
Y = f(X)
y_min_absolute = min(Y)
x_range = [0, 2]
t0 = time.time()
y_min1 = golden_section_optm(f, x_range, 100)
t1 = time.time()
y_min2 = golden_section_optm_fast(f, x_range, 0.0001)
t2 = time.time()
print('Slow algorithm:  time = ', t1 - t0, 'min=', y_min1[-1])
print('Fast algorithm:  time = ', t2 - t1, 'min=', y_min2[-1])


point_x = np.array(y_min2)

fig = plt.figure()
plt.plot(X, Y)
plt.plot(point_x[:, 0], point_x[:, 1], 'go')
plt.plot(point_x[:, 2], point_x[:, 3], 'ro')
plt.show()

# Optimization
