from Resampling import Resampling
import numpy as np

num_particles=20
y0_vals = np.random.uniform( 3600, 4300, (num_particles, 1) )
x0_vals = np.random.uniform( 3500, 5000, (num_particles, 1) )
theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

# initialize weights for all particles

w0_vals = np.random.uniform(0,100,(num_particles,1))
w0_vals = w0_vals / num_particles

X_bar = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
resampler = Resampling()
print(X_bar)
for i in range(num_particles):
    X_bar = resampler.low_variance_sampler(X_bar)
    print(X_bar)
    input()
