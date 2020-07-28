import numpy as np
import pdb

import random

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """

        weights = X_bar[:,-1]
        weights = weights /np.sum(weights)
        num_particles = X_bar.shape[0]

        X_bar_resampled=[]
        r = random.uniform(0,(1.0/num_particles))
        c=weights[0]
        i=0
        for m in range(1,num_particles+1):

            U= r + (m-1)*(1.0/num_particles)
            while(U > c):
                i=i+1
                c=c+weights[i]

            X_bar_resampled.append(X_bar[i])


        return np.array(X_bar_resampled)



if __name__ == "__main__":
    pass
