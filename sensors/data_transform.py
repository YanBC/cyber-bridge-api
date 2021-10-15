import numpy as np


def stdvec(x):
    """ stdvec returns a 1D numpy array, given a list or 2d numpy array.
    """
    x = np.asarray(x)
    x = x.reshape(len(x),)
    return x


# Smoothing function.
def smooth(x, n):
    """ smooth(x,n): smooths x over a moving window of size n. """
    x = stdvec(x)
    return np.convolve(x, np.ones((n,))/n, mode='full')


def curvature(x, y, n=0):
    """ Calculate the curvature for x, y coords.
    """
    # Make sure the format is correct.
    x = stdvec(x)
    y = stdvec(y)
    # Calculate first derivative.
    d1 = np.sqrt(np.diff(y)**2 + np.diff(x)**2)
    dx = np.diff(x)/d1
    dy = np.diff(y)/d1
    d1x = (dx[0:-1] + dx[1:])/2
    d1y = (dy[0:-1] + dy[1:])/2
    # Calculate second derivative.
    d2 = (d1[0:-1] + d1[1:])/2
    d2x = np.diff(dx)/d2
    d2y = np.diff(dy)/d2
    # Smooth the derivatives (optional).
    if n:
        d1x = smooth(d1x, n)
        d1y = smooth(d1y, n)
        d2x = smooth(d2x, n)
        d2y = smooth(d2y, n)
    # Calculate curvature.
    return (d1x*d2y - d1y*d2x)/((d1x**2 + d1y**2)**(3/2))
