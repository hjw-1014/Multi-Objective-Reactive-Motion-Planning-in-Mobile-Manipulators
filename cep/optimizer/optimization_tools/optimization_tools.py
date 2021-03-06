import numpy as np


class BestSolution(object):
    """container to keep track of the best solution seen.
    Keeps also track of the genotype, if available.
    """
    def __init__(self, x=None, f=np.inf, evals=None):
        """initialize the best solution with ``x``, ``f``, and ``evals``.
        Better solutions have smaller ``f``-values.
        """
        self.x = x
        self.x_geno = None
        self.f = f if f is not None and f is not np.nan else np.inf
        self.evals = evals
        self.evalsall = evals
        self.last = _BlancClass()
        self.last.x = x
        self.last.f = f
    def update(self, arx, xarchive=None, arf=None, evals=None):
        """checks for better solutions in list ``arx``.
        Based on the smallest corresponding value in ``arf``,
        alternatively, `update` may be called with a `BestSolution`
        instance like ``update(another_best_solution)`` in which case
        the better solution becomes the current best.
        ``xarchive`` is used to retrieve the genotype of a solution.
        """
        if isinstance(arx, BestSolution):
            if self.evalsall is None:
                self.evalsall = arx.evalsall
            elif arx.evalsall is not None:
                self.evalsall = max((self.evalsall, arx.evalsall))
            if arx.f is not None and arx.f < np.inf:
                self.update([arx.x], xarchive, [arx.f], arx.evals)
            return self
        assert arf is not None
        # find failsave minimum
        try:
            minidx = np.nanargmin(arf)
        except ValueError:
            return
        if minidx is np.nan:
            return
        minarf = arf[minidx]
        # minarf = reduce(lambda x, y: y if y and y is not np.nan
        #                   and y < x else x, arf, np.inf)
        if minarf < np.inf and (minarf < self.f or self.f is None):
            self.x, self.f = arx[minidx], arf[minidx]
            if xarchive is not None and xarchive.get(self.x) is not None:
                self.x_geno = xarchive[self.x].get('geno')
            else:
                self.x_geno = None
            self.evals = None if not evals else evals - len(arf) + minidx + 1
            self.evalsall = evals
        elif evals:
            self.evalsall = evals
        self.last.x = arx[minidx]
        self.last.f = minarf
    def get(self):
        """return ``(x, f, evals)`` """
        return self.x, self.f, self.evals  # , self.x_geno
