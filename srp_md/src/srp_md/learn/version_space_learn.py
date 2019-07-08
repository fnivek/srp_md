# SRP MD imports
import learn

# Python imports
import logging


class VersionSpaceLearner(learn.BaseLearner):
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def learn(self, obs):
        self._logger.debug('Learn')
        return self.canidate_elimination(obs)

    def canidate_elimination(self, examples):
        """ Canidate Elimination.

        The canidate elimination algorithm takes observations and generates a
        consistent version space of hypothesis.

        Assumptions:
          * No noise/errors in examples
          * There is a consistent conjunctive hypothesis with the examples

        Algorithmic Notes:
          * Convergence is fater if the hypotheses are ordered s.t. examples
            that satisfy closses to half the hypotheses in the current version
            space are used first
          * If Null set returned the examples are not consistent
          * Many hypothesis may be found and a strategy for picking one must be
            chosen

        Input:
          examples - a list of tuples of features and goal classification, i.e.
              [((Exist(X), not On(X, Y), On(Z, X)), False), ((...), True), ...]

        output:

        """
        # Init general h_g and specific h_s boundries
        h_g = [[None for _ in examples[0][0]]]
        h_s = None
        self._logger.debug('h_g: %s', h_g)
        self._logger.debug('h_s: %s', h_s)

        # current_examples = []
        for ex in examples:
            # current_examples.append(ex)

            # Positive example
            if ex[1]:
                # Prune h_g of inconsistent hypotheses
                h_g = [h for h in h_g if self.consistent(h, [ex])]
                self._logger.debug('h_g pruned to: %s', h_g)

                # Generalize h_s or initilize if first positive example
                if h_s is None:
                    h_s = [ex[0]]
                    self._logger.debug('h_s initilized to: %s', h_s)
                    continue
                h_s_copy = list(h_s)
                for h in h_s_copy:
                    if h is not self.consistent(h, [ex]):
                        # Make all inconsistentcies None
                        h_s.remove(h)
                        h_new = list(h)
                        for i in xrange(len(h)):
                            if h[i] != ex[0][i]:
                                h_new[i] = None
                        # Prune if more general than any h in h_g
                        if any(g > self.generality(h_new) for g in
                               map(self.generality, h_g)):
                            self._logger.debug('Generalize %s to %s', h, h_new)
                            h_s.append(h_new)
                        else:
                            self._logger.debug(
                                '%s is more general than any h in h_g', h_new)
                        # TODO(Henry): Remove Any hypothesis that is more
                        # general than another hypothesis in S

            # Negative example
            else:
                # Prune h_s of inconsistent hypotheses
                if h_s is not None:
                    h_s = [h for h in h_s if self.consistent(h, [ex])]
                    self._logger.debug('h_s pruned to: %s', h_s)

                # Specialize h_g (stay consistent with h_s)
                h_g_copy = list(h_g)
                for h in h_g_copy:
                    if h is not self.consistent(h, [ex]):
                        h_g.remove(h)
                        # TODO(Henry): Generalize multiple features if needed
                        for i in xrange(len(h)):
                            if h[i] is None:
                                h_new = list(h)
                                h_new[i] = not ex[0][i]
                                # TODO(Henry): Ensure that h_new is more
                                # general than some member in h_s

                                # Add h_new
                                h_g.append(h_new)
                                self._logger.debug('%s specialized to %s', h,
                                                   h_new)
                        # TODO(Henry): Remove for h_g any hypothesis that is
                        # less general than another hypothesis in h_g

    def consistent(self, h, examples):
        """ Check for consistency. """
        consistent = True
        for ex in examples:
            # Positive example all features not None must match
            if ex[1]:
                consistent = all([h[i] == ex[0][i] for i in xrange(len(h))
                                 if h[i] is not None])

            # Negative example
            else:
                consistent = any([h[i] != ex[0][i] for i in xrange(len(h))
                                 if h[i] is not None])
            if not consistent:
                self._logger.debug('%s is not consistent with %s', h, examples)
                return False

        self._logger.debug('%s is consistent with %s', h, examples)
        return True

    def generality(self, h):
        return sum(1 for x in h if x is None)


# Register the learner
learn.learners['version_space_learner'] = VersionSpaceLearner
