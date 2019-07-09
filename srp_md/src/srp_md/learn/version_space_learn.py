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
                # For h_g: Prune h_g of inconsistent hypotheses
                h_g = [h for h in h_g if self.consistent(h, [ex], 'b')]
                self._logger.debug('h_g pruned to: %s', h_g)

                # For h_s: Generalize h_s or initilize if first positive example
                if h_s is None:
                    h_s = [ex[0]]
                    self._logger.debug('h_s initilized to: %s', h_s)
                    self._logger.debug('Example Learned.')
                    continue

                # For h_s: Generalize h_s to include positive example
                h_s_copy = list(h_s)
                for h in h_s_copy:
                    if h is not self.consistent(h, [ex], 'g'):
                        # Remove h from h_s
                        h_s.remove(h)
                        # Generalize h with None's
                        h_new = list(h)
                        for i in xrange(len(h)):
                            if h[i] != ex[0][i]:
                                h_new[i] = None
                        # Add to h_s if some member of h_g is more general than h_new
                        # and h_new is consistent with example
                        if any(self.generality(g, h_new) for g in h_g) and self.consistent(h_new, [ex], 'g'):
                            self._logger.debug('Generalize %s to %s', h, h_new)
                            # Don't add if more general than other h in h_s
                            if any(self.generality(h_new, s) for s in h_s):
                                pass
                            # Also prune h_s more general hypotheses of h_new
                            else:
                                h_s = [s for s in h_s if not self.generality(s, h_new)]
                                h_s.append(h_new)
                        # Prune otherwise
                        else:
                            self._logger.debug(
                                '%s is more general than any h in h_g', h_new)

            # Negative example
            else:
                # For h_s: Prune h_s of inconsistent hypotheses
                if h_s is not None:
                    h_s = [h for h in h_s if self.consistent(h, [ex], 'g')]
                    self._logger.debug('h_s pruned to: %s', h_s)

                # For h_g: Specify h_g to exclude negative example
                h_g_copy = list(h_g)
                for h in h_g_copy:
                    #print "What is h?", h
                    if not self.consistent(h, [ex], 'b'):
                        #print "Did it come here?"
                        # Remove h from h_g
                        h_g.remove(h)
                        # For each minimal specification of h, do
                        for i in xrange(len(h)):
                            if h[i] is None:
                                h_new = list(h)
                                h_new[i] = not ex[0][i]
                                # Add to h_g if some member of h_s is more specific than h_new
                                # and h_new is consistent with example
                                if (h_s is None or any(self.generality(h_new, s) for s in h_s)) and self.consistent(h_new, [ex], 'b'):
                                    self._logger.debug('Specified %s to %s', h, h_new)
                                    # Don't add if more specific than other h in h_g
                                    if any(self.generality(g, h_new) for g in h_g):
                                        pass
                                    # Also prune h_g more specific hypotheses of h_new
                                    else:
                                        h_g = [g for g in h_g if not self.generality(h_new, g)]
                                        h_g.append(h_new)
                                # Prune otherwise
                                else:
                                    self._logger.debug(
                                        '%s is more specific than any h in h_s', h_new)

            self._logger.debug('h_g: %s', h_g)
            self._logger.debug('h_s: %s', h_s)
            self._logger.debug('Example Learned.')

    def consistent(self, h, examples, color):
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
                if color == 'b':
                    self._logger.debug('%s is not consistent with %s', h, ex)
                else:
                    self._logger.debug('%s is not consistent with %s', h, ex)
                return consistent

        if color == 'b':
            self._logger.debug('%s is consistent with %s', h, examples)
        else:
            self._logger.debug('%s is consistent with %s', h, examples)
        #print "What is consistent: ", consistent
        return consistent

    def generality(self, h_1, h_2):
        """ Check if h_1 is more general than or equal to h_2 """
        general = all([h_1[i] == h_2[i] for i in xrange(len(h_1)) if h_1[i] is not None])
        return general

# Register the learner
learn.learners['version_space_learner'] = VersionSpaceLearner
