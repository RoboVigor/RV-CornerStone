"""Bounded propose/register adapter; no library-owned evaluation loop or motion."""
from dataclasses import replace
from importlib.metadata import version
import math
from numbers import Real
from .config import STAGE_FIELDS, f32, wire_bounds


class BayesianProposer:
    def __init__(self, config):
        config.validate("C")
        try:
            installed = version("bayesian-optimization")
        except ModuleNotFoundError:
            raise RuntimeError("install optional bayesian-optimization==3.1.0 and packaging==25.0") from None
        if installed != "3.1.0":
            raise RuntimeError("requires bayesian-optimization==3.1.0")
        from bayes_opt import BayesianOptimization
        from bayes_opt.acquisition import UpperConfidenceBound
        c = config.raw["gate_c"]
        self.fields = STAGE_FIELDS[c["stage"]]
        self.bounds = {name: wire_bounds(*c["search"][name]) for name in self.fields}
        self.optimizer = BayesianOptimization(f=None, pbounds=self.bounds, random_state=c["random_seed"],
                                               verbose=0, allow_duplicate_points=True,
                                               acquisition_function=UpperConfidenceBound(kappa=2.576))
        self.observations = 0

    def register(self, parameters, outcome):
        if not outcome.get("valid") or not math.isfinite(outcome["utility"]):
            raise ValueError("only complete valid aggregate observations may be registered")
        self.optimizer.register(params={name: getattr(parameters, name) for name in self.fields}, target=outcome["utility"])
        self.observations += 1

    def suggest(self):
        if not self.observations:
            raise ValueError("evaluate/register the accepted baseline before any suggestion")
        return self.optimizer.suggest()


def materialize(config, proposal, version_number):
    """Only stage axes vary; inactive gains and algorithm remain accepted baseline."""
    c = config.raw["gate_c"]
    fields = STAGE_FIELDS[c["stage"]]
    if type(proposal) is not dict or set(proposal) != set(fields):
        raise ValueError("proposal axes differ from the approved stage")
    values = {}
    for name in fields:
        lo, hi = wire_bounds(*c["search"][name])
        value = proposal[name]
        # numpy scalar outputs are normalized explicitly; bool still forbidden.
        if isinstance(value, bool) or not isinstance(value, Real):
            raise ValueError("proposal must be a real number, not bool/string")
        value = float(value)
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError("proposal outside admitted finite bounds")
        values[name] = min(hi, max(lo, f32(value)))
    result = replace(config.baseline, version=version_number, **values).validate(config.safety)
    return type(result).unpack(result.pack())
