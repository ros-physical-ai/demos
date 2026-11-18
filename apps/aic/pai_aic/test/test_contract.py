"""The AIC contract loads against the built workspace."""

from pathlib import Path

from rosetta.contract.schema import load_contract

CONTRACT = Path(__file__).resolve().parents[1] / "config" / "rosetta" / "aic.yaml"
FPS = 20


def test_contract_loads():
    """Load-time validation passes: codec paths resolve, select names and timelines check out."""
    contract = load_contract(CONTRACT)
    assert contract.fps == FPS


def test_contract_has_one_numeric_observation_and_one_action():
    """LeRobot's live path allows one numeric observation key and one action key."""
    contract = load_contract(CONTRACT)
    numeric = [e.key for e in contract.observations if not e.key.startswith("observation.images")]
    assert numeric == ["observation.state"]
    assert [e.key for e in contract.actions] == ["action"]
