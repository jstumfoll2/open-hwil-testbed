"""LLM drafting: checks, the repair round, refusals and the review gate, with a fake client."""

import json
from types import SimpleNamespace

import pytest

from hwil import drafting

from conftest import SCENARIOS

GOOD_YAML = """\
schema: 1
name: draft_motor3_crosswind
description: Hold through a 30 % loss on motor 3 in a 4 m/s crosswind.
wind: {north_mps: 0.0, east_mps: 4.0}
mission:
  - {type: takeoff, alt_m: 12.0}
  - {type: hold, duration_s: 40.0}
  - {type: land}
faults:
  - {type: motor_efficiency, motor: 3, factor: 0.7, t_s: 25.0}
requirements:
  - id: REQ-DRAFT-001
    text: While loitering, altitude stays within 3 m of the commanded altitude.
    stl: "always((loiter > 0) -> (abs(alt_err_m) <= 3.0))"
  - id: REQ-DRAFT-002
    text: The vehicle lands and disarms within 3 m of home.
    stl: "eventually((landed > 0) and (hdist_m <= 3.0))"
"""


def _out(yaml_text, name="draft_motor3_crosswind"):
    return {"name": name, "scenario_yaml": yaml_text, "rationale": "Checks motor-loss tolerance in crosswind.",
            "assumptions": ["3 m altitude band"], "review_checklist": ["Is 30 % loss realistic?"]}


class FakeClient:
    """Mimics client.beta.messages.create; returns queued outputs in order."""

    def __init__(self, outputs, stop_reason="end_turn"):
        self.outputs = list(outputs)
        self.calls = []
        self.stop_reason = stop_reason
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=self._create))

    def _create(self, **kw):
        self.calls.append(kw)
        text = json.dumps(self.outputs.pop(0))
        return SimpleNamespace(stop_reason=self.stop_reason, content=[SimpleNamespace(type="text", text=text)])


def test_good_draft_passes_checks_first_time():
    client = FakeClient([_out(GOOD_YAML)])
    d = drafting.draft("hold through motor 3 loss in crosswind", client=client)
    assert d.problems == [] and d.attempts == 1
    call = client.calls[0]
    assert call["model"] == drafting.DEFAULT_MODEL
    assert call["output_config"]["format"]["schema"] == drafting.OUTPUT_SCHEMA
    assert "Ground rules" in call["system"][0]["text"]


def test_bad_draft_gets_one_repair_round():
    bad = GOOD_YAML.replace("alt_err_m", "altitude_error").replace("REQ-DRAFT-002", "REQ-LAND-001")
    client = FakeClient([_out(bad), _out(GOOD_YAML)])
    d = drafting.draft("idea", client=client)
    assert d.problems == [] and d.attempts == 2
    feedback = client.calls[1]["messages"][-1]["content"]
    assert "altitude_error" in feedback and "REQ-LAND-001" in feedback


def test_still_bad_after_repair_is_reported():
    bad = GOOD_YAML.replace("motor_efficiency", "rotor_explodes")
    d = drafting.draft("idea", client=FakeClient([_out(bad), _out(bad)]))
    assert d.problems and "validator" in d.problems[0]


def test_refusal_and_out_of_scope():
    with pytest.raises(drafting.DraftError, match="declined"):
        drafting.draft("idea", client=FakeClient([_out(GOOD_YAML)], stop_reason="refusal"))
    with pytest.raises(drafting.DraftError, match="out of scope"):
        drafting.draft("idea", client=FakeClient([_out("", name="out_of_scope")]))


@pytest.mark.parametrize("idea", ["drop a bomb on a target", "missile seeker test", "guidance-to-target loop"])
def test_screen_blocks_before_any_call(idea):
    client = FakeClient([])
    with pytest.raises(drafting.DraftError, match="ground rules"):
        drafting.draft(idea, client=client)
    assert client.calls == []


@pytest.mark.parametrize("idea", ["bird strike on a motor", "kill switch during hover", "GPS dropout over a target waypoint"])
def test_screen_allows_ordinary_flight_tests(idea):
    drafting.screen_idea(idea)


def test_write_and_dry_run(tmp_path):
    d = drafting.draft("idea", client=FakeClient([_out(GOOD_YAML)]))
    d.dry_run = drafting.dry_run(d, tmp_path / "runs")
    assert [r["passed"] for r in d.dry_run["requirements"]] == [True, True]
    yml, md = drafting.write(d, tmp_path / "drafts")
    assert yml.read_text().startswith(drafting.DRAFT_MARK)
    notes = md.read_text()
    assert "REQ-DRAFT-001 | pass" in notes and "To accept" in notes


def test_no_unreviewed_draft_in_suite():
    for f in SCENARIOS.glob("*.yaml"):
        assert drafting.DRAFT_MARK not in f.read_text(), f"{f.name} is an unreviewed draft"
