"""LLM-drafted test scenarios with human review (M5).

    hwil draft "hold position through a 30 % loss on motor 3 in a 5 m/s crosswind"

Claude turns a plain-language test idea into a scenario file: mission, wind,
faults and STL requirements. The draft is checked before anyone sees it:

1. The YAML must pass the same validator as every scenario (``hwil validate``).
2. Every STL requirement may only use signals that exist in the log.
3. Requirement IDs must not clash with the existing suite.
4. Unless ``--no-run`` is given, the draft is flown once with the builtin
   controller, and the review notes record which requirements passed, failed or
   were never exercised.

If any of the first three checks fails, the errors are sent back once for a
repair. The result goes to ``scenarios/drafts/<name>.yaml`` with a DRAFT header,
next to ``<name>.review.md``. Nothing in ``scenarios/drafts/`` runs in the suite
or CI. A person reads the draft, edits it, removes the header and moves it into
``scenarios/``. The test suite refuses any scenario in ``scenarios/`` that still
carries the DRAFT header.

The API key is read the usual way (``ANTHROPIC_API_KEY`` or an ``ant auth
login`` profile). Install the extra with ``pip install -e ".[ai]"``.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import scenario as scn
from .executive import LOG_COLUMNS
from .requirements import DERIVED

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = "claude-opus-5-5"
DRAFT_MARK = "# DRAFT: written by an LLM, not yet reviewed."
MAX_ATTEMPTS = 2

# The project's ground rules apply to drafted tests too. The system prompt says
# so, and this screen stops obviously out-of-scope ideas before any API call.
OUT_OF_SCOPE = re.compile(
    r"\b(weapons?|warheads?|missiles?|munitions?|seekers?|payload release|bomb(s|ing)?|"
    r"targeting|guidance[- ]to[- ]target|engagement)\b",
    re.IGNORECASE)

STL_WORDS = {"always", "eventually", "historically", "once", "since", "until", "and", "or", "not",
             "implies", "abs", "prev", "next", "rise", "fall", "s", "ms", "us", "true", "false"}
SIGNALS = set(LOG_COLUMNS) | set(DERIVED)

OUTPUT_SCHEMA = {
    "type": "object",
    "properties": {
        "name": {"type": "string", "description": "snake_case file stem, 3 to 40 characters"},
        "scenario_yaml": {"type": "string", "description": "the complete scenario file"},
        "rationale": {"type": "string", "description": "what the test checks and why the limits are what they are"},
        "assumptions": {"type": "array", "items": {"type": "string"}},
        "review_checklist": {"type": "array", "items": {"type": "string"},
                             "description": "specific things a reviewer should check before accepting"},
    },
    "required": ["name", "scenario_yaml", "rationale", "assumptions", "review_checklist"],
    "additionalProperties": False,
}


class DraftError(RuntimeError):
    pass


@dataclass
class Draft:
    idea: str
    name: str
    scenario_yaml: str
    rationale: str
    assumptions: list[str]
    review_checklist: list[str]
    model: str
    attempts: int
    problems: list[str] = field(default_factory=list)
    dry_run: dict | None = None


def screen_idea(idea: str) -> None:
    m = OUT_OF_SCOPE.search(idea)
    if m:
        raise DraftError(f"'{m.group(0)}' is outside this project's ground rules (civilian, generic "
                         "flight testing only); nothing was sent.")


def _existing() -> tuple[list[str], set[str]]:
    lines, ids = [], set()
    for p in sorted((ROOT / "scenarios").glob("*.yaml")):
        raw = yaml.safe_load(p.read_text()) or {}
        lines.append(f"- {p.stem}: {str(raw.get('description', '')).strip().splitlines()[0] if raw.get('description') else ''}")
        ids.update(r.get("id", "") for r in raw.get("requirements", []))
    return lines, ids


def system_prompt() -> str:
    existing, _ = _existing()
    docs = (ROOT / "docs" / "scenarios.md").read_text()
    examples = "\n\n".join(f"### scenarios/{n}.yaml\n```yaml\n{(ROOT / 'scenarios' / f'{n}.yaml').read_text()}```"
                           for n in ("motor_degraded", "waypoint_square"))
    return f"""You write test scenarios for open-hwil-testbed, an open-source hardware-in-the-loop test
bench for a small civilian quadcopter (a generic F450 airframe in JSBSim) flown either by a simple
builtin controller or by PX4.

Ground rules, which override any request: civilian and generic flight testing only. No weapons,
targeting, guidance-to-target, seeker, payload-release or engagement content. Everything you write
will be published. If an idea falls outside this, return a scenario named "out_of_scope" with an
empty scenario_yaml and explain in the rationale.

Write one scenario file for the test idea the user gives. Rules:
- Follow the file format below exactly. Use only mission types takeoff, hold, goto, land (takeoff
  first, land last) and only fault types motor_efficiency, gps_dropout, baro_bias.
- Every requirement needs a unique id starting with REQ-DRAFT-, a plain-English text that a reviewer
  can check against the STL, and an STL spec using only the signals listed below.
- Make each requirement exercised: guard it with a phase the mission actually reaches, and keep time
  windows inside the run. Include a final landing requirement.
- Choose limits a healthy vehicle meets with margin, and that the fault under test could plausibly
  break. State every limit you guessed in assumptions.
- Keep wind at or below 6 m/s per axis; stronger wind tips the vehicle before takeoff.
- Do not duplicate an existing scenario; say how yours differs in the rationale.
- In review_checklist, list the specific judgment calls a reviewer should check.

Log signals: {", ".join(LOG_COLUMNS)}
Derived signals: {", ".join(f"{k} ({v})" for k, v in DERIVED.items())}

Existing scenarios:
{chr(10).join(existing)}

# Scenario file format (docs/scenarios.md)

{docs}

# Examples

{examples}
"""


def check(text: str, name: str) -> tuple[dict | None, list[str]]:
    """Validate a drafted scenario. Returns (resolved scenario, problems)."""
    problems: list[str] = []
    if not re.fullmatch(r"[a-z][a-z0-9_]{2,39}", name or ""):
        problems.append(f"name {name!r} must be snake_case, 3 to 40 characters")
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as e:
        return None, problems + [f"YAML does not parse: {e}"]
    if not isinstance(raw, dict):
        return None, problems + ["scenario_yaml must be a YAML mapping"]
    try:
        sc = scn.resolve(raw)
        scn.validate(sc)
    except (scn.ScenarioError, KeyError, TypeError, ValueError) as e:
        return None, problems + [f"validator: {e}"]
    if not sc["requirements"]:
        problems.append("the scenario has no requirements")
    _, existing_ids = _existing()
    seen = set()
    for r in sc["requirements"]:
        rid = r["id"]
        if not str(rid).startswith("REQ-DRAFT-"):
            problems.append(f"{rid}: ids must start with REQ-DRAFT-")
        if rid in existing_ids or rid in seen:
            problems.append(f"{rid}: id already used")
        seen.add(rid)
        unknown = sorted({w for w in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", re.sub(r"\d+(\.\d+)?(ms|us|s)\b", "", r["stl"]))
                          if w not in SIGNALS and w not in STL_WORDS})
        if unknown:
            problems.append(f"{rid}: unknown signal(s) {', '.join(unknown)}")
    return sc, problems


def _call(client, model: str, system: str, messages: list) -> tuple[dict, list]:
    try:
        import anthropic
    except ImportError:  # a test's fake client needs no SDK
        anthropic = None
    request = dict(
        model=model,
        max_tokens=16000,
        system=[{"type": "text", "text": system, "cache_control": {"type": "ephemeral"}}],
        messages=messages,
        output_config={"effort": "high", "format": {"type": "json_schema", "schema": OUTPUT_SCHEMA}},
        # Server-side fallback if the model declines; routed by refusal category.
        betas=["server-side-fallback-2026-07-01"],
        fallbacks="default",
    )
    if anthropic is None:
        resp = client.beta.messages.create(**request)
    else:
        try:
            resp = client.beta.messages.create(**request)
        except anthropic.AuthenticationError as e:
            raise DraftError("no valid Anthropic credentials (set ANTHROPIC_API_KEY or run `ant auth login`)") from e
        except anthropic.APIStatusError as e:
            raise DraftError(f"Claude API error {e.status_code}: {e.message}") from e
        except anthropic.APIConnectionError as e:
            raise DraftError(f"could not reach the Claude API: {e}") from e
        except TypeError as e:  # raised by the SDK when no credentials are configured
            if "authentication" not in str(e):
                raise
            raise DraftError("no Anthropic credentials found (set ANTHROPIC_API_KEY or run `ant auth login`)") from e
    if resp.stop_reason == "refusal":
        raise DraftError("the model declined to draft this test")
    if resp.stop_reason == "max_tokens":
        raise DraftError("the draft was cut off (max_tokens)")
    text = next((b.text for b in resp.content if b.type == "text"), "")
    try:
        return json.loads(text), resp.content
    except json.JSONDecodeError as e:
        raise DraftError(f"model output was not valid JSON: {e}") from e


def draft(idea: str, client=None, model: str = DEFAULT_MODEL) -> Draft:
    screen_idea(idea)
    if client is None:
        try:
            import anthropic
        except ImportError as e:
            raise DraftError('the anthropic package is missing; install with pip install -e ".[ai]"') from e
        client = anthropic.Anthropic()
    system = system_prompt()
    messages = [{"role": "user", "content": f"Test idea: {idea}"}]
    out, problems = {}, []
    for attempt in range(1, MAX_ATTEMPTS + 1):
        out, content = _call(client, model, system, messages)
        if out.get("name") == "out_of_scope":
            raise DraftError(f"the model judged this out of scope: {out.get('rationale', '')}")
        _, problems = check(out.get("scenario_yaml", ""), out.get("name", ""))
        if not problems:
            break
        if attempt < MAX_ATTEMPTS:
            messages += [{"role": "assistant", "content": content},
                         {"role": "user", "content": "The draft failed these checks. Fix them and return the "
                          "whole draft again:\n" + "\n".join(f"- {p}" for p in problems)}]
    return Draft(idea=idea, name=out.get("name", "draft"), scenario_yaml=out.get("scenario_yaml", ""),
                 rationale=out.get("rationale", ""), assumptions=list(out.get("assumptions", [])),
                 review_checklist=list(out.get("review_checklist", [])), model=model, attempts=attempt,
                 problems=problems)


def dry_run(d: Draft, out_root: Path) -> dict:
    from .run import run_scenario

    sc, problems = check(d.scenario_yaml, d.name)
    if sc is None or problems:
        return {"skipped": "draft did not pass the checks"}
    _, man = run_scenario(sc, {"plots": False}, out_root, run_name=f"draft_{d.name}")
    res = man["results"]
    return {"aborted": res.get("aborted"), "landed": res.get("landed"),
            "requirements": [{"id": r["id"], "passed": r["passed"], "vacuous": r.get("vacuous", False),
                              "robustness": r["robustness"]} for r in res["requirements"]]}


def write(d: Draft, out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    yml = out_dir / f"{d.name}.yaml"
    yml.write_text(f"{DRAFT_MARK}\n# Idea: {d.idea}\n# Review {d.name}.review.md, then remove these lines and move "
                   f"the file into scenarios/.\n{d.scenario_yaml.rstrip()}\n", encoding="utf-8")
    lines = [f"# Review: {d.name}", "", f"**Idea:** {d.idea}", "",
             f"Drafted by `{d.model}` in {d.attempts} attempt(s). Not reviewed yet.", ""]
    if d.problems:
        lines += ["## Checks that still fail", "", *[f"- {p}" for p in d.problems], ""]
    else:
        lines += ["Passed the validator, the signal check and the ID check.", ""]
    lines += ["## Rationale", "", d.rationale, "", "## Assumptions to confirm", "",
              *[f"- [ ] {a}" for a in d.assumptions], "", "## Review checklist", "",
              *[f"- [ ] {c}" for c in d.review_checklist],
              "- [ ] Every requirement's text says the same thing as its STL",
              "- [ ] The test is civilian and generic and fits the ground rules", ""]
    if d.dry_run:
        lines += ["## Dry run (builtin controller)", ""]
        if "skipped" in d.dry_run:
            lines += [f"Skipped: {d.dry_run['skipped']}", ""]
        else:
            if d.dry_run.get("aborted"):
                lines += [f"Run aborted: {d.dry_run['aborted']}", ""]
            lines += ["| Requirement | Result | Margin |", "|---|---|---|"]
            for r in d.dry_run["requirements"]:
                verdict = "NOT EXERCISED" if r["vacuous"] else ("pass" if r["passed"] else "FAIL")
                lines.append(f"| {r['id']} | {verdict} | {r['robustness']:.3g} |")
            lines += ["", "A failure here is either a wrong limit in the draft or a real finding. "
                      "Decide which before accepting.", ""]
    lines += ["## To accept", "", f"1. Edit `{yml.name}` until the checklist holds.",
              "2. Delete the three `#` header lines at the top.",
              f"3. `git mv scenarios/drafts/{yml.name} scenarios/` and run `pytest tests/test_all_scenarios.py`.",
              f"4. Delete `{d.name}.review.md`, or keep it in the pull request description.", ""]
    md = out_dir / f"{d.name}.review.md"
    md.write_text("\n".join(lines), encoding="utf-8")
    return yml, md
