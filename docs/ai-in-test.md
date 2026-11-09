# AI in test (M5)

M5 adds two tools on top of the STL harness. Both work offline on run folders and
scenario files. Neither one decides on its own whether something passes or
ships.

| Tool | What it does | Who has the final say |
|---|---|---|
| `hwil anomaly` | Learns what nominal runs look like and flags odd stretches in new runs, naming the signals responsible | An engineer reads the flagged signals; the STL requirements still decide pass or fail |
| `hwil draft` | Has Claude draft a scenario file (mission, wind, faults, STL requirements) from a plain-language test idea | A reviewer edits and accepts the draft; nothing runs in the suite until then |

## Anomaly detection

See [anomaly.md](anomaly.md) for the method and results. In short:

- It catches every one-motor thrust loss of about 12 % or more, plus GPS
  dropouts and baro bias steps.
- It flagged none of 40 held-out clean runs.
- In 64 of 80 motor-loss runs, it caught a degraded vehicle whose every
  requirement still passed.

## LLM-drafted test cases

```bash
pip install -e ".[ai]"            # adds the anthropic SDK
export ANTHROPIC_API_KEY=...      # or: ant auth login
hwil draft "hold position through a 30 % thrust loss on motor 3 in a 5 m/s crosswind"
```

What happens:

1. **Local screen.** An idea that mentions weapons, warheads, seekers,
   targeting and similar terms is rejected before anything is sent. This is
   the project's ground rules, enforced in code.
2. **Drafting.** One request goes to `claude-opus-5-5` with structured output:
   the scenario YAML, a rationale, the assumptions made and a review checklist.
   The system prompt carries:
   - the ground rules;
   - [scenarios.md](scenarios.md) and two example scenarios;
   - the list of log and derived signals;
   - the names of existing scenarios, so the draft doesn't duplicate one.

   The request opts into server-side refusal fallbacks (`fallbacks: "default"`).
3. **Checks.** The draft must:
   - pass `hwil validate`'s validator;
   - use only known signals in its STL;
   - use fresh `REQ-DRAFT-` IDs.

   Failures go back to the model for one repair attempt. Anything still
   failing is listed in the review notes, and the command exits 1.
4. **Dry run.** The draft is flown once with the builtin controller, and each
   requirement is recorded as passed, failed or not exercised. A failure there
   means either a wrong limit or a real finding, and the reviewer decides which.
   `--no-run` skips this step.
5. **Output.** `scenarios/drafts/<name>.yaml` gets a `# DRAFT` header, next to
   `<name>.review.md`, which holds:
   - the rationale;
   - the assumptions as checkboxes;
   - the model's review checklist plus two fixed items: the requirement text
     matches its STL, and the test fits the ground rules;
   - the dry-run table.

To accept a draft, work through the checklist, delete the header and move the
file into `scenarios/`. `tests/test_drafting.py` fails if any file in
`scenarios/` still has the DRAFT header, so an unreviewed draft cannot slip
into the suite.

**Cost.** One draft is usually one request, or two if a repair is needed. The
system prompt is about 3k tokens.

**Tests.** `tests/test_drafting.py` uses a fake client, so CI needs no API key.
It covers:
- the checks;
- the repair round;
- refusal and out-of-scope handling;
- the local screen, including ordinary ideas it must let through, such as
  "bird strike" or "kill switch";
- writing the files;
- the dry run.

The live API path has not been run from the cloud session that built it,
because no key was available there.
