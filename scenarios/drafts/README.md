# scenarios/drafts/

Scenarios drafted by `hwil draft` land here, each with a `<name>.review.md`.
Nothing in this folder runs in the suite or CI.

To accept a draft:

1. Work through its review checklist.
2. Remove the `# DRAFT` header lines.
3. Move the file into `scenarios/`.

`tests/test_drafting.py` fails if a file in `scenarios/` still has the header.
See [docs/ai-in-test.md](../../docs/ai-in-test.md).
