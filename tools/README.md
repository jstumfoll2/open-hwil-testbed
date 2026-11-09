# tools/

Helper scripts that work on run folders. The main ones live in the `hwil`
package so they are importable from tests:

- `hwil run` / `hwil reproduce` / `hwil validate` (`hwil/cli.py`)
- plots and `report.html` (`hwil/report.py`)
- STL grading of any `log.csv` (`hwil/requirements.py`)

Planned here: batch summaries across many runs (M4), and log anomaly detection
(M6).
