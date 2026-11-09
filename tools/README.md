# tools/

Helper scripts that work on run folders. The main ones live in the `hwil`
package so they are importable from tests:

- `hwil run` / `hwil reproduce` / `hwil validate` (`hwil/cli.py`)
- plots and `report.html` (`hwil/report.py`)
- STL grading of any `log.csv` (`hwil/requirements.py`)

- `hwil suite` and `hwil montecarlo` for many runs at once (`hwil/suite.py`,
  `hwil/montecarlo.py`)

Planned here: log anomaly detection (M5).
