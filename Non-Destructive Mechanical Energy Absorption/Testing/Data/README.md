# MTS DAQ Data

Raw force / displacement / time exports from the MTS tensile tester. Each file has 8 preamble lines, a `"Crosshead","Load","Time"` header row, a units row, then comma-separated samples.

| File | Notes |
|---|---|
| `DAQ- Crosshead, … - (Timed)6.txt` | Run 6 |
| `DAQ- Crosshead, … - (Timed)9up.txt` | Run 9, tensile (up) leg |
| `DAQ- Crosshead, … - (Timed)9dw.csv` | Run 9, compressive (down) leg |
| `DAQ- Crosshead, … - (Timed)11.csv` | Run 11 |
| `DAQ- Crosshead, … - (Timed)12.txt` | Run 12 |
| `DAQ- Crosshead, … - (Timed)13.txt` | Run 13 |
| `DAQ- Crosshead, … - (Timed)14.txt` | Run 14 |

Columns: crosshead displacement (mm), load (N), time (s).
