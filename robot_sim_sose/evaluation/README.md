# Evaluation Data

```text
evaluation/
├── datasets/       Input bags and ground-truth sidecars
├── results/        Retained metrics, maps, databases, and plots
└── runs/           New recorder output (created on demand and Git-ignored)
```

`datasets/w1/` contains the W1 bag and `waypoints.json`. Existing W1 outputs
are grouped by artifact type under `results/w1/`.

Raw bags and RTAB-Map databases are large and ignored by ordinary Git. Do not
assume that cloning the repository transfers them; use external storage or Git
LFS and keep checksums with archived datasets. Metrics JSON/CSV, map YAML/PGM,
and final plots are small reproducible artifacts that may be committed.

See [`../docs/evaluation.md`](../docs/evaluation.md) for recording, replay, and
analysis commands.
