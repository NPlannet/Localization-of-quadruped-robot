# Evaluation Data

```text
evaluation/
├── datasets/   Input sidecars and externally stored bags
├── runs/       New recordings and temporary benchmark runs; Git-ignored
├── results/    Retained metrics, maps, and plots
└── templates/  Benchmark matrix configuration examples
```

Raw MCAP bags and RTAB-Map databases are too large for ordinary Git. Archive
them externally with a checksum. Small, presentation-relevant metrics, maps,
and plots may be committed under `results/`.

See [the evaluation guide](../docs/evaluation.md) for the workflow.
