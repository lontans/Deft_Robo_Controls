# Lab / controls notebooks

Import **`deft_controls_sdk`** — not `pcb_lab` helpers. See [`../deft_controls_sdk/README.md`](../deft_controls_sdk/README.md) quick start.

| Notebook | Use |
|----------|-----|
| [`closed_loop_smoke.ipynb`](closed_loop_smoke.ipynb) | Command→FB prove on one slot (`proxy.actions` mount/apply/clear) |
| [`build_assembly.ipynb`](build_assembly.ipynb) | Construct `Assembly` / apply CFG / move |

Open from Cursor/VS Code or Jupyter with cwd under `scripts/` (or run the path bootstrap cell). Needs the same env as `pcb_lab` (`pip install -r requirements.txt`).

Live cells open the COM port — only one owner at a time (close `pcb_lab.debug test` / dashboard first).
