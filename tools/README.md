# Tuning GUI Setup

Install the GUI dependencies once per machine:

```powershell
python -m pip install -r tools\requirements.txt
```

Run the Qt tuning GUI:

```powershell
python tools\tune_map.py
```

If `PySide6` or `pyntcore` is missing, the GUI prints the dependency install
command and exits.
