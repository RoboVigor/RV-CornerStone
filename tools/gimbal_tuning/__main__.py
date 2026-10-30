"""Allow `python -m gimbal_tuning` from tools without installation."""
from .cli import main

raise SystemExit(main())
