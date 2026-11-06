#!/usr/bin/env python3
"""Entry point also dispatched by the native rh command."""
from csi_mavlink.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
