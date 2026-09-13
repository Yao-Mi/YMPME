#!/usr/bin/env python3
"""Build all PME functionality into one relocatable libympme.a SDK."""
from build_modular import main

if __name__ == "__main__":
    main(unified=True)
