# gc_task.py
#
# ==============================================================================
# GCTask
# ------------------------------------------------------------------------------
# Task to handle garbage collection and memory management
# ==============================================================================

import gc
from time import sleep_ms

class GCTask:
    """Task to periodically run garbage collection."""

    def __init__(self):
        pass

    def run(self):
        while True:
            # print("Running garbage collection...")
            # print("Free memory before GC:", gc.mem_free())
            gc.collect()  # Run garbage collection
            yield # Yield control back to the scheduler