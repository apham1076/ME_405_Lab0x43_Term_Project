# gc_task.py
#
# ==============================================================================
# GCTask
# ------------------------------------------------------------------------------
# Task to handle garbage collection and memory management
# ==============================================================================

import gc

class GCTask:
    """Task to periodically run garbage collection."""

    def __init__(self):
        pass

    def run(self):
        while True:
            gc.collect()  # Run garbage collection
            yield  # Yield control back to the scheduler