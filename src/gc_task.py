"""
@file gc_task.py
@brief Task to periodically run garbage collection.

This module defines the GCTask class, which periodically invokes the garbage collector to manage memory usage in the system.
"""

import gc

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