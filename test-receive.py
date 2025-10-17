#!/usr/bin/env python3
import ctypes
import threading
import time
import atexit

# Load WiringOP shared library
_lib = ctypes.CDLL("libwiringPi.so")  # or libwiringPi.so.2, depending on your install

# WiringOP function signatures
_lib.wiringPiSetup.argtypes = []  # int result
_lib.wiringPiSetup.restype = ctypes.c_int

_lib.pinMode.argtypes = [ctypes.c_int, ctypes.c_int]
_lib.pinMode.restype = None

_lib.wiringPiISR.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
_lib.wiringPiISR.restype = ctypes.c_int

# Constants from WiringPi / WiringOP
INPUT = 0
OUTPUT = 1
INT_EDGE_BOTH = 3  # both rising and falling

# Python-level storage for edges
edge_times = []  # list of (timestamp_ns, level) tuples
edge_lock = threading.Lock()

@ctypes.CFUNCTYPE(None, ctypes.c_int)
def _edge_callback(pin):
    now = time.monotonic_ns()
    # Read the pin level
    level = _lib.digitalRead(pin)
    with edge_lock:
        edge_times.append((now, level))
    # Return (void)

def cleanup():
    # Nothing special to clean ISR, but you may want to disable it
    pass

atexit.register(cleanup)

def capture_bursts(gpio_pin, timeout_s=0.08):
    """Use WiringOP to capture edges on gpio_pin and return bursts of durations."""
    # Setup
    if _lib.wiringPiSetup() < 0:
        raise RuntimeError("wiringPiSetup failed")
    _lib.pinMode(gpio_pin, INPUT)
    # Attach ISR on both edges
    ret = _lib.wiringPiISR(gpio_pin, INT_EDGE_BOTH, ctypes.cast(_edge_callback, ctypes.c_void_p))
    if ret != 0:
        raise RuntimeError(f"wiringPiISR failed: {ret}")

    bursts = []
    durations = []
    last_t = None
    last_level = None

    print(f"Listening on WiringOP pin {gpio_pin}, timeout {timeout_s}s")

    try:
        while True:
            # poll for edges
            time.sleep(timeout_s)
            with edge_lock:
                local = edge_times[:]
                edge_times.clear()
            if not local:
                # timeout, finish burst if any durations
                if durations:
                    bursts.append(list(durations))
                    print("Burst:", durations)
                    durations.clear()
                    last_t = None
                    last_level = None
                continue
            # Process each edge event
            for (t_ns, level) in local:
                if last_t is None:
                    last_t = t_ns
                    last_level = level
                    continue
                delta_us = (t_ns - last_t) // 1000
                durations.append(int(delta_us))
                last_t = t_ns
                last_level = level
    except KeyboardInterrupt:
        print("Stopping...")
        # flush final
        if durations:
            bursts.append(list(durations))
            print("Burst:", durations)
    return bursts

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Capture IR with WiringOP ISR")
    p.add_argument("--wiring-pin", type=int, required=True,
                   help="WiringOP pin number (not physical header) to use for IR input")
    p.add_argument("--timeout", type=float, default=0.08, help="burst timeout in seconds")
    args = p.parse_args()

    bursts = capture_bursts(args.wiring_pin, timeout_s=args.timeout)
    print(f"Captured {len(bursts)} bursts.")
