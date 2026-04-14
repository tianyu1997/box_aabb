"""Monitor S3/S4 experiment progress."""
import time, sys, os

LOG = os.path.join(os.environ.get("TEMP", "/tmp"), "sbf5_experiments", "s3s4_log.txt")
STDERR = os.path.join(os.environ.get("TEMP", "/tmp"), "sbf5_experiments", "run_s3s4_stderr.txt")

start = time.time()
while time.time() - start < 14400:  # 4 hour max
    try:
        with open(LOG, "r", encoding="utf-8") as f:
            log = f.read()
        if "ALL_DONE" in log:
            print("ALL_DONE detected!")
            print(log)
            break
        with open(STDERR, "r", encoding="utf-8") as f:
            lines = f.readlines()
        trial_lines = [l for l in lines if "/1080]" in l or "/240]" in l]
        if trial_lines:
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] {trial_lines[-1].strip()}")
        if "DONE" in log and "S4" in log:
            print("S4 DONE in log")
            print(log)
            break
    except Exception as e:
        print(f"Error: {e}")
    time.sleep(300)  # check every 5 min

elapsed = (time.time() - start) / 60
print(f"Elapsed: {elapsed:.0f} min")
