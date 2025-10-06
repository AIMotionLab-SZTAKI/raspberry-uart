import matplotlib.pyplot as plt
import json

log = []

with open("pwm_log.jsonl", "r") as f:
    for line in f:
        entry = json.loads(line)
        log.append(entry)

plt.figure()
plt.plot(log)
plt.savefig("fig.png")