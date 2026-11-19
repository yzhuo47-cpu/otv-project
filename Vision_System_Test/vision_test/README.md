# Vision System Test — The Harvesters

Checks two things: the OTV **connects** to the vision system, and the camera **sees marker 205**.
Nothing moves (no motors, no servos). USB power is enough.

**When to use it:** only if a mission run stops after its first step (the seed is planted but the OTV doesn't back up, or it reads the first plot and stops). That means the OTV isn't connected, or the camera can't see the marker. This test shows which one.

## How to use

1. Tools → Board → **Arduino UNO R4 WiFi**, and the right Port
2. Open `vision_test.ino`, upload. **Keep USB in.** Put the OTV inside the arena (room 1201)
3. Open Tools → Serial Monitor, set **115200**. Within a few seconds you should see:
   `WiFi connected` → `WebSocket connected` → `connected, marker visible` (once per second)
4. Open **enes100.umd.edu → Student Tools → Vision System → 1201**:
   - At the bottom: **"The Harvesters has connected"**
   - In the Team list choose **The Harvesters**: the table shows **SEED, 205, YES**
   - Push the OTV: X and Y change

## What the result means

| Serial Monitor shows | Meaning | Fix |
|---|---|---|
| `connected, marker visible` | All good | Run the mission again |
| `connected, marker NOT visible` | Connected, but the camera can't see the marker | Marker flat on top, facing up, not covered by the arm or wires, no glare. Is 205 in "Seen markers" on the page? |
| `WiFi join timed out` | Can't join the Wi-Fi | Be in room 1201, move closer; ask a TA |
| `MAC is not in the credential table` | This board isn't registered | Ask a TA |
