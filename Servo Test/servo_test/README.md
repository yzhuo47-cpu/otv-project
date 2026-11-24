# Servo Test — The Harvesters

Finds the real servo angles **without USB**: only the battery is on. The servo turns slowly by itself, and the current angle is shown on the **vision system page**. You watch the OTV and write down the number at the right moment.

Only Wi-Fi is needed (the page must say "The Harvesters has connected"). The marker doesn't need to be visible.

---

## How to run (same every time)

1. Power switch **OFF**, plug in USB, set `TEST` at the top, click **Upload**
2. **Unplug USB**
3. Open **enes100.umd.edu → Student Tools → Vision System → 1201**, choose **The Harvesters**
4. Power **ON**. When the page says **"Connected. Test starts in 10 s"**, you have 10 s to put the OTV in place
5. Watch the OTV and the page. Write down the numbers you need
6. At the end it goes back to the start and prints **"Done"**. Press **RESET** to run again

**If a servo buzzes, shakes or gets warm: power OFF right away.** It is pushing on something.

Tip: film the OTV and the laptop screen together with a phone, so you can check the numbers later. The page can show the number about 1 step late.

---

## TEST 1 — Arm (`TEST = 1`)

Put the OTV at the **front** of the site, right side facing it, bucket above the **near-left plot (orzo)**.

The arm starts at **90** and goes **down** 1° every 2 s, to **60** (about 60 s).

**Write down:** the number when the bucket **just touches** the orzo (touching, not pressing in). Once you have it, you can power off.

If the arm goes **up** instead of down, power off and tell Jonathan.

---

## TEST 2 — Bucket (`TEST = 2`)

First put the touch angle from TEST 1 into `ARM_FIXED` at the top, then upload.

Same place as TEST 1. The arm stays at the touch angle. The bucket turns from 30 to 150, 3° every 1.5 s (about 60 s). Don't put a bean in.

**Write down two numbers:**
- **Level:** the bucket is flat, so a bean would stay in
- **Scoop:** the bucket is turned so it digs into the orzo (the side that collects orzo goes down)

---

## Put the numbers into the mission code

At the top of `mission.ino`:

| Setting | = |
|---|---|
| `ARM_UP` | touch + 15 |
| `ARM_DOWN` | touch − 12 |
| `BUCKET_HOLD` | level |
| `BUCKET_SCOOP` | scoop |
| `BUCKET_DUMP` | 2 × level − scoop |

Example: touch 88, level 92, scoop 145 → `ARM_UP = 103`, `ARM_DOWN = 76`, `BUCKET_HOLD = 92`, `BUCKET_SCOOP = 145`, `BUCKET_DUMP = 39`.

(Bigger number = arm up on our OTV, so up is touch + 15 and down is touch − 12.)
