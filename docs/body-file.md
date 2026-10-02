# The body file

At boot the driver measures the body: how each group responds, how much its
readings shake at rest, the kinematics of each arm, each eye's lens and where
it is mounted, the shapes of the links, and each hand. What it measured can
be kept in a file that travels with this robot, so the next boot does not
start from zero.

The file holds the body only. Nothing about the world, the task or the
things on the table is kept in it.

## 1. Use

```
body_driver --listen 9080 --body body.dat     # reload from body.dat if it is there, store into it
body_driver --listen 9080                     # measure from zero and keep nothing
```

With `--body`, the boot reloads the file when it exists and writes what it
measures back into the same file as it measures. Delete the file to measure
from zero again.

## 2. What a reload does

Every quantity is stored with its uncertainty and the version of the method
that measured it.

- A quantity whose measuring method has changed since it was stored is
  measured again; the others are reloaded.
- The reloaded body is checked against what the robot shows now (a push and
  a look). A quantity the robot contradicts is measured again; one it agrees
  with is kept.
- Quantities are never patched by hand, and a stored number is never taken
  over a measurement that disagrees with it.

`driver/bin/replay RECORDING --body FILE` writes the body measured from a
recording, so its estimates can be scored against the truth of the run
(`driver/bin/score`).

## 3. Do not edit it

Every number in the file was measured by this robot. A number edited by hand
is a number nobody measured, which the driver's rules forbid. To measure
again, delete the file or give another path.

The layout of the file is the boot's (driver/src/robot, Driver.Robot.Boot);
it is described here once that layer lands on main.
