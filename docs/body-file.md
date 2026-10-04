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
  measured again, and so is every quantity measured from it (section 4); the
  others are reloaded.
- A file whose key (the size of every group, which groups take commands,
  the size of every image) is not what the robot reports belongs to another
  body: nothing is reloaded from it.
- What is reloaded stands for the session: the boot skips the stages that
  measured it, and the estimators do not measure it again. The readings'
  travel and the step responses go on growing from what was reloaded.
- Quantities are never patched by hand, and a stored number is never taken
  over a measurement that disagrees with it.

`driver/bin/replay RECORDING --body FILE` writes the body measured from a
recording, so its estimates can be scored against the truth of the run
(`driver/bin/score`).

## 3. Do not edit it

Every number in the file was measured by this robot. A number edited by hand
is a number nobody measured, which the driver's rules forbid. To measure
again, delete the file or give another path.

## 4. What it holds

A JSON object. `key` is the body's shape; `beats` how many beats measured it.
Every other member is one quantity with its `method`, listed with what it is
measured from:

| Member | What | Measured from |
|---|---|---|
| `noise` | every channel's reading noise and its degrees of freedom | the readings |
| `travel` | every channel's lowest and highest reading so far | the readings |
| `steps` | every group's longest wait for an answer, its free pushes' shortfalls | the noise |
| `lags` | every eye's image lag, and whether it is known | the noise |
| `responses` | every eye's cell noise at rest, what each channel's push does to each cell, each group's effect on the eye | the noise, the lags |
| `graph` | every group's role and arm, the arms, the carrier, every eye's mount | the responses |
| `kinematics` | every arm's fit (joints, lens, uncertainty), the table's normal | the graph, the responses |

`Driver.Robot.Load_Body` reloads a file into a model by the same rules; a
model that has seen no robot takes the file's groups and eyes, so a measured
body can be loaded and planned on with no robot connected.
