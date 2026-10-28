# Current UART bug analysis

## Confirmed observations

- The Work Queue version opens the UART with `O_RDWR`; it is not read-only.
- If `open()`, `tcgetattr()`, or `tcsetattr()` fails, startup returns failure.
- `/dev/ttyS3` exists and NSH can execute `echo test > /dev/ttyS3`.
- The Work Queue module reported repeated write `errno=13` (`EACCES`) on
  `/dev/ttyS3` and `/dev/ttyS1`.
- `/dev/ttyS4` produced read/write `errno=9` (`EBADF`).
- No connected gimbal explains `RX=0`, but cannot explain UART write failure.
- The Work Queue implementation calls `serial_.open()` from `init()` during
  the start command, then calls `read()` and `write()` from `Run()` on the
  shared Work Queue.
- Its current `write_all()` can block inside a shared Work Queue.

## What the logs do not prove by themselves

The logs alone do not prove that task-group file descriptor context is the
only cause. Port ownership or a board-specific UART behavior can produce
similar error numbers. The former single `last_error` also obscured which
operation produced an error.

## Why this Task variant is the chosen diagnostic and production fix

This variant makes these invariants explicit:

1. `instantiate()`, `open()`, `poll()`, `read()`, `write()`, `close()`, and
   object destruction all execute in the independent module Task.
2. Open, read, write, and poll errors have separate counters and errno values.
3. I/O is non-blocking and `poll()` has a maximum 20 ms timeout.
4. A fixed 44-byte buffer preserves partial-write progress.
5. A new frame is never inserted before the previous frame is complete.
6. The Task UART fix was isolated from protocol changes. The later telemetry
   extension adds an explicit flight-data packer with byte-level host tests;
   the validated feedback decoder remains unchanged.

If this version transmits normally on `/dev/ttyS3`, it strongly supports the
old execution-context/lifecycle diagnosis. If it still fails, the separated
diagnostics identify whether the failure occurs in open, poll, read, or write,
and investigation should then focus on port ownership and board UART setup.

