# Installed SDK EDG cache contract

This offline review applies to `vendor/jaka/lib/libjakaAPI.so` SHA-256
`64362541f964460e41495817c679894f55b1878b978528812bc8229862151fa5`.
The binary was inspected with `nm -C`, `objdump -d -C`, and `readelf -rW`;
it was not loaded or executed. Recheck this contract if that binary changes.
The existing source digest includes the vendor SDK identity.

## Decoded state and timestamp have one writer

`edg_get_stat` (0x1b10a0) calls `JAKAZuRobot::edg_get_stat` (0x19d230).
That implementation calls `JAKARobotInterface::edg_recv` (0x178370) at
0x19d24f, then `multirob_get_stat` (0x177f00) at 0x19d25e. The receiver
virtual slot +0x48 resolves through relocation 0xe3b0b0 to
`edg::EdgCli::recv` (0x246360). It drains the current transport queue and
passes the decoded feedback area at `EdgCli + 0x28` to `stat_deserialize`
(at 0x24644e–0x24648e). It returns before copying that state to the caller.
`get_stat_act_pose` (0x245ab0) copies q, v, and other fields from the same
feedback area. `get_stat_details` (0x245a00) reads only its counter at
+0x28, seconds at +0x30, and nanoseconds at +0x38. It does not receive data.

The asynchronous `TransportCli::start_recv` callback (0x249e80) stores
received bytes in the separate TransportCli queue. It does not decode or
write EdgCli's feedback cache. Decoding occurs in the explicit `recv` call.

`edg_servo_j_extend` (0x1b0f30) calls `JAKAZuRobot::edg_servo_j`
(0x19d1b0), then `multirob_servo_j` (0x177c50). Its virtual slot +0x20
resolves to `set_cmd_joint_pos` (0x245a20), which writes the separate
command area (+0x4a8 onwards); slot +0x40 resolves to `send` (0x246210).
The send body (0x246160) updates command counter +0x460 and host send
clock +0x468, calls `cmd_serialize`, then `TransportCli::send_data`
(0x248070). This path neither calls `recv` nor updates decoded feedback.
The virtual resolutions were checked against the binary relocations,
including 0xe3b088, 0xe3b0a8, and 0xe3b0b0.

JakaSystem's worker is the sole caller of get_stat/details. Safety RPCs
use the separate serialized control path. Shutdown joins that worker
before disabling EDG or destroying its handle. Under this ownership
contract, successful `get_stat` followed by `details` yields one decoded
sample and its own source timestamp. Adding another decoded-cache reader
would invalidate this pairing and is prohibited by this contract.

## Admission and evidence

The old before/get/after equality test rejected normal cache advances.
Historical mode-0 data contained 1499 such changes in 12 seconds, alongside
1497 later accepted duplicate reads. A continuous series of fresh reads
could therefore fail the old test even though source data advanced.
The failed first real hold did not record the SDK stamps or call timing,
so this defect is a confirmed code issue, not a proven full explanation
of that specific interruption.

The replacement accepts a strictly increasing source timestamp directly
after a successful state read. Duplicate timestamps never renew freshness;
backward timestamps fail. The 24 ms feedback deadline remains unchanged.
A previously expired sample or SDK read taking over 24 ms cannot be hidden
by a newly returned timestamp. Receipt is conservatively anchored to the
SDK call start. This is not synchronized cabinet-to-host latency proof.

The worker stores only the last 512 raw records in a preallocated ring.
Records include call begin/return times, return codes, source stamps, q/v,
previous accepted anchor and the admission result. No worker-loop I/O or
allocation is added. After the worker and poller join, the driver prints
summary diagnostics and writes JSONL if `V3_EDG_AUDIT_PATH` is set to an
absolute path on the project drive. Stop RPC timing is logged separately.
These timings observe one run; they do not establish a physical stop bound.

The native stub tests model the actual SDK behavior: only get_stat advances
the decoded cache, and details reads that cache. They do not load the SDK,
connect a robot, or initialize ROS. A new strict fake regression and a new
real hold are still required; this correction grants no hardware readiness.
