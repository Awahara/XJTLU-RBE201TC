#!/usr/bin/env python3
"""Motor control for the RDK X5 robot (Yahboom 4-channel encoder motor driver).

Protocol: ASCII over /dev/ttyUSB0 @ 115200
  $mtype:1#              set motor type            (replies "Set mtype:OK!")
  $spd:m1,m2,m3,m4#      set speeds, -1000..1000   (the ONLY reliable stop)
  $upload:1,0,0#         DO NOT USE - see below
  $stop#                 NOT SUPPORTED on this firmware

*** WARNING: NEVER SEND $upload:1,0,0# ***
Enabling the encoder report KILLS this board's receive channel. Verified on
both the RDK and a Windows PC: after $upload:1,0,0# the board streams
$MAll:...# telemetry and, paired 1:1 with every frame, answers
"fail,Command not found!". From then on NO command is accepted - not $mtype,
not $spd, not even the stop command. $upload:0,0,0# does NOT undo it; only a
real power cycle (unplug the Type-C) restores the board.
This tool therefore NEVER enables telemetry. It only listens to telemetry
that some other program has already switched on.

Safety rules baked in:
  * Every speed command is sent as a BURST - see `burst()` for the measurements.
  * Nothing is reported as stopped that was not observed; there is no software
    confirmation available for $spd, so the tool says so plainly.
  * `check` sends NOTHING at all - use it before ever commanding motion.
  * The only guaranteed stop is cutting power to the driver board.
"""
import argparse
import collections
import re
import signal
import sys
import time

import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200
ENC_RE = re.compile(r'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')
GAP = 0.5           # seconds between commands
REPLY_WINDOW = 1.0  # seconds to wait for a reply
REPEAT_SPEED = 5    # every start/speed command is repeated this many times
REPEAT_STOP = 80    # every stop is a burst this many times (measured to work)


class MotorDriver:
    def __init__(self, port=PORT, baud=BAUD, gap=GAP, verbose=True):
        self.gap = gap
        self.verbose = verbose
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    # ---------- low level ----------
    def _say(self, msg):
        if self.verbose:
            print(msg)

    def send(self, cmd, window=REPLY_WINDOW):
        """Send $cmd# ONCE and return the board's own reply, or None if it stayed silent.

        Telemetry ($MAll...) is ignored. There is deliberately no re-send loop:
        a command that gets no acknowledgement is reported, not retried blindly.
        """
        self.ser.reset_input_buffer()
        self.ser.write(('$%s#' % cmd).encode('ascii'))
        self.ser.flush()
        junk = 0
        end = time.time() + window
        while time.time() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', 'replace').strip()
            if not line or line.startswith('$MAll'):
                continue
            if 'Command not found' in line or 'command is long' in line:
                junk += 1
                continue
            return line
        if junk:
            return 'NO-ACK (%d malformed replies from the board)' % junk
        return None

    def wait(self):
        time.sleep(self.gap)

    def burst(self, cmd, repeat=REPEAT_SPEED, gap=0.02):
        """Send $cmd# `repeat` times as fast as is sensible.

        Single speed commands are NOT reliable on this firmware. Measured:
          * ONE $spd:0,0,0,0# left a motor running for 6+ seconds.
          * ~100+ of them stopped it, three times out of three.
        So every speed command - start or stop - is sent as a burst.
        """
        payload = ('$%s#' % cmd).encode('ascii')
        for _ in range(repeat):
            self.ser.write(payload)
            self.ser.flush()
            time.sleep(gap)

    def set_speed(self, m1, m2, m3, m4, repeat=REPEAT_SPEED):
        """Set speeds, repeating the command. $spd never acknowledges."""
        s = [max(-1000, min(1000, int(v))) for v in (m1, m2, m3, m4)]
        self.burst('spd:%d,%d,%d,%d' % tuple(s), repeat=repeat)
        self.wait()
        return None

    def stop(self, repeat=REPEAT_STOP):
        """Stop every motor with a heavy burst.

        There is no way to confirm this in software: $spd does not acknowledge,
        and encoder telemetry cannot be turned on because doing so destroys the
        command channel. WATCH THE ROBOT. The only guaranteed stop is cutting
        power to the driver board.
        """
        self.burst('spd:0,0,0,0', repeat=repeat)
        self.burst('pwm:0,0,0,0', repeat=repeat // 2)
        self.wait()
        return None

    def telemetry(self, seconds=2.0):
        """Listen to the incoming stream. Sends NOTHING - see the module warning.

        Returns (encoder readings, other lines, total).

        Telemetry is OFF by default on this board and this tool will not switch
        it on, because doing so destroys the command channel. If some other
        program already enabled it, this method will see it; the fix for that is
        a power cycle, not a command.
        """
        enc = collections.OrderedDict()
        other = collections.Counter()
        total = 0
        end = time.time() + seconds
        while time.time() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', 'replace').strip()
            if not line:
                continue
            total += 1
            m = ENC_RE.search(line)
            if m:
                k = tuple(int(g) for g in m.groups())
                enc[k] = enc.get(k, 0) + 1
            else:
                other[line] += 1
        return enc, other, total

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def preflight(md):
    """Send nothing. Report whether the driver MCU looks alive and healthy."""
    print('  listening only - no data sent ...')
    enc, other, total = md.telemetry(seconds=3.0)
    junk = sum(v for k, v in other.items()
               if 'Command not found' in k or 'command is long' in k)
    print('  lines received      : %d' % total)
    print('  distinct encoder    : %d  %s' % (len(enc), list(enc.items())[:4]))
    print('  malformed-cmd reply : %d' % junk)
    print('  other lines         : %d  %s' % (sum(other.values()) - junk, other.most_common(3)))
    if junk > 0:
        print('  ** The board is streaming telemetry and rejecting its own frames.')
        print('  ** That means its receive channel is dead and NO command will')
        print('  ** reach it - including the stop command.')
        print('  ** Cause: something sent $upload:1,0,0#.')
        print('  ** Fix: power-cycle the board by unplugging the Type-C.')
        print('  ** Do NOT try to command motion now.')
        return False
    if total == 0:
        # Telemetry OFF is the NORMAL, healthy state on this board - it does not
        # auto-report after a power cycle. Absence of traffic is not a fault, so
        # this is not a failure exit. Liveness is a separate question: use `probe`.
        print('  ** no telemetry, which is NORMAL (this board does not auto-report).')
        print('  ** run `probe` to confirm the board is actually alive.')
        return True
    # A single repeated encoder value is NORMAL while the wheels are still.
    # It is NOT evidence of a hung MCU - prove the encoders by hand instead.
    print('  ** no malformed replies: link looks clean.')
    print('  ** now turn a wheel BY HAND - the encoder numbers must change.')
    return True


def liveness(md):
    """Ask the board whether it is alive, using ONLY $mtype:1#.

    $mtype:1 sets the motor type. It cannot drive a motor, and it answers with
    a clear "Set mtype:OK!".

    Note it deliberately does NOT send $upload:1,0,0# to "wake up" telemetry.
    That command permanently breaks the board's receive channel (see the module
    docstring), which is exactly the trap that caused a runaway motor here.
    """
    print('  sending $mtype:1#   (sets motor type only - cannot drive the motors)')
    r1 = md.send('mtype:1')
    print('    reply:', r1 if r1 else 'NO REPLY')
    md.wait()
    if r1 and r1.startswith('Set mtype'):
        print('  board is ALIVE and accepting commands.')
        return True
    print('  board did NOT answer.')
    print('  Either it is unpowered, or its receive channel is already broken')
    print('  (that happens if something sent $upload:1,0,0#).')
    print('  A power cycle - unplugging the Type-C - is the only fix for the latter.')
    return False


def run(md, speeds, seconds):
    print('  -> $spd:%s#  x%d  for %.2f s' % (','.join(str(s) for s in speeds),
                                               REPEAT_SPEED, seconds))
    md.set_speed(*speeds)
    time.sleep(seconds)
    print('  stopping (burst of %d) ...' % REPEAT_STOP)
    md.stop()
    print('  stop burst sent - WATCH THE ROBOT, this is not confirmed in software')


def main():
    ap = argparse.ArgumentParser(description='RDK X5 robot motor control')
    ap.add_argument('mode', choices=['check', 'probe', 'fwd', 'back', 'left', 'right',
                                     'spin_left', 'spin_right', 'raw', 'stop', 'enc'])
    ap.add_argument('values', nargs='*', type=int,
                    help='raw: m1 m2 m3 m4 [seconds]; others: [speed] [seconds]')
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--gap', type=float, default=GAP, help='seconds between commands')
    args = ap.parse_args()

    speed = args.values[0] if args.values else 300
    seconds = args.values[1] if len(args.values) > 1 else 1.5

    md = MotorDriver(port=args.port, gap=args.gap)

    def on_sigint(signum, frame):
        print('\nInterrupted - sending stop burst')
        md.stop()
        md.close()
        sys.exit(130)
    signal.signal(signal.SIGINT, on_sigint)

    try:
        mode = args.mode
        if mode == 'check':
            ok = preflight(md)
            sys.exit(0 if ok else 1)
        elif mode == 'probe':
            sys.exit(0 if liveness(md) else 1)
        elif mode == 'enc':
            for _ in range(3):
                enc, _, _ = md.telemetry(seconds=0.4)
                print('  encoders:', list(enc.items())[:1])
        elif mode == 'stop':
            md.stop()
            print('  stop burst of %d sent - WATCH THE ROBOT to confirm' % REPEAT_STOP)
        elif mode == 'raw':
            if len(args.values) < 4:
                ap.error('raw needs: m1 m2 m3 m4 [seconds]')
            secs = args.values[4] if len(args.values) > 4 else 1.5
            run(md, args.values[:4], secs)
        else:
            table = {
                'fwd':        (speed, speed, speed, speed),
                'back':       (-speed, -speed, -speed, -speed),
                'spin_left':  (-speed, -speed, speed, speed),
                'spin_right': (speed, speed, -speed, -speed),
                'left':       (-speed, speed, speed, -speed),
                'right':      (speed, -speed, -speed, speed),
            }
            run(md, table[mode], seconds)
    finally:
        md.close()


if __name__ == '__main__':
    main()
