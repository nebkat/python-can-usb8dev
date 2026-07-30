"""Standalone recorder / test harness: ``python -m can_usb8dev`` (or ``usb8dev-record``).

Useful to confirm an adapter works independently of Wireshark or any extcap.
"""

import argparse
import logging
import sys
import time

from .bus import Usb8DevBus


def main(argv=None):
    p = argparse.ArgumentParser(
        prog="usb8dev-record",
        description="Record CAN from an 8devices USB2CAN/Korlan in pure Python",
    )
    p.add_argument("--list", action="store_true", help="list connected adapters and exit")
    p.add_argument("-c", "--channel", help="device serial (default: first found)")
    p.add_argument("-b", "--bitrate", type=int, default=250000)
    p.add_argument("--listen-only", action="store_true", help="silent / listen-only mode")
    p.add_argument("--loopback", action="store_true", help="internal loopback, no bus needed")
    p.add_argument("-o", "--output",
                   help="log file (extension picks format: .asc .blf .csv .log)")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    if args.list:
        found = Usb8DevBus._detect_available_configs()
        if not found:
            print("No USB2CAN devices found.")
        for c in found:
            print(f"  {c['channel']}")
        return 0

    import can

    with Usb8DevBus(channel=args.channel, bitrate=args.bitrate,
                    listen_only=args.listen_only, loopback=args.loopback) as bus:
        listeners = [can.Printer()]
        if args.output:
            listeners.append(can.Logger(args.output))
        notifier = can.Notifier(bus, listeners)
        print(f"Recording from {bus.channel_info} @ {args.bitrate} bit/s. Ctrl+C to stop.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            notifier.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
