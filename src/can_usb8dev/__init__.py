"""python-can-usb8dev — pure-Python python-can backend for the 8devices USB2CAN/Korlan."""

from importlib.metadata import PackageNotFoundError, version

from .bus import Usb8DevBus, solve_bit_timing

try:
    __version__ = version("python-can-usb8dev")
except PackageNotFoundError:  # running from a source tree without install
    __version__ = "0.0.0+local"

__all__ = ["Usb8DevBus", "solve_bit_timing", "__version__"]
