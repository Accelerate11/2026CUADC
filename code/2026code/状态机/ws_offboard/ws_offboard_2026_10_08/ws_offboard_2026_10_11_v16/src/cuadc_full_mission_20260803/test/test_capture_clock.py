import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from capture_clock import CaptureClock, exposure_midpoint_ns

RS = NS(timestamp_domain=NS(global_time='global'), frame_metadata_value=NS(
    sensor_timestamp='sensor', frame_timestamp='frame', actual_exposure='exposure'))
BASE = 1_790_000_000_000_000_000


class Frame:
    def __init__(self, stamp=BASE, sensor=99000, frame=100000, domain='global'):
        self.stamp, self.domain = stamp, domain
        self.md = {'sensor': sensor, 'frame': frame}
    def get_timestamp(self): return self.stamp / 1e6
    def get_frame_timestamp_domain(self): return self.domain
    def supports_frame_metadata(self, key): return key in self.md
    def get_frame_metadata(self, key): return self.md[key]


def frames(color=None, depth=None):
    return NS(get_color_frame=lambda: color or Frame(), get_depth_frame=lambda: depth or Frame())


class TestClock(unittest.TestCase):
    def read(self, clock, fs=None, host=BASE+30_000_000, ros=None, mono=1_000_000_000):
        return clock.read(fs or frames(), RS, host, host if ros is None else ros, mono)
    def test_midpoint_not_arrival(self):
        stamp, values = self.read(CaptureClock())
        self.assertEqual(stamp, BASE-1_000_000)
        self.assertEqual(values['color_acquisition_age_ms'], 31)
        self.assertIsNone(values['color_actual_exposure'])
    def test_wrap(self):
        self.assertEqual(exposure_midpoint_ns(BASE/1e6, 2**32-500, 500), BASE-1_000_000)
    def test_reject_domain_missing_metadata_skew(self):
        bad = Frame(); del bad.md['sensor']
        for fs in [frames(color=Frame(domain='hardware')), frames(color=bad),
                   frames(depth=Frame(stamp=BASE-30_000_000))]:
            with self.assertRaises(ValueError): self.read(CaptureClock(), fs)
    def test_reject_stale_future_sim_time(self):
        for host in (BASE+300_000_000, BASE-20_000_000):
            with self.assertRaises(ValueError): self.read(CaptureClock(), host=host)
        with self.assertRaises(ValueError): self.read(CaptureClock(), ros=10)
    def test_reject_duplicate_and_clock_step(self):
        clock = CaptureClock(); self.read(clock)
        with self.assertRaises(ValueError): self.read(clock)
        with self.assertRaises(ValueError):
            self.read(clock, frames(Frame(stamp=BASE+100_000_000),Frame(stamp=BASE+100_000_000)),
                      host=BASE+130_000_000, mono=1_010_000_000)
    def test_variable_processing_delay_does_not_change_stamp(self):
        for delay in (20_000_000, 100_000_000, 200_000_000):
            stamp, _ = self.read(CaptureClock(), host=BASE+delay)
            self.assertEqual(stamp, BASE-1_000_000)


if __name__ == '__main__': unittest.main()
