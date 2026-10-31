import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location("plot_runs",Path(__file__).parents[1]/"scripts"/"plot_runs.py")
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class PlotMetricsTests(unittest.TestCase):
    def load(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"run.csv"
            with path.open("w",newline="") as stream:
                writer=csv.writer(stream)
                writer.writerow(["connection","segment","controller","dt_s","cte_m","speed_mph","steering","compute_us"])
                writer.writerows(rows)
            return module.read_run(path)

    def test_time_weighting_and_reset_boundary(self):
        _,m=self.load([[1,0,'mpc',1,1,10,0,100],[1,0,'mpc',3,3,20,1,300],[1,1,'mpc',1,0,0,-1,200]])
        self.assertAlmostEqual(m['rms_cte_m'],(28/5)**.5)
        self.assertAlmostEqual(m['rms_steering_rate_per_s'],(1/15)**.5)
        self.assertEqual(m['segments'],2)
        self.assertAlmostEqual(m['saturation_percent'],80)
        self.assertAlmostEqual(m['mean_speed_mph'],14)
        self.assertAlmostEqual(m['mean_compute_ms'],.2)

    def test_bad_samples(self):
        for dt,e in [(0,1),(1,'nan'),(-1,0)]:
            with self.assertRaises(ValueError):
                self.load([[1,0,'pid',dt,e,10,0,10]])

    def test_empty_and_mixed_controllers(self):
        with self.assertRaises(ValueError): self.load([])
        with self.assertRaises(ValueError):
            self.load([[1,0,'pid',1,0,10,0,10],[1,0,'lqr',1,0,10,0,10]])

if __name__=='__main__': unittest.main()
