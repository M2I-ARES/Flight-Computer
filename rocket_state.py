import math
from dataclasses import dataclass, field
from typing import List

@dataclass
class RocketState:
    # Positioning
    pos:          List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # ECEF m
    vel:          List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m/s

    # Orientation
    mrp:          List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # Modified Rodriguez Params
    ang_rate:     List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # rad/s
    mag:          List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # normalised

    # Sensors
    accel:        List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m/s²
    accel_bias:   List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m/s²
    gyro_bias:    List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # rad/s

    # Uncertainty
    pos_std:      List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m
    vel_std:      List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # m/s
    heading_err:  float = 0.0                                                    # rad

    # Controls
    time:             float = 0.0
    drogue_deploy:    bool  = False
    main_deploy:      bool  = False
    drogue_desired:   bool  = False
    main_desired:     bool  = False
    drogue_override:  bool  = False
    main_override:    bool  = False

    def copy(self):
        import copy
        return copy.deepcopy(self)

    def ecef_to_geodetic(self):
        """Convert ECEF pos to (lat_deg, lon_deg, alt_m) using WGS-84."""
        x, y, z = self.pos
        if x == 0.0 and y == 0.0 and z == 0.0:
            return None, None, None
        a  = 6378137.0
        e2 = 6.69437999014e-3
        lon = math.degrees(math.atan2(y, x))
        p   = math.sqrt(x*x + y*y)
        lat = math.atan2(z, p * (1 - e2))
        for _ in range(5):
            N   = a / math.sqrt(1 - e2 * math.sin(lat)**2)
            lat = math.atan2(z + e2 * N * math.sin(lat), p)
        N   = a / math.sqrt(1 - e2 * math.sin(lat)**2)
        alt = p / math.cos(lat) - N if abs(math.cos(lat)) > 1e-10 else abs(z) / math.sin(lat) - N * (1 - e2)
        return math.degrees(lat), lon, alt

    def mrp_to_quaternion(self):
        """Convert MRP to quaternion [w, x, y, z]."""
        mx, my, mz = self.mrp
        n2 = mx*mx + my*my + mz*mz
        w  = (1 - n2) / (1 + n2)
        s  = 2.0 / (1 + n2)
        return [round(w, 5), round(mx*s, 5), round(my*s, 5), round(mz*s, 5)]

    def to_dict(self):
        """Serialise to a dict ready for Socket.IO emission."""
        lat, lon, alt = self.ecef_to_geodetic()
        quat = self.mrp_to_quaternion()
        return {
            "position":    [round(v, 3) for v in self.pos],
            "velocity":    [round(v, 3) for v in self.vel],
            "acceleration":[round(v, 3) for v in self.accel],
            "accel_bias":  [round(v, 4) for v in self.accel_bias],
            "gyro_bias":   [round(v, 5) for v in self.gyro_bias],
            "mrp":         [round(v, 5) for v in self.mrp],
            "quaternion":  quat,
            "ang_rate":    [round(math.degrees(v), 3) for v in self.ang_rate],  # → deg/s
            "magnetometer":[round(v, 4) for v in self.mag],
            "pos_std":     [round(v, 3) for v in self.pos_std],
            "vel_std":     [round(v, 3) for v in self.vel_std],
            "heading_err": round(math.degrees(self.heading_err), 3),             # → deg
            "lat":         round(lat, 7) if lat is not None else None,
            "lon":         round(lon, 7) if lon is not None else None,
            "altitude":    round(alt,  2) if alt is not None else None,
            "time":        round(self.time, 2),
            "controls": {
                "drogue_deploy":   self.drogue_deploy,
                "main_deploy":     self.main_deploy,
                "drogue_desired":  self.drogue_desired,
                "main_desired":    self.main_desired,
                "drogue_override": self.drogue_override,
                "main_override":   self.main_override,
            }
        }
