from __future__ import annotations

import asyncio
import math
from zr10_bridge import Detection, ZR10Bridge, backproject_ground_pixel


async def main() -> None:
    bridge = ZR10Bridge(simulate=True)
    await bridge.connect()
    state = await bridge.set_attitude(10, -60)
    state = await bridge.set_zoom(5)
    assert state.yaw_deg == 10 and state.pitch_deg == -60 and state.zoom == 5
    tracked = await bridge.track_detection(Detection('person', .9, 1000, 500, 1040, 560, 0), (1920, 1080))
    assert tracked.yaw_deg > 10 and tracked.pitch_deg <= -60
    observation = backproject_ground_pixel(
        pixel=(960, 540), image_size=(1920, 1080), vehicle_enu_m=(100, 50, 40),
        vehicle_yaw_deg=0, gimbal_yaw_deg=0, gimbal_pitch_deg=-90)
    assert math.isclose(observation.east_m, 100, abs_tol=1e-8)
    assert math.isclose(observation.north_m, 50, abs_tol=1e-8)
    assert math.isclose(observation.slant_range_m, 40, abs_tol=1e-8)
    await bridge.close()
    print('zr10 bridge validation passed')


if __name__ == '__main__':
    asyncio.run(main())
