"""Exact trial limits with roundoff-only tolerance; no runtime/flight authority."""
import math


def horizontal_velocity(value, limit=.15):
    if len(value)!=2 or not all(math.isfinite(x) for x in value):
        raise ValueError('finite horizontal velocity pair required')
    speed=math.hypot(*value)
    if not math.isfinite(limit) or limit<=0 or speed>limit+1e-12:
        raise ValueError('horizontal trial limits exceeded')
    if speed>limit:
        scale=math.nextafter(limit,0.)/speed
        return tuple(x*scale for x in value)
    return tuple(value)
