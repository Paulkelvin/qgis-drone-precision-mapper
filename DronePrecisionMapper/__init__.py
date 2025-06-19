# -*- coding: utf-8 -*-
"""
Drone Precision Mapper
A QGIS plugin for precise pixel-to-geocoordinate mapping from drone photos
"""

def classFactory(iface):
    """Load DronePrecisionMapper class from file DronePrecisionMapper."""
    from .drone_precision_mapper import DronePrecisionMapper
    return DronePrecisionMapper(iface) 