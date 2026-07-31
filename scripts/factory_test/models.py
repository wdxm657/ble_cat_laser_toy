# -*- coding: utf-8 -*-
"""Shared data models for factory test."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class ScanDevice:
    name: str
    address: str
    rssi: Optional[int]
    manufacturer_data: str
