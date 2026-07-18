#!/usr/bin/env python3

from typing import Literal, Optional

from pydantic import BaseModel, Field


class CameraSettingsRequest(BaseModel):
    enabled: Optional[bool] = None
    mode: Optional[Literal["photo", "video"]] = None
    quality: Optional[int] = Field(None, ge=1, le=100)
