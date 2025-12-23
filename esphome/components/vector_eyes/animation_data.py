"""
Data models for Vector animation parsing and conversion.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


@dataclass
class KeyframeData:
    """Represents a single keyframe in an animation."""
    trigger_time: int  # Milliseconds from animation start
    keyframe_type: str  # "ProceduralFaceKeyFrame", "RobotAudioKeyFrame", etc.
    data: Dict[str, Any]  # Raw keyframe data from JSON
    
    def validate(self) -> bool:
        """Validate that required fields are present."""
        if self.trigger_time < 0:
            return False
        if not self.keyframe_type:
            return False
        if not self.data:
            return False
        return True


@dataclass
class AudioEventData:
    """Represents an audio event that should trigger during animation."""
    trigger_time: int  # Milliseconds from animation start
    event_names: List[str]  # Audio event names from Wwise
    wav_file: Optional[str] = None  # Set after mapping (e.g., "blink.wav")
    
    def validate(self) -> bool:
        """Validate that required fields are present."""
        if self.trigger_time < 0:
            return False
        if not self.event_names:
            return False
        return True


@dataclass
class AnimationData:
    """Complete animation data parsed from JSON."""
    name: str  # Animation identifier (e.g., "anim_keepalive_blink_01")
    keyframes: List[KeyframeData] = field(default_factory=list)
    duration_ms: int = 0  # Total animation duration
    audio_events: List[AudioEventData] = field(default_factory=list)
    
    def validate(self) -> bool:
        """Validate that required fields are present and valid."""
        if not self.name:
            return False
        if self.duration_ms < 0:
            return False
        
        # Validate all keyframes
        for kf in self.keyframes:
            if not kf.validate():
                return False
        
        # Validate all audio events
        for ae in self.audio_events:
            if not ae.validate():
                return False
        
        return True
    
    def get_procedural_face_keyframes(self) -> List[KeyframeData]:
        """Get only ProceduralFaceKeyFrame keyframes."""
        return [kf for kf in self.keyframes if kf.keyframe_type == "ProceduralFaceKeyFrame"]
    
    def get_audio_keyframes(self) -> List[KeyframeData]:
        """Get only RobotAudioKeyFrame keyframes."""
        return [kf for kf in self.keyframes if kf.keyframe_type == "RobotAudioKeyFrame"]
    
    def get_head_angle_keyframes(self) -> List[KeyframeData]:
        """Get only HeadAngleKeyFrame keyframes."""
        return [kf for kf in self.keyframes if kf.keyframe_type == "HeadAngleKeyFrame"]
    
    def get_lift_height_keyframes(self) -> List[KeyframeData]:
        """Get only LiftHeightKeyFrame keyframes."""
        return [kf for kf in self.keyframes if kf.keyframe_type == "LiftHeightKeyFrame"]
    
    def get_record_heading_keyframes(self) -> List[KeyframeData]:
        """Get only RecordHeadingKeyFrame keyframes."""
        return [kf for kf in self.keyframes if kf.keyframe_type == "RecordHeadingKeyFrame"]
