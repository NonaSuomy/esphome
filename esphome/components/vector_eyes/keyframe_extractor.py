"""
Keyframe extraction and processing for Vector animations.
"""
import logging
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

try:
    from .animation_data import KeyframeData, AnimationData
except ImportError:
    from animation_data import KeyframeData, AnimationData


logger = logging.getLogger(__name__)


@dataclass
class ProceduralFaceData:
    """Extracted procedural face parameters."""
    trigger_time: int
    duration: int
    scale_x: float
    scale_y: float
    angle: float
    center_x: float
    center_y: float
    left_lid_top: float
    left_lid_bottom: float
    right_lid_top: float
    right_lid_bottom: float


@dataclass
class HeadAngleData:
    """Extracted head angle parameters."""
    trigger_time: int
    angle: float


@dataclass
class LiftHeightData:
    """Extracted lift height parameters."""
    trigger_time: int
    height: float


@dataclass
class RecordHeadingData:
    """Extracted record heading parameters."""
    trigger_time: int
    heading: float


class KeyframeExtractor:
    """Extracts and organizes keyframes by type from animation data."""
    
    def extract_procedural_face(self, keyframe: KeyframeData) -> Optional[ProceduralFaceData]:
        """
        Extract face animation parameters from a ProceduralFaceKeyFrame.
        
        Args:
            keyframe: KeyframeData with type "ProceduralFaceKeyFrame"
            
        Returns:
            ProceduralFaceData or None if extraction fails
        """
        if keyframe.keyframe_type != "ProceduralFaceKeyFrame":
            logger.warning(f"[WARN] KeyframeExtractor: Expected ProceduralFaceKeyFrame, got {keyframe.keyframe_type}")
            return None
        
        data = keyframe.data
        
        try:
            # Extract basic face parameters with defaults
            scale_x = float(data.get("faceScaleX", 1.0))
            scale_y = float(data.get("faceScaleY", 1.0))
            angle = float(data.get("faceAngle", 0.0))
            center_x = float(data.get("faceCenterX", 0.0))
            center_y = float(data.get("faceCenterY", 0.0))
            
            # Extract eye lid parameters
            left_eye = data.get("leftEye", [])
            right_eye = data.get("rightEye", [])
            
            # Lid parameters are at indices 5 and 6 in the eye arrays
            left_lid_top = float(left_eye[5]) if len(left_eye) > 5 else 0.6
            left_lid_bottom = float(left_eye[6]) if len(left_eye) > 6 else 0.6
            right_lid_top = float(right_eye[5]) if len(right_eye) > 5 else 0.6
            right_lid_bottom = float(right_eye[6]) if len(right_eye) > 6 else 0.6
            
            return ProceduralFaceData(
                trigger_time=keyframe.trigger_time,
                duration=0,  # Will be calculated later
                scale_x=scale_x,
                scale_y=scale_y,
                angle=angle,
                center_x=center_x,
                center_y=center_y,
                left_lid_top=left_lid_top,
                left_lid_bottom=left_lid_bottom,
                right_lid_top=right_lid_top,
                right_lid_bottom=right_lid_bottom
            )
        except (ValueError, TypeError, IndexError) as e:
            logger.warning(f"[WARN] KeyframeExtractor: Failed to extract ProceduralFace data: {e}")
            return None
    
    def extract_audio_event(self, keyframe: KeyframeData) -> Optional[List[str]]:
        """
        Extract audio event names from a RobotAudioKeyFrame.
        
        Args:
            keyframe: KeyframeData with type "RobotAudioKeyFrame"
            
        Returns:
            List of audio event names or None if extraction fails
        """
        if keyframe.keyframe_type != "RobotAudioKeyFrame":
            logger.warning(f"[WARN] KeyframeExtractor: Expected RobotAudioKeyFrame, got {keyframe.keyframe_type}")
            return None
        
        data = keyframe.data
        audio_names = data.get("audioName", [])
        
        # Handle case where audioName might be in eventGroups
        if not audio_names and "eventGroups" in data:
            for group in data["eventGroups"]:
                if isinstance(group, dict) and "audioName" in group:
                    audio_names.extend(group["audioName"])
        
        if not audio_names:
            logger.warning(f"[WARN] KeyframeExtractor: No audio names found in RobotAudioKeyFrame")
            return None
        
        # Ensure it's a list
        if not isinstance(audio_names, list):
            audio_names = [audio_names]
        
        return audio_names
    
    def extract_head_angle(self, keyframe: KeyframeData) -> Optional[HeadAngleData]:
        """
        Extract head movement data from a HeadAngleKeyFrame.
        
        Args:
            keyframe: KeyframeData with type "HeadAngleKeyFrame"
            
        Returns:
            HeadAngleData or None if extraction fails
        """
        if keyframe.keyframe_type != "HeadAngleKeyFrame":
            logger.warning(f"[WARN] KeyframeExtractor: Expected HeadAngleKeyFrame, got {keyframe.keyframe_type}")
            return None
        
        data = keyframe.data
        
        try:
            angle = float(data.get("angle_deg", 0.0))
            return HeadAngleData(
                trigger_time=keyframe.trigger_time,
                angle=angle
            )
        except (ValueError, TypeError) as e:
            logger.warning(f"[WARN] KeyframeExtractor: Failed to extract HeadAngle data: {e}")
            return None
    
    def extract_lift_height(self, keyframe: KeyframeData) -> Optional[LiftHeightData]:
        """
        Extract lift height data from a LiftHeightKeyFrame.
        
        Args:
            keyframe: KeyframeData with type "LiftHeightKeyFrame"
            
        Returns:
            LiftHeightData or None if extraction fails
        """
        if keyframe.keyframe_type != "LiftHeightKeyFrame":
            logger.warning(f"[WARN] KeyframeExtractor: Expected LiftHeightKeyFrame, got {keyframe.keyframe_type}")
            return None
        
        data = keyframe.data
        
        try:
            height = float(data.get("height_mm", 0.0))
            return LiftHeightData(
                trigger_time=keyframe.trigger_time,
                height=height
            )
        except (ValueError, TypeError) as e:
            logger.warning(f"[WARN] KeyframeExtractor: Failed to extract LiftHeight data: {e}")
            return None
    
    def extract_record_heading(self, keyframe: KeyframeData) -> Optional[RecordHeadingData]:
        """
        Extract record heading data from a RecordHeadingKeyFrame.
        
        Args:
            keyframe: KeyframeData with type "RecordHeadingKeyFrame"
            
        Returns:
            RecordHeadingData or None if extraction fails
        """
        if keyframe.keyframe_type != "RecordHeadingKeyFrame":
            logger.warning(f"[WARN] KeyframeExtractor: Expected RecordHeadingKeyFrame, got {keyframe.keyframe_type}")
            return None
        
        data = keyframe.data
        
        try:
            heading = float(data.get("heading_deg", 0.0))
            return RecordHeadingData(
                trigger_time=keyframe.trigger_time,
                heading=heading
            )
        except (ValueError, TypeError) as e:
            logger.warning(f"[WARN] KeyframeExtractor: Failed to extract RecordHeading data: {e}")
            return None
    
    def sort_by_trigger_time(self, keyframes: List[KeyframeData]) -> List[KeyframeData]:
        """
        Sort keyframes chronologically by trigger time.
        Preserves relative order for keyframes with the same trigger time.
        
        Args:
            keyframes: List of KeyframeData objects
            
        Returns:
            Sorted list of KeyframeData objects
        """
        return sorted(keyframes, key=lambda kf: kf.trigger_time)
    
    def extract_all_procedural_faces(self, animation: AnimationData) -> List[ProceduralFaceData]:
        """
        Extract all procedural face keyframes from an animation.
        
        Args:
            animation: AnimationData object
            
        Returns:
            List of ProceduralFaceData objects, sorted by trigger time
        """
        face_keyframes = animation.get_procedural_face_keyframes()
        face_keyframes = self.sort_by_trigger_time(face_keyframes)
        
        faces = []
        for kf in face_keyframes:
            face_data = self.extract_procedural_face(kf)
            if face_data:
                faces.append(face_data)
        
        # Calculate durations
        for i in range(len(faces)):
            if i < len(faces) - 1:
                faces[i].duration = faces[i + 1].trigger_time - faces[i].trigger_time
            else:
                # Last frame gets default duration
                faces[i].duration = 33  # ~30 FPS
        
        return faces
    
    def extract_all_by_type(self, animation: AnimationData) -> Dict[str, List[KeyframeData]]:
        """
        Extract and group all keyframes by type.
        
        Args:
            animation: AnimationData object
            
        Returns:
            Dictionary mapping keyframe type to list of keyframes
        """
        keyframes_by_type = {}
        
        for kf in animation.keyframes:
            if kf.keyframe_type not in keyframes_by_type:
                keyframes_by_type[kf.keyframe_type] = []
            keyframes_by_type[kf.keyframe_type].append(kf)
        
        # Sort each type by trigger time
        for kf_type in keyframes_by_type:
            keyframes_by_type[kf_type] = self.sort_by_trigger_time(keyframes_by_type[kf_type])
        
        return keyframes_by_type
