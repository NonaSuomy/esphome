"""
Validation and reporting for animation conversion.

This module provides validation of converted animations against original data
and generates human-readable validation reports.
"""
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass

try:
    from .animation_data import AnimationData
    from .keyframe_extractor import ProceduralFaceData
except ImportError:
    from animation_data import AnimationData
    from keyframe_extractor import ProceduralFaceData


logger = logging.getLogger(__name__)


@dataclass
class ValidationReport:
    """Results of animation validation."""
    animation_name: str
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    duration_match: bool
    keyframe_counts: Dict[str, int]
    audio_event_count: int


class ValidationReporter:
    """
    Validates converted animations and generates reports.
    
    Performs validation checks:
    - Duration preservation
    - Keyframe count comparison
    - Audio assignment validation
    """
    
    def __init__(self):
        """Initialize the validation reporter."""
        self.reports: List[ValidationReport] = []
    
    def validate_conversion(
        self,
        original: AnimationData,
        converted_keyframes: List[ProceduralFaceData],
        keyframe_sound_map: Dict[int, str]
    ) -> ValidationReport:
        """
        Compare original and converted animation data.
        
        Args:
            original: Original AnimationData from JSON
            converted_keyframes: List of converted ProceduralFaceData
            keyframe_sound_map: Mapping of keyframe index to sound filename
            
        Returns:
            ValidationReport with validation results
        """
        errors = []
        warnings = []
        
        # Check duration match
        duration_match = self.check_duration_match(original, converted_keyframes)
        if not duration_match:
            errors.append(
                f"Duration mismatch: original {original.duration_ms}ms, "
                f"converted {self._calculate_converted_duration(converted_keyframes)}ms"
            )
        
        # Check keyframe counts
        keyframe_counts = self.check_keyframe_counts(original)
        converted_count = len(converted_keyframes)
        original_face_count = len(original.get_procedural_face_keyframes())
        
        if converted_count != original_face_count:
            warnings.append(
                f"Keyframe count mismatch: original {original_face_count} ProceduralFace keyframes, "
                f"converted {converted_count} keyframes"
            )
        
        # Check audio assignment
        audio_issues = self.check_audio_assignment(original, keyframe_sound_map)
        errors.extend(audio_issues)
        
        # Count audio events
        audio_event_count = len(original.audio_events)
        
        # Create report
        is_valid = len(errors) == 0
        report = ValidationReport(
            animation_name=original.name,
            is_valid=is_valid,
            errors=errors,
            warnings=warnings,
            duration_match=duration_match,
            keyframe_counts=keyframe_counts,
            audio_event_count=audio_event_count
        )
        
        self.reports.append(report)
        return report
    
    def check_duration_match(
        self,
        original: AnimationData,
        converted_keyframes: List[ProceduralFaceData]
    ) -> bool:
        """
        Verify that total duration matches the original JSON.
        
        Args:
            original: Original AnimationData
            converted_keyframes: List of converted keyframes
            
        Returns:
            True if durations match (within 1ms tolerance)
        """
        original_duration = original.duration_ms
        converted_duration = self._calculate_converted_duration(converted_keyframes)
        
        # Allow 1ms tolerance for rounding
        return abs(original_duration - converted_duration) <= 1
    
    def _calculate_converted_duration(
        self,
        keyframes: List[ProceduralFaceData]
    ) -> int:
        """
        Calculate total duration from converted keyframes.
        
        Args:
            keyframes: List of ProceduralFaceData
            
        Returns:
            Total duration in milliseconds
        """
        if not keyframes:
            return 0
        
        # Duration is the trigger time of the last keyframe plus its duration
        last_keyframe = keyframes[-1]
        return last_keyframe.trigger_time + last_keyframe.duration
    
    def check_keyframe_counts(self, original: AnimationData) -> Dict[str, int]:
        """
        Count keyframes by type in the original animation.
        
        Args:
            original: Original AnimationData
            
        Returns:
            Dictionary mapping keyframe type to count
        """
        counts = {}
        
        for kf in original.keyframes:
            kf_type = kf.keyframe_type
            counts[kf_type] = counts.get(kf_type, 0) + 1
        
        return counts
    
    def check_audio_assignment(
        self,
        original: AnimationData,
        keyframe_sound_map: Dict[int, str]
    ) -> List[str]:
        """
        Find unassigned or duplicate audio events.
        
        Args:
            original: Original AnimationData
            keyframe_sound_map: Mapping of keyframe index to sound filename
            
        Returns:
            List of error messages
        """
        errors = []
        
        # Check for duplicate audio assignments (same sound assigned to multiple keyframes)
        sound_to_keyframes = {}
        for kf_idx, sound in keyframe_sound_map.items():
            if sound not in sound_to_keyframes:
                sound_to_keyframes[sound] = []
            sound_to_keyframes[sound].append(kf_idx)
        
        for sound, keyframes in sound_to_keyframes.items():
            if len(keyframes) > 1:
                errors.append(
                    f"Audio file '{sound}' assigned to multiple keyframes: {keyframes}"
                )
        
        # Check if all audio events were assigned
        assigned_count = len(keyframe_sound_map)
        total_audio_events = len(original.audio_events)
        
        if assigned_count < total_audio_events:
            unassigned_count = total_audio_events - assigned_count
            # This is a warning, not an error (some audio events may be beyond tolerance)
            logger.info(
                f"[INFO] ValidationReporter: {unassigned_count} of {total_audio_events} "
                f"audio events were not assigned to keyframes"
            )
        
        return errors
    
    def generate_report(self, report: ValidationReport) -> str:
        """
        Create human-readable validation report.
        
        Args:
            report: ValidationReport to format
            
        Returns:
            Formatted report string
        """
        lines = []
        lines.append(f"=== Validation Report: {report.animation_name} ===")
        lines.append(f"Status: {'VALID' if report.is_valid else 'INVALID'}")
        lines.append("")
        
        # Duration
        lines.append(f"Duration Match: {'YES' if report.duration_match else 'NO'}")
        
        # Keyframe counts
        lines.append("Keyframe Counts by Type:")
        for kf_type, count in sorted(report.keyframe_counts.items()):
            lines.append(f"  {kf_type}: {count}")
        
        # Audio events
        lines.append(f"Audio Events: {report.audio_event_count}")
        
        # Errors
        if report.errors:
            lines.append("")
            lines.append("Errors:")
            for error in report.errors:
                lines.append(f"  - {error}")
        
        # Warnings
        if report.warnings:
            lines.append("")
            lines.append("Warnings:")
            for warning in report.warnings:
                lines.append(f"  - {warning}")
        
        lines.append("=" * 50)
        
        return "\n".join(lines)
    
    def generate_summary_report(self) -> str:
        """
        Generate a summary report for all validated animations.
        
        Returns:
            Formatted summary report string
        """
        if not self.reports:
            return "No animations validated."
        
        lines = []
        lines.append("=== Validation Summary ===")
        lines.append(f"Total Animations: {len(self.reports)}")
        
        valid_count = sum(1 for r in self.reports if r.is_valid)
        invalid_count = len(self.reports) - valid_count
        
        lines.append(f"Valid: {valid_count}")
        lines.append(f"Invalid: {invalid_count}")
        lines.append("")
        
        # List invalid animations
        if invalid_count > 0:
            lines.append("Invalid Animations:")
            for report in self.reports:
                if not report.is_valid:
                    lines.append(f"  - {report.animation_name}")
                    for error in report.errors:
                        lines.append(f"      {error}")
        
        lines.append("=" * 50)
        
        return "\n".join(lines)
