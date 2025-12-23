"""
Property-based tests for trigger time clamping.

Tests Property 24: Trigger time clamping
Validates: Requirements 7.4
"""

import pytest
from hypothesis import given, strategies as st, assume
import logging

try:
    from .code_generator import CodeGenerator
    from .animation_data import KeyframeData, AudioEventData
    from .keyframe_extractor import ProceduralFaceData
except ImportError:
    from code_generator import CodeGenerator
    from animation_data import KeyframeData, AudioEventData
    from keyframe_extractor import ProceduralFaceData


# Strategy for generating trigger times (including negative and out-of-range)
@st.composite
def trigger_time_strategy(draw):
    """Generate trigger times including edge cases."""
    # Generate times that might be negative, zero, positive, or very large
    return draw(st.integers(min_value=-10000, max_value=100000))


# Strategy for generating animation durations
@st.composite
def duration_strategy(draw):
    """Generate realistic animation durations."""
    # Typical animations are 100ms to 10000ms
    return draw(st.integers(min_value=100, max_value=10000))


class TestTriggerTimeClamping:
    """
    Test suite for Property 24: Trigger time clamping.
    
    **Feature: vector-animation-audio-sync, Property 24: Trigger time clamping**
    
    For any keyframe with a trigger time less than zero or greater than the 
    animation duration, the system should clamp the time to the valid range 
    [0, duration].
    """
    
    def setup_method(self):
        """Set up test fixtures."""
        self.code_gen = CodeGenerator()
    
    @given(
        trigger_time=trigger_time_strategy(),
        duration=duration_strategy()
    )
    def test_clamp_trigger_time_returns_valid_range(self, trigger_time, duration):
        """
        Property: Clamped trigger time must be in valid range [0, duration].
        
        For any trigger time and duration, the clamped result must be:
        - Greater than or equal to 0
        - Less than or equal to duration
        """
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, duration)
        
        # Clamped time must be in valid range
        assert 0 <= clamped_time <= duration, \
            f"Clamped time {clamped_time} is not in range [0, {duration}] (original: {trigger_time})"
    
    @given(
        trigger_time=st.integers(min_value=0, max_value=10000),
        duration=st.integers(min_value=0, max_value=10000)
    )
    def test_valid_trigger_times_unchanged(self, trigger_time, duration):
        """
        Property: Valid trigger times should not be clamped.
        
        If trigger time is already in the valid range [0, duration], it should
        be returned unchanged and was_clamped should be False.
        """
        assume(0 <= trigger_time <= duration)
        
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, duration)
        
        # Should not be clamped
        assert not was_clamped, \
            f"Valid trigger time {trigger_time} was incorrectly marked as clamped"
        
        # Should be unchanged
        assert clamped_time == trigger_time, \
            f"Valid trigger time {trigger_time} was changed to {clamped_time}"
    
    @given(
        trigger_time=st.integers(min_value=-10000, max_value=-1),
        duration=duration_strategy()
    )
    def test_negative_trigger_times_clamped_to_zero(self, trigger_time, duration):
        """
        Property: Negative trigger times must be clamped to 0.
        
        Any trigger time less than 0 should be clamped to 0, and was_clamped
        should be True.
        """
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, duration)
        
        # Should be clamped to 0
        assert clamped_time == 0, \
            f"Negative trigger time {trigger_time} was clamped to {clamped_time} instead of 0"
        
        # Should be marked as clamped
        assert was_clamped, \
            f"Negative trigger time {trigger_time} was not marked as clamped"
    
    @given(
        duration=duration_strategy(),
        excess=st.integers(min_value=1, max_value=10000)
    )
    def test_excessive_trigger_times_clamped_to_duration(self, duration, excess):
        """
        Property: Trigger times exceeding duration must be clamped to duration.
        
        Any trigger time greater than duration should be clamped to duration,
        and was_clamped should be True.
        """
        trigger_time = duration + excess
        
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, duration)
        
        # Should be clamped to duration
        assert clamped_time == duration, \
            f"Excessive trigger time {trigger_time} was clamped to {clamped_time} instead of {duration}"
        
        # Should be marked as clamped
        assert was_clamped, \
            f"Excessive trigger time {trigger_time} was not marked as clamped"
    
    @given(
        trigger_time=trigger_time_strategy(),
        duration=duration_strategy()
    )
    def test_clamping_is_idempotent(self, trigger_time, duration):
        """
        Property: Clamping is idempotent.
        
        Clamping a trigger time twice should produce the same result as
        clamping it once.
        """
        clamped_once, _ = self.code_gen.clamp_trigger_time(trigger_time, duration)
        clamped_twice, _ = self.code_gen.clamp_trigger_time(clamped_once, duration)
        
        assert clamped_once == clamped_twice, \
            f"Clamping is not idempotent: {trigger_time} -> {clamped_once} -> {clamped_twice}"
    
    @given(
        trigger_time=trigger_time_strategy(),
        duration=duration_strategy()
    )
    def test_was_clamped_flag_accuracy(self, trigger_time, duration):
        """
        Property: was_clamped flag must accurately reflect whether clamping occurred.
        
        The was_clamped flag should be True if and only if the trigger time
        was outside the valid range [0, duration].
        """
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, duration)
        
        # Check if clamping was needed
        needed_clamping = (trigger_time < 0) or (trigger_time > duration)
        
        assert was_clamped == needed_clamping, \
            f"was_clamped flag is incorrect: trigger_time={trigger_time}, duration={duration}, " \
            f"was_clamped={was_clamped}, needed_clamping={needed_clamping}"
    
    def test_edge_case_zero_duration(self):
        """
        Test edge case: zero duration animation.
        
        All trigger times should be clamped to 0 for zero-duration animations.
        """
        test_cases = [-100, -1, 0, 1, 100]
        
        for trigger_time in test_cases:
            clamped_time, was_clamped = self.code_gen.clamp_trigger_time(trigger_time, 0)
            
            assert clamped_time == 0, \
                f"Trigger time {trigger_time} was not clamped to 0 for zero-duration animation"
            
            if trigger_time != 0:
                assert was_clamped, \
                    f"Non-zero trigger time {trigger_time} was not marked as clamped for zero-duration animation"
    
    def test_edge_case_boundary_values(self):
        """
        Test edge case: boundary values.
        
        Test exact boundary values (0 and duration) to ensure they're not clamped.
        """
        duration = 5000
        
        # Test 0 (lower boundary)
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(0, duration)
        assert clamped_time == 0
        assert not was_clamped
        
        # Test duration (upper boundary)
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(duration, duration)
        assert clamped_time == duration
        assert not was_clamped
        
        # Test just below 0
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(-1, duration)
        assert clamped_time == 0
        assert was_clamped
        
        # Test just above duration
        clamped_time, was_clamped = self.code_gen.clamp_trigger_time(duration + 1, duration)
        assert clamped_time == duration
        assert was_clamped
    
    def test_logging_occurs_when_clamping(self, caplog):
        """
        Test that logging occurs when clamping happens.
        
        The system should log warnings when trigger times are clamped.
        """
        with caplog.at_level(logging.WARNING):
            # Test negative clamping
            self.code_gen.clamp_trigger_time(-100, 5000)
            assert any("Clamping negative trigger time" in record.message for record in caplog.records)
            
            caplog.clear()
            
            # Test excessive clamping
            self.code_gen.clamp_trigger_time(10000, 5000)
            assert any("Clamping trigger time" in record.message for record in caplog.records)


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "--tb=short"])
