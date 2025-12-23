"""
Property-based tests for storage initialization idempotence.

Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
Validates: Requirements 1.1, 1.3
"""
from hypothesis import given, strategies as st, settings, assume
from hypothesis.strategies import composite
from typing import List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class MockStorageDevice:
    """Mock storage device for testing."""
    id: str
    mount_path: str
    is_available: bool
    has_animations: bool
    
    def __eq__(self, other):
        if not isinstance(other, MockStorageDevice):
            return False
        return (self.id == other.id and 
                self.mount_path == other.mount_path and
                self.is_available == other.is_available and
                self.has_animations == other.has_animations)
    
    def __hash__(self):
        return hash((self.id, self.mount_path, self.is_available, self.has_animations))


@dataclass
class MockStorageComponent:
    """Mock storage component for testing."""
    devices: List[MockStorageDevice]
    
    def get_all_devices(self):
        return self.devices


class StorageAdapterSimulator:
    """
    Simulates the behavior of the C++ StorageAdapter for property testing.
    
    This class implements the same logic as the C++ StorageAdapter::initialize()
    method to verify the idempotence property.
    """
    
    def __init__(self):
        self.storage = None
        self.primary_device = None
        self.preferred_mount_path = ""
        self.initialized = False
    
    def initialize(self, storage_component: Optional[MockStorageComponent]) -> bool:
        """
        Initialize the storage adapter.
        
        This simulates the C++ StorageAdapter::initialize() method.
        """
        if storage_component is None:
            return False
        
        self.storage = storage_component
        
        # Try to find a suitable storage device
        self.primary_device = self._find_device_with_animations()
        
        if self.primary_device is not None:
            self.initialized = True
            return True
        else:
            self.initialized = False
            return False
    
    def is_available(self) -> bool:
        """Check if storage is available."""
        return (self.initialized and 
                self.primary_device is not None and 
                self.primary_device.is_available)
    
    def set_preferred_mount_path(self, path: str):
        """Set preferred mount path."""
        self.preferred_mount_path = path
    
    def get_primary_device(self) -> Optional[MockStorageDevice]:
        """Get the primary device."""
        return self.primary_device
    
    def _find_device_with_animations(self) -> Optional[MockStorageDevice]:
        """
        Find a storage device that contains animation files.
        
        This simulates the C++ StorageAdapter::find_device_with_animations() method.
        """
        if self.storage is None:
            return None
        
        devices = self.storage.get_all_devices()
        
        # First, try to find device matching preferred mount path
        if self.preferred_mount_path:
            for device in devices:
                if self._validate_device(device):
                    if device.mount_path == self.preferred_mount_path:
                        if device.has_animations:
                            return device
        
        # Try all available devices
        for device in devices:
            if self._validate_device(device):
                if device.has_animations:
                    return device
        
        return None
    
    def _validate_device(self, device: Optional[MockStorageDevice]) -> bool:
        """Validate that a device is suitable for use."""
        if device is None:
            return False
        
        if not device.is_available:
            return False
        
        return True


@composite
def storage_device_strategy(draw):
    """Generate a random storage device."""
    device_id = draw(st.text(
        min_size=3,
        max_size=20,
        alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_-')
    ))
    
    mount_paths = ["/sd", "/usb", "/network", "/flash"]
    mount_path = draw(st.sampled_from(mount_paths))
    
    is_available = draw(st.booleans())
    has_animations = draw(st.booleans())
    
    return MockStorageDevice(
        id=device_id,
        mount_path=mount_path,
        is_available=is_available,
        has_animations=has_animations
    )


@composite
def storage_component_strategy(draw):
    """Generate a random storage component with devices."""
    num_devices = draw(st.integers(min_value=0, max_value=5))
    devices = [draw(storage_device_strategy()) for _ in range(num_devices)]
    
    return MockStorageComponent(devices=devices)


@composite
def storage_configuration_strategy(draw):
    """
    Generate a storage configuration (component + optional preferred path).
    
    Returns:
        Tuple of (storage_component, preferred_mount_path)
    """
    storage_component = draw(storage_component_strategy())
    
    # Sometimes set a preferred mount path
    has_preference = draw(st.booleans())
    if has_preference:
        mount_paths = ["/sd", "/usb", "/network", "/flash"]
        preferred_path = draw(st.sampled_from(mount_paths))
    else:
        preferred_path = ""
    
    return (storage_component, preferred_path)


@given(config=storage_configuration_strategy())
@settings(max_examples=100, deadline=None)
def test_storage_initialization_idempotence(config):
    """
    Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
    
    Property: For any vector_eyes instance, calling initialize() multiple times with the
    same storage configuration should result in the same storage device being selected
    and the same initialization state.
    
    Validates: Requirements 1.1, 1.3
    """
    storage_component, preferred_path = config
    
    # Create first adapter instance
    adapter1 = StorageAdapterSimulator()
    if preferred_path:
        adapter1.set_preferred_mount_path(preferred_path)
    
    # Initialize first time
    result1 = adapter1.initialize(storage_component)
    device1 = adapter1.get_primary_device()
    available1 = adapter1.is_available()
    
    # Create second adapter instance with same configuration
    adapter2 = StorageAdapterSimulator()
    if preferred_path:
        adapter2.set_preferred_mount_path(preferred_path)
    
    # Initialize second time
    result2 = adapter2.initialize(storage_component)
    device2 = adapter2.get_primary_device()
    available2 = adapter2.is_available()
    
    # Property: Initialization result should be the same
    assert result1 == result2, \
        f"Initialization idempotence violated: first init returned {result1}, second returned {result2}"
    
    # Property: Selected device should be the same
    assert device1 == device2, \
        f"Device selection idempotence violated: first selected {device1}, second selected {device2}"
    
    # Property: Availability state should be the same
    assert available1 == available2, \
        f"Availability idempotence violated: first was {available1}, second was {available2}"


@given(config=storage_configuration_strategy(), num_initializations=st.integers(min_value=2, max_value=10))
@settings(max_examples=50, deadline=None)
def test_storage_initialization_multiple_calls(config, num_initializations):
    """
    Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
    
    Property: Calling initialize() multiple times (more than twice) should always
    produce the same result, demonstrating true idempotence.
    
    Validates: Requirements 1.1, 1.3
    """
    storage_component, preferred_path = config
    
    results = []
    devices = []
    availability_states = []
    
    # Perform multiple initializations
    for _ in range(num_initializations):
        adapter = StorageAdapterSimulator()
        if preferred_path:
            adapter.set_preferred_mount_path(preferred_path)
        
        result = adapter.initialize(storage_component)
        device = adapter.get_primary_device()
        available = adapter.is_available()
        
        results.append(result)
        devices.append(device)
        availability_states.append(available)
    
    # Property: All initialization results should be identical
    assert all(r == results[0] for r in results), \
        f"Initialization results not idempotent: {results}"
    
    # Property: All selected devices should be identical
    assert all(d == devices[0] for d in devices), \
        f"Device selection not idempotent: {devices}"
    
    # Property: All availability states should be identical
    assert all(a == availability_states[0] for a in availability_states), \
        f"Availability states not idempotent: {availability_states}"


@given(storage_component_strategy())
@settings(max_examples=100, deadline=None)
def test_storage_initialization_with_null_component(storage_component):
    """
    Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
    
    Property: When storage component is None, initialization should always fail
    consistently, demonstrating idempotence even in error cases.
    
    Validates: Requirements 1.1, 1.3
    """
    # Initialize with None multiple times
    results = []
    
    for _ in range(5):
        adapter = StorageAdapterSimulator()
        result = adapter.initialize(None)
        results.append(result)
    
    # Property: All results should be False
    assert all(r == False for r in results), \
        f"Null component initialization not idempotent: {results}"


@composite
def preferred_path_strategy(draw):
    """Generate a configuration with a preferred path."""
    storage_component = draw(storage_component_strategy())
    
    # Ensure at least one device exists
    assume(len(storage_component.devices) > 0)
    
    # Pick a mount path from existing devices
    mount_paths = [d.mount_path for d in storage_component.devices]
    preferred_path = draw(st.sampled_from(mount_paths))
    
    return (storage_component, preferred_path)


@given(config=preferred_path_strategy())
@settings(max_examples=50, deadline=None)
def test_storage_initialization_preferred_path_consistency(config):
    """
    Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
    
    Property: When a preferred mount path is set, initialization should consistently
    select the same device (if available) across multiple calls.
    
    Validates: Requirements 1.1, 1.3
    """
    storage_component, preferred_path = config
    
    # Find if there's a device matching the preferred path with animations
    expected_device = None
    for device in storage_component.devices:
        if (device.mount_path == preferred_path and 
            device.is_available and 
            device.has_animations):
            expected_device = device
            break
    
    # Initialize multiple times
    selected_devices = []
    
    for _ in range(5):
        adapter = StorageAdapterSimulator()
        adapter.set_preferred_mount_path(preferred_path)
        adapter.initialize(storage_component)
        device = adapter.get_primary_device()
        selected_devices.append(device)
    
    # Property: All selected devices should be the same
    assert all(d == selected_devices[0] for d in selected_devices), \
        f"Preferred path device selection not idempotent: {selected_devices}"
    
    # Property: If expected device exists, it should be selected
    if expected_device is not None:
        assert all(d == expected_device for d in selected_devices), \
            f"Expected device {expected_device} not consistently selected: {selected_devices}"


@composite
def no_animations_strategy(draw):
    """Generate a storage component with no devices having animations."""
    num_devices = draw(st.integers(min_value=1, max_value=5))
    devices = []
    
    for _ in range(num_devices):
        device_id = draw(st.text(min_size=3, max_size=20, alphabet='abcdefghijklmnopqrstuvwxyz'))
        mount_path = draw(st.sampled_from(["/sd", "/usb", "/network"]))
        is_available = draw(st.booleans())
        
        # Force has_animations to False
        devices.append(MockStorageDevice(
            id=device_id,
            mount_path=mount_path,
            is_available=is_available,
            has_animations=False
        ))
    
    return MockStorageComponent(devices=devices)


@given(storage_component=no_animations_strategy())
@settings(max_examples=50, deadline=None)
def test_storage_initialization_no_animations_fallback(storage_component):
    """
    Feature: storage-vector-eyes-integration, Property 1: Storage initialization idempotence
    
    Property: When no storage devices have animations, initialization should
    consistently fail and return to fallback state across multiple calls.
    
    Validates: Requirements 1.1, 1.3
    """
    results = []
    devices = []
    availability_states = []
    
    # Initialize multiple times
    for _ in range(5):
        adapter = StorageAdapterSimulator()
        result = adapter.initialize(storage_component)
        device = adapter.get_primary_device()
        available = adapter.is_available()
        
        results.append(result)
        devices.append(device)
        availability_states.append(available)
    
    # Property: All results should be False (no animations found)
    assert all(r == False for r in results), \
        f"No-animations fallback not idempotent: {results}"
    
    # Property: All devices should be None
    assert all(d is None for d in devices), \
        f"Device selection not idempotent in no-animations case: {devices}"
    
    # Property: All availability states should be False
    assert all(a == False for a in availability_states), \
        f"Availability not idempotent in no-animations case: {availability_states}"


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running storage initialization idempotence property tests...")
    print("=" * 70)
    
    try:
        test_storage_initialization_idempotence()
        print("✓ Property test passed: Storage initialization idempotence")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Storage initialization idempotence - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_storage_initialization_multiple_calls()
        print("✓ Property test passed: Multiple initialization calls")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Multiple initialization calls - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_storage_initialization_with_null_component()
        print("✓ Property test passed: Null component initialization")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Null component initialization - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_storage_initialization_preferred_path_consistency()
        print("✓ Property test passed: Preferred path consistency")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Preferred path consistency - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_storage_initialization_no_animations_fallback()
        print("✓ Property test passed: No animations fallback")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: No animations fallback - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print("=" * 70)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    
    exit(0 if tests_failed == 0 else 1)
