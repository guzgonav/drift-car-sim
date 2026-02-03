"""
Signal filtering utilities for drift state estimation.
"""

def low_pass_filter(current_value, previous_value, alpha):
    """
    Simple low-pass filter to smooth noisy data.

    filtered = alpha * current + (1 - alpha) * previous

    Args:
        current_value: Current measurement
        previous_value: Previous filtered value
        alpha: Filter coefficient (0-1)
               0 = ignore current, 1 = no filtering

    Returns:
        float: Filtered value
    """
    if previous_value is None:
        return current_value
    return alpha * current_value + (1.0 - alpha) * previous_value
