def calculate_refresh_rate(cpu_clock_hz, prescaler):
    """
    Calculate effective LED refresh rate using BCM (Binary Code Modulation) timing
    
    Args:
        cpu_clock_hz (float): CPU clock frequency in Hz
        prescaler (int): Timer prescaler value (typically 1, 8, 64, etc.)
    
    Returns:
        float: Effective single cycle refresh rate in Hz 
        float: Effective total cycle refresh rate in Hz 
    """
    # Calculate timer tick duration
    timer_freq = cpu_clock_hz / prescaler
    timer_tick = 1.0 / timer_freq
    
    # BCM time slice values (OCR2A sequence)
    ocr_values = [1, 2, 4, 8, 16, 32, 64, 128]
    
    # Calculate total cycle time
    total_cycle_time = sum((ocr + 1) * timer_tick for ocr in ocr_values)
    
    # Return refresh rate (1 cycle per total cycle time)
    return timer_freq, 1.0 / total_cycle_time

# Example usage (2 MHz clock with prescaler 64)
print(calculate_refresh_rate(2e6, 64))  # Output: ~118.8 Hz