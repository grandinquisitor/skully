import math

# Available prescaler values on Timer2
available_prescalers = [1, 8, 32, 64, 128, 256, 1024]
NUM_PRESCALERS = len(available_prescalers)

def calculate_prescaler(
        f_cpu,
        animation_ms, # Animation interval in milliseconds
        target_cycles): # Target BCM cycles per animation frame

    bcm_ticks = 263  # Sum of all BCM time slice durations

    # Calculate ideal prescaler
    ideal_prescaler = (f_cpu / 1000 * animation_ms) / (target_cycles * bcm_ticks)

    print(f"For F_CPU={f_cpu} Hz and animation interval={animation_ms} ms:")
    print(f"Ideal prescaler would be: {ideal_prescaler:.2f}\n")

    # Find best available prescaler
    best_prescaler = 0
    best_error = float('inf')  # Use infinity for initial error
    best_cycles = 0

    for prescaler in available_prescalers:
        # Calculate actual BCM cycles per animation with this prescaler
        cycles = (f_cpu / 1000 * animation_ms) / (prescaler * bcm_ticks)

        # Calculate actual timer tick duration in microseconds
        tick_us = (prescaler * 1000000.0) / f_cpu

        # Check if this is within valid range
        if 1 <= cycles <= 255:
            error = abs(cycles - target_cycles)

            print(f"Prescaler {prescaler:4d}: {cycles:.2f} cycles per animation (tick = {tick_us:.2f} µs)")

            if error < best_error:
                best_error = error
                best_prescaler = prescaler
                best_cycles = round(cycles)

    if best_prescaler == 0:
        print("\nNo valid prescaler found!")
    else:
        print(f"\nBest prescaler: {best_prescaler}")
        print(f"This will give {best_cycles} BCM cycles per animation frame")
        actual_ms = best_cycles * bcm_ticks * best_prescaler / (f_cpu / 1000)
        print(f"Actual animation interval: {actual_ms:.2f} ms")
        print(f"BCM cycle duration: {actual_ms / best_cycles:.2f} ms")


if __name__ == "__main__":
    print("=== 1 MHz CPU ===")
    calculate_prescaler(1000000, 8, 15)
    print("\n=== 2 MHz CPU ===")
    calculate_prescaler(2000000, 8, 15)
