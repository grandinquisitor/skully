def select_data_rate(read_freq_ms):
    # Data rates in Hz
    data_rates = [
        ("LIS3DH_ODR_1Hz", 1),
        ("LIS3DH_ODR_10Hz", 10),
        ("LIS3DH_ODR_25Hz", 25),
        ("LIS3DH_ODR_50Hz", 50),
        ("LIS3DH_ODR_100Hz", 100),
        ("LIS3DH_ODR_200Hz", 200),
        ("LIS3DH_ODR_400Hz", 400),
        ("LIS3DH_ODR_1kHz620", 1620)
    ]

    # Convert read frequency to Hz
    read_freq_hz = 1000 / read_freq_ms

    # Find the best data rate
    for name, rate in data_rates:
        if rate >= read_freq_hz:
            return name

    # If no suitable rate is found, return the highest available
    return data_rates[-1][0]

if __name__ == "__main__":
    read_freq_ms = 64
    selected_rate = select_data_rate(read_freq_ms)
    print(f"Recommended data rate for {read_freq_ms}ms read rate: {selected_rate}")