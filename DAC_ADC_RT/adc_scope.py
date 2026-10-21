"""adc_scope.py - slow waveform monitor for the AWG board (run on the PYNQ board).

Usage in a notebook, after the AWG overlay is loaded and the DAC is playing:

    from adc_scope import setup_adc, capture, find_pulses, plot
    setup_adc(ol)                    # once
    x = capture(ol)                  # 32768 samples = 13.3 us at 2.4576 GSps
    print(find_pulses(x))            # start, width, height of each pulse found
    plot(x)

Samples are returned normalised to ADC full scale (+-1.0). Convert to volts with your own
calibration (attenuator + ADC full-scale for your input).
"""
import time
import numpy as np

FS_ADC = 2.4576e9           # effective rate of the I stream: 4.9152 GSps ADC, decimation 2
SAMPLES_PER_BEAT = 8        # 128-bit AXI-Stream beat = 8 x int16
MAX_BEATS = 4096            # packet_generator limit (2..4096)
PGEN_PACKETSIZE = 0x00
PGEN_TRANSFER = 0x04


def setup_adc(ol, tile=0, block=0):
    """Put the ADC mixer at 0 Hz so the I stream is the plain input waveform. Call once after loading."""
    import xrfdc
    b = ol.usp_rf_data_converter_0.adc_tiles[tile].blocks[block]
    b.MixerSettings["Freq"] = 0.0
    b.UpdateEvent(xrfdc.EVENT_MIXER)


def capture(ol, n_samples=32768, dma="axi_dma_adc", pgen="packet_generator_adc", timeout=2.0):
    """Grab one contiguous block of ADC samples (free-running, not synchronised to the AWG)."""
    from pynq import allocate
    beats = int(np.ceil(n_samples / SAMPLES_PER_BEAT))
    if not 2 <= beats <= MAX_BEATS:
        raise ValueError(f"n_samples must be between {2*SAMPLES_PER_BEAT} and {MAX_BEATS*SAMPLES_PER_BEAT}")
    pg = getattr(ol, pgen)
    d = getattr(ol, dma)
    pg.write(PGEN_TRANSFER, 0)
    pg.write(PGEN_PACKETSIZE, beats)
    buf = allocate(shape=(beats * SAMPLES_PER_BEAT,), dtype=np.int16)
    try:
        d.recvchannel.transfer(buf)
        pg.write(PGEN_TRANSFER, 1)
        t0 = time.time()
        while not d.recvchannel.idle:
            if time.time() - t0 > timeout:
                raise TimeoutError("ADC DMA did not complete - is the ADC clock running / overlay loaded with xrfclk set?")
            time.sleep(0.001)
        pg.write(PGEN_TRANSFER, 0)
        x = np.array(buf, dtype=np.float64) * 2.0 ** -15
    finally:
        buf.freebuffer()
    return x[:n_samples]


def find_pulses(x, fs=FS_ADC, frac=0.5):
    """Locate pulses by thresholding at `frac` of the peak (after removing the median baseline).
    Returns a list of dicts: start_ns (from capture start), width_ns (at threshold), height (normalised)."""
    x = np.asarray(x, dtype=np.float64)
    y = x - np.median(x)
    sign = 1.0 if y.max() >= -y.min() else -1.0
    y = sign * y
    peak = y.max()
    if peak < 1e-3:
        return []
    above = y > frac * peak
    d = np.diff(above.astype(np.int8))
    rise = np.flatnonzero(d == 1) + 1
    fall = np.flatnonzero(d == -1) + 1
    out = []
    for r in rise:
        f = fall[fall > r]
        if len(f) == 0:
            break   # pulse cut by the end of the capture
        out.append({"start_ns": r / fs * 1e9,
                    "width_ns": (f[0] - r) / fs * 1e9,
                    "height": float(sign * y[r:f[0]].max())})
    return out


def plot(x, fs=FS_ADC):
    import matplotlib.pyplot as plt
    t_us = np.arange(len(x)) / fs * 1e6
    plt.figure(figsize=(10, 3.5))
    plt.plot(t_us, x)
    plt.xlabel("time (us)")
    plt.ylabel("ADC (fraction of full scale)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
