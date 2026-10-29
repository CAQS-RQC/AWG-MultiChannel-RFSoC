"""
bode_sweep.py  --  measure the DAC -> ADC bandwidth (Bode-plot style frequency sweep)
=====================================================================================

What it does
------------
For every frequency of a (log-spaced) list it
  1. makes the AWG play a continuous sine of constant amplitude at that frequency,
  2. captures the ADC (triggered on the start of the AWG sequence),
  3. measures the amplitude of that sine in the capture with a least-squares sine fit
     (no FFT leakage, works for any capture length),
and finally plots gain [dB] vs frequency (log axis), marks the -3 dB points and
saves a CSV + PNG.

Hardware: DAC_A -> cable/attenuator -> ADC_A  (a loopback; use enough attenuation so
the ADC does not clip, the script warns if it does).  The measured curve is the WHOLE
chain: DAC reconstruction (sin(x)/x droop), baluns/cables/attenuators, ADC front end.

Limits
------
* The ADC runs at 2.4576 GS/s (decimation 2) => it only sees up to ~1.23 GHz
  (Nyquist).  The sweep stops at 0.98 * Nyquist by default; frequencies above that
  are rejected unless you pass allow_alias=True (the ADC decimation filter attenuates
  that region, so the numbers there are not trustworthy).
* Frequencies are snapped to the nearest value that fits an INTEGER number of cycles
  into the AWG window.  This keeps the looped waveform continuous (no phase jump at
  the loop point).  With window = 1 us the grid is 1 MHz; use a longer window
  (e.g. 5 us -> 200 kHz grid) for finer low-frequency resolution.
* Gain is relative (0 dB = strongest point) unless you give `adc_vfs` (ADC full-scale
  peak voltage in volts), then the absolute DAC->ADC voltage gain is also saved.

Use from a notebook (same session as your AWG + scope):
    from bode_sweep import run_bode
    res = run_bode(awg, scope, fmin=5e6, fmax=1.2e9, points=100, amp=0.1)
    res["bw_low_hz"], res["bw_high_hz"]       # -3 dB band edges

Use from a terminal on the board (as root):
    python3 bode_sweep.py                      # real hardware, default settings
    python3 bode_sweep.py --fmax 1e9 --points 200 --amp 0.05
    python3 bode_sweep.py --demo               # no hardware: simulated band-pass chain
"""

from __future__ import annotations

import argparse
import csv
import time
import warnings
from typing import Optional

import numpy as np

from rfsoc_awg import AWG, Pulse, DAC_SR, MAX_VPK, SLOT_SAMPLES, us, ns

ADC_FS = 2.4576e9            # ADC sample rate in the AWG+scope bitstream [Hz]
ADC_NYQUIST = ADC_FS / 2


# --------------------------------------------------------------------------- #
#  Measurement helpers
# --------------------------------------------------------------------------- #
def fit_sine(t_s: np.ndarray, y: np.ndarray, f: float):
    """
    Least-squares fit  y ~ a*sin(2*pi*f*t) + b*cos(2*pi*f*t) + c  at the KNOWN frequency f.
    Returns (amplitude, phase_rad, offset, residual_rms).  Amplitude = peak value.
    """
    w = 2 * np.pi * f * t_s
    A = np.column_stack([np.sin(w), np.cos(w), np.ones_like(w)])
    (a, b, c), *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ np.array([a, b, c])
    return float(np.hypot(a, b)), float(np.arctan2(b, a)), float(c), float(np.sqrt(np.mean(resid ** 2)))


def bandwidth_3db(freqs: np.ndarray, gain_db: np.ndarray, drop_db: float = 3.0):
    """
    -3 dB band edges relative to the PEAK of the curve (so a band-pass chain with an AC
    coupled low end works too).  Returns (f_low, f_high), each None if the curve never
    falls `drop_db` below the peak on that side.  Linear interpolation in log(f).
    """
    ipk = int(np.argmax(gain_db))
    level = gain_db[ipk] - drop_db

    def crossing(idx_range):
        prev = ipk
        for i in idx_range:
            if gain_db[i] < level:
                # interpolate between prev (above level) and i (below level)
                x0, x1 = np.log10(freqs[prev]), np.log10(freqs[i])
                y0, y1 = gain_db[prev], gain_db[i]
                return float(10 ** (x0 + (level - y0) * (x1 - x0) / (y1 - y0)))
            prev = i
        return None

    return crossing(range(ipk - 1, -1, -1)), crossing(range(ipk + 1, len(freqs)))


# --------------------------------------------------------------------------- #
#  The sweep
# --------------------------------------------------------------------------- #
def run_bode(awg: AWG, scope, fmin: float = 5e6, fmax: float = 1.2e9, points: int = 100,
             amp: float = 0.1, dac: str = "DAC_A", slot: str = "z", log: bool = True,
             length_ns: float = 2000.0, pre_ns: float = 10.0, averages: int = 3,
             settle_s: float = 0.01, adc_vfs: Optional[float] = None, auto_window: bool = True,
             allow_alias: bool = False, out: Optional[str] = "bode", show: bool = True,
             progress: bool = True) -> dict:
    """
    Sweep a sine from `dac` and measure it with `scope`.  See the module docstring.

    awg, scope   your AWG (connected) and AdcScope (or anything with configure/acquire)
    fmin, fmax   sweep range [Hz]
    points       number of requested points (snapping to the window grid can merge a few)
    amp          sine peak amplitude [V] at the DAC (max 0.25 V). Keep the ADC unclipped!
    dac, slot    which output / slot plays the sine ("DAC_A" pairs with trigger "awg2")
    log          log-spaced (True) or linear-spaced (False) frequencies
    length_ns    MINIMUM ADC capture length per acquisition [ns]; it is automatically
                 extended to >= 3 cycles of each frequency (up to the scope depth, ~53 us)
    auto_window  if True and fmin needs more than the current AWG window, the window is
                 lengthened (to 10 cycles of fmin, at most 26.67 us). This CLEARS the AWG
                 waveforms. Lowest possible frequency = 1 / window = 37.5 kHz at 26.67 us.
    averages     captures per frequency (amplitudes are averaged)
    settle_s     wait after each upload before capturing [s]
    adc_vfs      ADC full-scale PEAK voltage [V]; gives absolute gain if known
    out          file prefix: writes <out>.csv and <out>.png (None = write nothing)
    show         display the figure (plt.show())
    Returns a dict with arrays freq_hz, amp_fs, gain_db (relative to peak), resid_fs,
    clipped (bool array) and the -3 dB edges bw_low_hz / bw_high_hz.
    """
    if not (0 < amp <= MAX_VPK):
        raise ValueError(f"amp must be in (0, {MAX_VPK:.3f}] V")
    if fmax > 0.98 * ADC_NYQUIST and not allow_alias:
        raise ValueError(f"fmax={fmax/1e6:.0f} MHz is above the ADC Nyquist limit "
                         f"({0.98*ADC_NYQUIST/1e6:.0f} MHz). Lower it or pass allow_alias=True.")
    dac = dac.upper()
    trigger = "awg2" if dac == "DAC_A" else "awg0"          # DAC_A = AWG channel 2, DAC_B = 0

    # --- the AWG window sets the lowest frequency (1 cycle) and the frequency grid (1/window)
    if auto_window and 10 / fmin > awg.window:
        awg.set_window(min(10 / fmin, SLOT_SAMPLES / DAC_SR))
        print(f"Window set to {awg.window/us:.2f} us so that low frequencies fit "
              f"(max possible is {SLOT_SAMPLES/DAC_SR/us:.2f} us)")
    f_floor = 1 / awg.window
    if fmin < 0.99 * f_floor:
        print(f"NOTE: the AWG window ({awg.window/us:.2f} us) cannot hold fewer than 1 cycle, so the "
              f"lowest usable frequency is {f_floor/1e3:.1f} kHz (not {fmin/1e3:.3g} kHz). "
              f"Starting there.")
    fmin = max(fmin, f_floor)
    if fmax <= fmin:
        raise ValueError(f"fmax ({fmax:.3g} Hz) must be above the lowest usable frequency "
                         f"({f_floor:.3g} Hz).")

    # frequency list, snapped to integer cycles per AWG window, duplicates removed
    raw = np.geomspace(fmin, fmax, points) if log else np.linspace(fmin, fmax, points)
    freqs = np.unique([awg.coherent_freq(f) for f in raw])
    freqs = freqs[(freqs >= 0.5 * fmin) & (freqs < DAC_SR / 2)]
    if len(freqs) < min(5, points):
        raise ValueError(
            f"Only {len(freqs)} distinct frequency/ies fit the grid of {1/awg.window:.3g} Hz "
            f"(= 1/window) between {fmin:.3g} and {fmax:.3g} Hz. Use a longer AWG window "
            f"(max {SLOT_SAMPLES/DAC_SR/us:.1f} us -> {DAC_SR/SLOT_SAMPLES/1e3:.1f} kHz grid) "
            f"or a higher fmin.")
    max_len_ns = (getattr(scope, "depth", 131072) * 8 / ADC_FS * 1e9) * 0.97   # scope buffer limit
    print(f"Bode sweep {freqs[0]:.4g} .. {freqs[-1]:.4g} Hz, {len(freqs)} points, "
          f"amp {amp*1e3:.0f} mV, window {awg.window/us:.2f} us "
          f"(frequency grid {1/awg.window:.4g} Hz), trigger={trigger}")

    awg.clear()                                              # only our sine, nothing else
    awg.select_slot(slot)

    amps, resid, clipped = [], [], []
    t0 = time.time()
    for i, f in enumerate(freqs):
        awg.set_waveform(dac, slot, Pulse(freq=f, amp=amp))
        if i == 0:
            awg.play(channels=[dac], slot=slot)              # first point: upload + outputs on
        else:
            awg.upload()                                     # outputs stay on, RAM is rewritten
        time.sleep(settle_s)
        len_i = float(np.clip(3e9 / f, length_ns, max_len_ns))       # >= 3 cycles if the buffer allows
        scope.configure(trigger=trigger, pre_ns=pre_ns, length_ns=len_i)
        if len_i * 1e-9 * f < 1.5 and i == 0:
            print(f"  WARNING: only {len_i*1e-9*f:.2f} cycles fit in the ADC buffer at {f/1e3:.1f} kHz; "
                  f"the low-frequency amplitudes are less accurate.")
        a_list, r_list, clip = [], [], False
        for _ in range(max(1, averages)):
            t_ns, y = scope.acquire()
            y = np.asarray(y, float)
            a, _, _, r = fit_sine(np.asarray(t_ns, float) * 1e-9, y, f)
            a_list.append(a)
            r_list.append(r)
            clip |= bool(np.max(np.abs(y)) > 0.95)
        amps.append(float(np.mean(a_list)))
        resid.append(float(np.mean(r_list)))
        clipped.append(clip)
        if progress and (i % 10 == 0 or i == len(freqs) - 1):
            print(f"  [{i+1:3d}/{len(freqs)}] {f:12.5g} Hz   ADC amplitude {amps[-1]:.4f} FS"
                  f"{'  (CLIPPING!)' if clip else ''}")
    awg.stop()

    freqs, amps, resid, clipped = (np.asarray(x) for x in (freqs, amps, resid, clipped))
    gain_db = 20 * np.log10(np.maximum(amps, 1e-12) / np.max(amps))
    lo, hi = bandwidth_3db(freqs, gain_db)
    res = dict(freq_hz=freqs, amp_fs=amps, gain_db=gain_db, resid_fs=resid, clipped=clipped,
               bw_low_hz=lo, bw_high_hz=hi, amp_dac_v=amp,
               gain_abs_db=(20 * np.log10(amps * adc_vfs / amp) if adc_vfs else None))
    if clipped.any():
        warnings.warn(f"ADC clipped at {int(clipped.sum())} points: add attenuation or lower amp.")
    print(f"Done in {time.time()-t0:.1f} s.  -3 dB band: "
          f"{'<' + format(freqs[0]/1e6, '.1f') + ' MHz' if lo is None else format(lo/1e6, '.2f') + ' MHz'}"
          f" .. {'>' + format(freqs[-1]/1e6, '.0f') + ' MHz' if hi is None else format(hi/1e6, '.1f') + ' MHz'}")

    if out:
        _save(res, out)
    _plot(res, out, show)
    return res


# --------------------------------------------------------------------------- #
#  Output
# --------------------------------------------------------------------------- #
def _save(res: dict, prefix: str) -> None:
    with open(f"{prefix}.csv", "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["freq_hz", "adc_amp_fs", "gain_db_rel_peak",
                     "gain_abs_db" if res["gain_abs_db"] is not None else "gain_abs_db(n/a)",
                     "fit_residual_fs", "adc_clipped"])
        for i, f in enumerate(res["freq_hz"]):
            wr.writerow([f"{f:.1f}", f"{res['amp_fs'][i]:.6f}", f"{res['gain_db'][i]:.3f}",
                         "" if res["gain_abs_db"] is None else f"{res['gain_abs_db'][i]:.3f}",
                         f"{res['resid_fs'][i]:.6f}", int(res["clipped"][i])])
    print(f"Saved {prefix}.csv")


def _plot(res: dict, prefix: Optional[str], show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    f, g = res["freq_hz"], res["gain_db"]               # x axis in Hz
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True,
                                  gridspec_kw={"height_ratios": [3, 1.3]})
    ax.semilogx(f, g, "o-", ms=3, lw=1)
    ax.axhline(-3, color="r", ls=":", lw=1)
    for edge, side in ((res["bw_low_hz"], "low"), (res["bw_high_hz"], "high")):
        if edge:
            ax.axvline(edge, color="r", ls="--", lw=0.8)
            ax.annotate(f"{edge:.4g} Hz", (edge, -3), xytext=(4, 8),
                        textcoords="offset points", color="r")
    bad = res["clipped"]
    if bad.any():
        ax.plot(f[bad], g[bad], "rx", label="ADC clipped")
        ax.legend()
    ax.set_ylabel("gain [dB], 0 dB = peak")
    ax.set_title("DAC -> ADC frequency response")
    ax.grid(True, which="both", alpha=0.3)
    ax2.semilogx(f, res["amp_fs"], "o-", ms=3, lw=1, color="C1", label="ADC amplitude")
    ax2.semilogx(f, res["resid_fs"], "-", lw=1, color="gray", label="fit residual (noise+distortion)")
    ax2.set_xlabel("frequency [Hz]")
    ax2.set_ylabel("ADC [FS]")
    ax2.set_yscale("log")
    ax2.grid(True, which="both", alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    if prefix:
        fig.savefig(f"{prefix}.png", dpi=150)
        print(f"Saved {prefix}.png")
    if show:
        plt.show()


# --------------------------------------------------------------------------- #
#  Simulated hardware for --demo (a band-pass chain: 10 MHz high-pass x 600 MHz low-pass)
# --------------------------------------------------------------------------- #
class SimScope:
    """AdcScope look-alike that 'measures' the playing waveform through a made-up
    band-pass response.  Only for testing the script without hardware."""

    def __init__(self, awg, f_hp=10e6, f_lp=600e6, noise=1e-3, vfs=0.5):
        self.awg, self.f_hp, self.f_lp, self.noise, self.vfs = awg, f_hp, f_lp, noise, vfs
        self.cfg = dict(length_ns=1000.0, pre_ns=10.0)
        self.last = {"triggered": True}
        self.depth = 131072                                   # beats, like the real scope

    def configure(self, trigger="awg2", level=0.0, pre_ns=10.0, length_ns=1000.0):
        self.cfg = dict(length_ns=length_ns, pre_ns=pre_ns)

    def H(self, f):
        return (f / self.f_hp) / np.sqrt(1 + (f / self.f_hp) ** 2) / np.sqrt(1 + (f / self.f_lp) ** 4)

    def acquire(self, timeout=0.5):
        a = self.awg
        v = a._waves.get(("DAC_A", a.slot_playing), np.zeros(a.n_samples))
        spec = np.fft.rfft(v) * self.H(np.fft.rfftfreq(len(v), 1 / DAC_SR))
        vf = np.fft.irfft(spec, len(v))                       # response of one (periodic) loop
        n = int(self.cfg["length_ns"] * 1e-9 * ADC_FS)
        t_ns = np.arange(n) / ADC_FS * 1e9 - self.cfg["pre_ns"]
        idx = np.round(t_ns * 1e-9 * DAC_SR).astype(int) % len(vf)
        y = vf[idx] / self.vfs + self.noise * np.random.randn(n)
        return t_ns, y


# --------------------------------------------------------------------------- #
#  Command line
# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(description="DAC->ADC bandwidth sweep (Bode-style)")
    p.add_argument("--demo", action="store_true", help="no hardware: simulated band-pass chain")
    p.add_argument("--bitfile", default="awg_adc.bit")
    p.add_argument("--fmin", type=float, default=5e6)
    p.add_argument("--fmax", type=float, default=1.2e9)
    p.add_argument("--points", type=int, default=100)
    p.add_argument("--amp", type=float, default=0.1, help="DAC peak amplitude [V]")
    p.add_argument("--dac", default="DAC_A", choices=["DAC_A", "DAC_B"])
    p.add_argument("--window-us", type=float, default=1.0, help="AWG window [us]; sets the frequency grid")
    p.add_argument("--length-ns", type=float, default=2000.0)
    p.add_argument("--avg", type=int, default=3)
    p.add_argument("--adc-vfs", type=float, default=None, help="ADC full-scale peak voltage [V]")
    p.add_argument("--linear", action="store_true", help="linear instead of log spacing")
    p.add_argument("--out", default="bode")
    p.add_argument("--no-show", action="store_true")
    a = p.parse_args()

    if a.demo:
        awg = AWG(window=a.window_us * us, dry_run=True, verbose=False)
        scope = SimScope(awg)
    else:                                                    # same start-up as your notebook
        from pynq import Overlay, PL
        import xrfclk
        from adc_scope import AdcScope
        PL.reset()
        xrfclk.set_ref_clks(lmk_freq=245.76, lmx_freq=491.52)
        ol = Overlay(a.bitfile)
        awg = AWG(window=a.window_us * us, bitfile=ol, verbose=False)
        awg.connect()
        scope = AdcScope(ol)

    run_bode(awg, scope, fmin=a.fmin, fmax=a.fmax, points=a.points, amp=a.amp, dac=a.dac,
             log=not a.linear, length_ns=a.length_ns, averages=a.avg, adc_vfs=a.adc_vfs,
             out=a.out, show=not a.no_show)


if __name__ == "__main__":
    main()
