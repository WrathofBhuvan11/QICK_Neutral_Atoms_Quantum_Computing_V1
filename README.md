# ZCU216 QICK Real-Time Qubit Readout

Low-latency, deterministic FPGA pipeline for neutral-atom quantum-computer qubit readout. Classifies 100 qubits (10×10 array) on a Xilinx ZCU216 RFSoC every frame, with the decision vector fed back to QICK tProc for mid-circuit control. Built around a fully pipelined 520 MHz PL datapath and a thin PYNQ host driver.

Target: **Xilinx ZCU216 (XCZU49DR-2FFVF1760E), Vivado 2023.2 + PYNQ.**

## Numbers worth knowing

| | |
|---|---|
| Image | 512×512 px @ 8 bpp |
| Qubits | 100 on a 10×10 grid; spacing 51 px |
| Pixel stream | 64-bit AXIS (8 px/beat), 32 768 beats/frame |
| PL pipeline | 7 stages, 520 MHz, fully deterministic |
| Result stream | 16-bit AXIS, 25 beats/frame (100/4 lanes) |
| **PL decision compute** | **~73 ns/frame** (bit-exact run-to-run) |
| FPGA streaming (current) | ~142 µs/frame (VDMA-bound) |
| End-to-end (current, w/ PS) | ~540 µs/frame |
| **End-to-end (proposed, camera-direct)** | **~125 ns after ROIs land** |

The PL itself is already sub-µs and deterministic. The wall today is the VDMA + PS round-trip; tomorrow it's the camera sensor itself.

## What's in this repo

- **`ZCU216_QICK_Qubit_Readout_Reference.docx`** - Full behavioural & microarchitectural design reference (parameters, address maps, pipeline stages, CDC strategy, XDC exceptions, PYNQ driver flow, testbench plan). This is the source-of-truth doc; an RTL+PYNQ engineer can rebuild every file from it.
- **`QICK_fpga_FULL.pptx`** - 22-slide presentation: physics/system context, QICK background, FPGA-fabric primer (CLB / LUT / routing), pipeline deep-dive, current vs proposed latency, comparison to Quantum Machines OPX+, and future work (camera-direct path via CoaXPress + Kaya FMC, history-based prediction, SPAD arrays).
- RTL sources, XDC, BD, testbench, and PYNQ notebook (per the design-reference file manifest).

## Pipeline at a glance

```
VDMA mm2s (300 MHz, 64-bit AXIS)
  → async FIFO (300→520, gray code)
  → pixel_injector       (frame/line FSM, 8 px/beat)
  → coord_matcher        (3-stage, 800-way compare cone)
  → roi_extractor        (2-row line buffer, 16-col sliding window)
  → roi_storage          (4 lanes × 2 ping-pong banks + noise baseline)
  → read_streamer        (4-lane BRAM read + saturating noise subtract)
  → 4× gaussian_filter_engine  (1-2-1 / 2-4-2 / 1-2-1 kernel, runtime threshold)
  → async FIFO (520→300)
  → AXI DMA s2mm (300 MHz, 16-bit AXIS)
```

Two AXI-Lite slaves expose qubit (X,Y) coords + control (`0xB000_0000`) and a 32-bit/520-MHz latency counter with 7 per-frame timestamps (`0xB002_0000`). Software reads the latency snapshot in the inter-frame idle window - every stage is timed cycle-accurately.

## Build / run

1. **Vivado 2023.2**, target `xczu49dr-ffvf1760-2-e`. Compile `params_pkg.sv` first; package the user IP as `xilinx.com:user:datastream_processor_qick:1.0` with `component.xml`; attach `datastream_processor.xdc` SCOPED OOC; build the BD per the wiring map in the reference doc; add the wrapper XDC; synth + impl; export `.bit` + `.hwh`.
2. **Simulation**: top is `tb_qubit_readout_extended`. Runs a dark-mode capture phase, 8 main frames (incl. checkerboard with backpressure and a near-threshold frame at scores 512 vs 496), and 2 reprogram frames at +5,+5 px offset. Expected output ends `*** ALL TESTS PASSED ***`.
3. **PYNQ deploy**: SD-card-boot the QICK image, copy `.bit`/`.hwh` to `/home/xilinx/jupyter_notebooks/Latency_Test_QICK_Diff/`, copy the notebook + `frames/`, run cells in order. Step 12 = one-time dark baseline; Step 13 = experiment loop with hardware noise subtraction live; Step 14 = N=20 determinism histograms.

## vs Quantum Machines OPX+

| | This work | OPX+ |
|---|---|---|
| FPGA chain (camera ingress → decision) | ~125 ns + QICK loop | <100 µs (excl. camera) |
| Per-qubit processing | ~0.48 ns (SIMD 4-lane parallel) | ~1 µs (processor-serialised) |
| Feedback path | Direct PL → QICK tProc, no PS round-trip | OPX+ conditional / parametric |
| Architecture | Custom RTL + QICK tProc; hard-wired, deterministic | QUA-programmable processor, more overhead |

## Roadmap

Camera-direct path (proposal): ORCA-Quest 2 qCMOS → 4× CoaXPress-6 (6.25 Gb/s/lane) → Kaya KY-FMC-II-CXP-4R → ZCU216 GTY/PL → 128-bit AXIS-Video @ 312.5 MHz → 128→64 truncator → CDC → existing 520 MHz `datastream_processor` → direct AXIS feedback into QICK tProc. Skips the PS entirely. With this in place the **sensor (~1.88 ms/frame), not the FPGA, sets the wall** - the FPGA adds ~125 ns, or 0.007 % of one frame.

Exploration: history-table-based qubit-state prediction (branch-predictor analogue, possibly out-of-order per qubit), faster sensor modes / lower bit depth on ORCA, and SPAD arrays (1 bit/pixel, much faster scan).

---