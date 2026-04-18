"""Wiring signatures for the PIPE Rev 7.1 SerDes low-pin-count interface.

These define the typed port interfaces used to connect PIPE adapter components.
Direction (In/Out) is from the perspective of the component that *owns*
the interface; connecting components see flipped directions.

The PIPE (PHY Interface for PCI Express, USB, DisplayPort, and Converged IO)
specification defines a standard interface between a MAC layer and a PHY.
Directions here are from the PHY's perspective:

  - ``In``  = driven by the MAC (received by the PHY)
  - ``Out`` = driven by the PHY (received by the MAC)
"""

from amaranth.lib.wiring import Signature, In, Out


class DRPRequestSignature(Signature):
    """Internal DRP request port for sub-controllers within the PIPE adapter.

    Each sub-controller (rate-change FSM, RxDet FSM, LFPS engine, etc.)
    presents this interface to a central DRP arbiter/mux.  The arbiter
    grants bus access and routes transactions to the underlying PHY
    reconfiguration port.

    The ``lock_req`` / ``lock_ack`` pair supports atomic multi-write
    sequences where a sub-controller must hold exclusive DRP access
    across several transactions (e.g. read-modify-write of a CSR).

    Directions are from the requesting sub-controller's perspective:

      - ``Out`` = driven by the sub-controller (toward the arbiter)
      - ``In``  = driven by the arbiter (toward the sub-controller)
    """

    def __init__(self):
        super().__init__(
            {
                "addr": Out(24),  # CSR address (24-bit reconfiguration space)
                "wrdata": Out(32),  # Write data
                "wren": Out(1),  # Write enable — assert to start a write
                "rden": Out(1),  # Read enable — assert to start a read
                "lock_req": Out(1),  # Request exclusive DRP bus lock (multi-write)
                "lock_ack": In(1),  # Lock granted by the arbiter
                "ready": In(1),  # Write accepted / arbiter ready for next txn
                "rdvld": In(1),  # Read data valid strobe
                "rddata": In(32),  # Read data returned from PHY CSR
            }
        )


class PIPESerDesSignature(Signature):
    """PIPE Rev 7.1 SerDes low-pin-count interface.

    This signature represents the complete PIPE interface between a MAC
    layer and a PHY, covering data transfer, clocking, command/status,
    message bus, and the MacCLK domain.  Optional signal groups are
    included based on which protocols are enabled.

    All directions are from the PHY's perspective:

      - ``In``  = driven by the MAC (inputs to the PHY)
      - ``Out`` = driven by the PHY (outputs from the PHY)

    Parameters
    ----------
    data_width : int
        Maximum data bus width in bits.  Must be one of 10, 20, 32, 40,
        64, or 80.  Determines the width of ``tx_data`` and ``rx_data``.
    usb : bool
        Include USB-specific signals (``rx_polarity``, ``rx_termination``).
    sata : bool
        Include SATA-specific signals (``rx_standby_status``,
        ``align_detect``).
    dp : bool
        Include DisplayPort-specific signals (reserved for future use).
    """

    def __init__(self, *, data_width=80, usb=True, sata=False, dp=False):
        members = {}

        # ----------------------------------------------------------------
        # Data interface
        # ----------------------------------------------------------------
        # TX data from MAC to PHY for serialisation.
        members["tx_data"] = In(data_width)
        # TX data valid qualifier (MAC asserts when tx_data is meaningful).
        members["tx_data_valid"] = In(1)
        # RX data from PHY to MAC after deserialisation.
        members["rx_data"] = Out(data_width)

        # ----------------------------------------------------------------
        # Clock interface
        # ----------------------------------------------------------------
        # PCLK input from the MAC (or PHY loopback); clocks the PIPE i/f.
        members["pclk"] = In(1)
        # PHY-generated PCLK at the maximum supported rate.
        members["max_pclk"] = Out(1)
        # Recovered clock from the CDR (byte-rate).
        members["rx_clk"] = Out(1)

        # ----------------------------------------------------------------
        # Command interface (MAC → PHY)
        # ----------------------------------------------------------------
        # Protocol selector: 0=default, 1=USB, 2=SATA, 3=DisplayPort.
        members["phy_mode"] = In(4)
        # Active-low asynchronous reset.
        members["reset_n"] = In(1)
        # Power state request: P0 (active) through P3 (off).
        members["power_down"] = In(4)
        # Signaling rate selector (gen1/2/3/4/5 encoding).
        members["rate"] = In(4)
        # TX data path width selector.
        members["width"] = In(3)
        # RX data path width selector (independent of TX).
        members["rx_width"] = In(3)
        # TX electrical idle per sub-lane (one bit each).
        members["tx_elec_idle"] = In(4)
        # Triggers receiver detection or loopback entry.
        members["tx_detect_rx_loopback"] = In(1)

        # USB-only command signals
        if usb:
            # RX polarity inversion (swap D+/D−).
            members["rx_polarity"] = In(1)
            # RX termination control (enable/disable 45 Ω termination).
            members["rx_termination"] = In(1)
            # MAC transmit LFPS mode (PIPE §8.10).
            # 0 = PHY generates LFPS pattern (default).
            # 1 = MAC drives LFPS on TxData (for Gen2 PWM/LBPM).
            members["mac_transmit_lfps"] = In(1)

        # USB + SATA shared command signals
        if usb or sata:
            # Places the receiver in low-power standby.
            members["rx_standby"] = In(1)
            # MAC acknowledges a PHY-initiated PCLK rate change.
            members["pclk_change_ack"] = In(1)

        # ----------------------------------------------------------------
        # Status interface (PHY → MAC)
        # ----------------------------------------------------------------
        # Single-cycle completion pulse for async PHY operations.
        members["phy_status"] = Out(1)
        # RX valid — CDR locked and recovered clock stable.
        members["rx_valid"] = Out(1)
        # Asynchronous electrical-idle detect on the RX pair.
        members["rx_elec_idle"] = Out(1)

        # USB + SATA shared status signals
        if usb or sata:
            # Encoded RX status (receiver detect result, SKP added, etc.).
            members["rx_status"] = Out(3)
            # PHY signals readiness for a PCLK frequency change.
            members["pclk_change_ok"] = Out(1)

        # SATA-only status signals
        if sata:
            # Indicates the receiver has exited standby.
            members["rx_standby_status"] = Out(1)
            # SATA ALIGN primitive detected on the RX stream.
            members["align_detect"] = Out(1)

        # ----------------------------------------------------------------
        # Message bus
        # ----------------------------------------------------------------
        # MAC-to-PHY message bus (8-bit encoded messages).
        members["m2p_msg_bus"] = In(8)
        # PHY-to-MAC message bus (8-bit encoded messages).
        members["p2m_msg_bus"] = Out(8)

        # ----------------------------------------------------------------
        # MacCLK domain
        # ----------------------------------------------------------------
        # Active-low async reset for the MacCLK domain.
        members["mac_clk_reset_n"] = In(1)
        # MacCLK rate selector (encodes target frequency).
        members["mac_clk_rate"] = In(5)
        # MAC requests the MacCLK output to be active.
        members["mac_clk_req"] = In(1)
        # PHY acknowledges and delivers MacCLK.
        members["mac_clk_ack"] = Out(1)
        # MacCLK output from the PHY to the MAC.
        members["mac_clk"] = Out(1)

        super().__init__(members)


class PIPEDebugSignature(Signature):
    """Debug and status outputs for PIPE adapter monitoring.

    All members are ``Out`` — the adapter drives them so that external
    logic (ILA probes, LED drivers, or a debug register file) can observe
    internal state without perturbing operation.

    FSM state encodings are adapter-defined; consumers should treat them
    as opaque values unless documented otherwise.
    """

    def __init__(self):
        super().__init__(
            {
                # Current PIPE power state (P0–P3 encoding).
                "power_state": Out(4),
                # Rate-change FSM state (idle / requesting / waiting / done …).
                "rate_fsm_state": Out(4),
                # Receiver-detect FSM state.
                "rxdet_fsm_state": Out(4),
                # LFPS (Low-Frequency Periodic Signaling) FSM state.
                "lfps_fsm_state": Out(4),
                # Message-bus controller FSM state.
                "msgbus_fsm_state": Out(4),
                # DRP arbiter — which sub-controller currently owns the bus.
                "drp_mux_owner": Out(4),
                # DRP bus is locked by the current owner (multi-write in progress).
                "drp_mux_locked": Out(1),
                # A DRP transaction is in progress (read or write).
                "drp_busy": Out(1),
                # PLL lock indicator from the PHY.
                "pll_lock": Out(1),
                # CDR (Clock and Data Recovery) lock indicator from the PHY.
                "cdr_lock": Out(1),
                # Init FSM done flag — asserted once power-on CSR init completes.
                "init_done": Out(1),
                # Init FSM state (0=IDLE, 1=WRITE, 2=DONE).
                "init_fsm_state": Out(4),
                # GowinSerDes UPAR arbiter internal state (2-bit).
                # Useful for anti-sweep trees and low-level debug.
                "serdes_arb_state": Out(2),
                # RX FIFO almost-empty flag from SerDes lane.
                "rx_fifo_aempty": Out(1),
                # RX FIFO empty flag from SerDes lane.
                "rx_fifo_empty": Out(1),
            }
        )
