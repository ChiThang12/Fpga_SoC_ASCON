// ============================================================================
// Module  : sync_fifo
// Project : ASCON Crypto Accelerator IP
//
// Description:
//   Generic synchronous FIFO with optional FWFT (First-Word-Fall-Through).
//   - Full/empty flags are registered (no combinational glitch)
//   - Simultaneous push + pop when not empty/full is supported
//   - DATA_OUT is registered (read latency = 1 cycle after pop)
//   - DATA_OUT holds last value when empty
//   - [OPT] fwft_dout/fwft_valid: combinational read of FIFO head,
//           available immediately when !empty. Enables zero-latency consume.
//
// Parameters:
//   WIDTH : data width in bits
//   DEPTH : number of entries (must be power of 2)
//
// [FIX] Split mem[] write from pointer reset into separate always blocks.
//
// Root cause of Yosys "Multiple edge sensitive events" error:
//   The original write block `always @(posedge clk or negedge rst_n)` had
//   BOTH `mem[wr_idx] <= din` AND `wr_ptr` reset inside it.  Yosys tries
//   to infer $adff (async-reset DFF) for every signal in that sensitivity
//   list, including the mem[] array.  RAM/ROM arrays cannot be synthesised
//   as $adff; the tool hits an internal contradiction and aborts.
//
// Fix: mem[] lives in its own `always @(posedge clk)` block with NO reset
//   branch.  This maps cleanly to a standard $dff / BRAM primitive.
//   wr_ptr stays in its own `always @(posedge clk or negedge rst_n)` block
//   so it still resets correctly.  Functional behaviour is unchanged:
//   mem[] contents are undefined after rst_n but can never be consumed
//   until wr_ptr advances, which only happens on a valid push.
// ============================================================================

module sync_fifo #(
    parameter WIDTH = 64,
    parameter DEPTH = 4           // must be power of 2
) (
    input  wire             clk,
    input  wire             rst_n,

    // Write port
    input  wire [WIDTH-1:0] din,
    input  wire             push,
    output wire             full,

    // Read port (registered — 1 cycle latency after pop)
    output reg  [WIDTH-1:0] dout,
    input  wire             pop,
    output wire             empty,

    // FWFT port (combinational — zero latency, valid when fwft_valid=1)
    output wire [WIDTH-1:0] fwft_dout,
    output wire             fwft_valid,

    // Status
    output wire [$clog2(DEPTH):0] count   // number of entries currently stored
);

    localparam PTR_W = $clog2(DEPTH);

    // Assertion: DEPTH must be a power of 2
    initial begin
        if ((DEPTH & (DEPTH - 1)) != 0) begin
            $error("sync_fifo: DEPTH=%0d must be a power of 2", DEPTH);
            $finish;
        end
    end

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_W:0]   wr_ptr;   // one extra bit for full/empty distinction
    reg [PTR_W:0]   rd_ptr;

    wire [PTR_W-1:0] wr_idx = wr_ptr[PTR_W-1:0];
    wire [PTR_W-1:0] rd_idx = rd_ptr[PTR_W-1:0];

    assign full  = (wr_ptr == {~rd_ptr[PTR_W], rd_ptr[PTR_W-1:0]});
    assign empty = (wr_ptr == rd_ptr);

    // ── FWFT: combinational read of current head ─────────────────────────
    // Available immediately when FIFO is not empty. Consumer can latch
    // fwft_dout in the same cycle as checking fwft_valid, then pop next cycle.
    assign fwft_dout  = mem[rd_idx];
    assign fwft_valid = ~empty;
    assign count = wr_ptr - rd_ptr;

    // ── Write pointer — async reset, infers $adff ─────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr <= {(PTR_W+1){1'b0}};
        else if (push && !full)
            wr_ptr <= wr_ptr + 1'b1;
    end

    // ── Memory array — posedge clk ONLY, infers $dff / BRAM ──────────────
    // [FIX] No negedge rst_n here: mem[] cannot be an $adff.
    // Contents are undefined after reset but safe — wr_ptr == rd_ptr == 0
    // (empty) so no pop can occur until a push writes valid data first.
    always @(posedge clk) begin
        if (push && !full)
            mem[wr_idx] <= din;
    end

    // ── Read pointer and output — async reset, infers $adff ──────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= {(PTR_W+1){1'b0}};
            dout   <= {WIDTH{1'b0}};
        end else if (pop && !empty) begin
            dout   <= mem[rd_idx];
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule