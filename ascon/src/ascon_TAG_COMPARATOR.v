// ============================================================
// Module: ascon_TAG_COMPARATOR
// Description: Constant-time comparison of computed tag vs
//              received tag. Used in decryption for authenticity.
// ============================================================
module ascon_TAG_COMPARATOR (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         compare,        // pulse: trigger comparison
    input  wire [127:0] tag_computed,   // from TAG_GENERATOR
    input  wire [127:0] tag_received,   // from external (cipher input)

    output reg          tag_match,      // 1 = authentic
    output reg          tag_done        // comparison complete
);

    // Constant-time XOR comparison (avoid timing side-channel)
    wire [127:0] diff;
    wire         all_zero;

    assign diff     = tag_computed ^ tag_received;
    assign all_zero = (diff == 128'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tag_match <= 1'b0;
            tag_done  <= 1'b0;
        end else if (compare) begin
            tag_match <= all_zero;
            tag_done  <= 1'b1;
        end else begin
            tag_done  <= 1'b0;
        end
    end

endmodule