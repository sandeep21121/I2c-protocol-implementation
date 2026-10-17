module I2C #(
    parameter SYSTEM_CLK_HZ = 100_000_000,
    parameter I2C_CLK_HZ    = 100_000,
    parameter ADDR_WIDTH    = 8,
    parameter DATA_WIDTH    = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire start,

    input  wire [ADDR_WIDTH-1:0] Address_input,
    input  wire [DATA_WIDTH-1:0] write_data,

    input  wire SDA_i,

    output reg  SCL_o,
    output reg  SDA_o,

    output reg  [DATA_WIDTH-1:0] read_data,
    output wire busy,
    output wire done
);

    // SCL divider
    localparam DIV = SYSTEM_CLK_HZ / (2 * I2C_CLK_HZ);

    reg [$clog2(DIV+1)-1:0] clk_count;
    wire tick = (clk_count == DIV-1);

    // FSM states
    localparam IDLE      = 3'd0,
               START     = 3'd1,
               ADDRESS   = 3'd2,
               ADDR_ACK  = 3'd3,
               WRITE     = 3'd4,
               WRITE_ACK = 3'd5,
               READ      = 3'd6,
               M_ACK     = 3'd7;

    reg [2:0] state;
    reg [1:0] phase;

    reg [ADDR_WIDTH-1:0] addr_reg;
    reg [DATA_WIDTH-1:0] data_reg;
    reg [DATA_WIDTH-1:0] read_reg;

    reg [$clog2(ADDR_WIDTH+1)-1:0] addr_bit;
    reg [$clog2(DATA_WIDTH+1)-1:0] data_bit;
    reg read_write;

    // SDA_o and SCL_o: 1 = release, 0 = pull LOW
    assign busy = (state != IDLE);
    assign done = (state == IDLE);

    always @(posedge clk) begin
        if (rst) begin
            clk_count <= 0;
            state     <= IDLE;
            phase     <= 0;

            SCL_o     <= 1;
            SDA_o     <= 1;

            addr_reg  <= 0;
            data_reg  <= 0;
            read_reg  <= 0;
            read_data <= 0;

            addr_bit  <= 0;
            data_bit  <= 0;
            read_write <= 0;
        end
        else begin

            // Clock divider
            if (state == IDLE) begin
                clk_count <= 0;
            end
            else if (tick) begin
                clk_count <= 0;
            end
            else begin
                clk_count <= clk_count + 1'b1;
            end

            // Start a transaction
            if (state == IDLE) begin
                SCL_o <= 1;
                SDA_o <= 1;
                phase <= 0;

                if (start) begin
                    state      <= START;
                    addr_reg   <= Address_input;
                    read_write <= Address_input[0];
                    addr_bit   <= 0;
                end
            end

            else if (tick) begin

                case (state)

                    // START: SDA falls while SCL is HIGH
                    START: begin
                        if (phase == 0) begin
                            SDA_o <= 0;
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            SDA_o <= addr_reg[ADDR_WIDTH-1];
                            phase <= 0;
                            state <= ADDRESS;
                        end
                    end

                    // Send address, MSB first
                    ADDRESS: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            phase <= 0;

                            if (addr_bit == ADDR_WIDTH-1) begin
                                state <= ADDR_ACK;
                                SDA_o <= 1;
                            end
                            else begin
                                addr_reg <= addr_reg << 1;
                                addr_bit <= addr_bit + 1'b1;
                                SDA_o <= addr_reg[ADDR_WIDTH-2];
                            end
                        end
                    end

                    // Receive slave ACK
                    ADDR_ACK: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            phase <= 0;

                            if (SDA_i == 0) begin
                                if (read_write) begin
                                    state   <= READ;
                                    data_bit <= 0;
                                    read_reg <= 0;
                                    SDA_o   <= 1;
                                end
                                else begin
                                    state    <= WRITE;
                                    data_reg <= write_data;
                                    data_bit <= 0;
                                    SDA_o    <= write_data[DATA_WIDTH-1];
                                end
                            end
                            else begin
                                state <= M_ACK;
                                SDA_o <= 0;
                            end
                        end
                    end

                    // Send data, MSB first
                    WRITE: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            phase <= 0;

                            if (data_bit == DATA_WIDTH-1) begin
                                state <= WRITE_ACK;
                                SDA_o <= 1;
                            end
                            else begin
                                data_reg <= data_reg << 1;
                                data_bit <= data_bit + 1'b1;
                                SDA_o <= data_reg[DATA_WIDTH-2];
                            end
                        end
                    end

                    // Receive slave ACK after data
                    WRITE_ACK: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            phase <= 0;
                            SDA_o <= 0;
                            state <= M_ACK;
                        end
                    end

                    // Receive data, MSB first
                    READ: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            phase <= 0;

                            read_reg <= {
                                read_reg[DATA_WIDTH-2:0], SDA_i
                            };

                            if (data_bit == DATA_WIDTH-1) begin
                                state <= M_ACK;
                                SDA_o <= 1; // NACK after one byte
                            end
                            else begin
                                data_bit <= data_bit + 1'b1;
                            end
                        end
                    end

                    // Master ACK/NACK pulse, then STOP
                    M_ACK: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            phase <= 1;
                        end
                        else begin
                            SCL_o <= 0;
                            SDA_o <= 0;
                            phase <= 0;
                            state <= STOP;
                        end
                    end

                    // STOP: SDA rises while SCL is HIGH
                    STOP: begin
                        if (phase == 0) begin
                            SCL_o <= 1;
                            SDA_o <= 0;
                            phase <= 1;
                        end
                        else begin
                            SDA_o <= 1;
                            SCL_o <= 1;
                            read_data <= read_reg;
                            phase <= 0;
                            state <= IDLE;
                        end
                    end

                    default: begin
                        state <= IDLE;
                        SCL_o <= 1;
                        SDA_o <= 1;
                        phase <= 0;
                    end

                endcase
            end
        end
    end

endmodule
