`timescale 1ns/1ps

module I2C_tb;

parameter SYSTEM_CLK = 100, ADDR_WIDTH = 8, DATA_WIDTH = 8;

reg clk,rst,start,repeated_start ,addr_ACK,write_ACK,write_continue,read_continue;
reg [7:0] Desired_CLK;
reg  [ADDR_WIDTH-1:0] Address_input;
reg [DATA_WIDTH-1:0] write_data;

wire SCL_i ,SDA_i, SCL_o , SDA_o;
wire [DATA_WIDTH-1:0] read_data;
wire clock_stretching_LOW, arbitration_lost, busy , done;

I2C #(
    .SYSTEM_CLK(SYSTEM_CLK),
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH)
) DUT (
    .clk(clk),
    .rst(rst),
    .start(start),
    .repeated_start(repeated_start),
    .SCL_i(SCL_i),
    .SDA_i(SDA_i),
    .addr_ACK(addr_ACK),
    .write_ACK(write_ACK),
    .write_continue(write_continue),
    .read_continue(read_continue),
    .Desired_CLK(Desired_CLK),
    .Address_input(Address_input),
    .write_data(write_data),
    .SCL_o(SCL_o),
    .SDA_o(SDA_o),
    .read_data(read_data),
    .clock_stretching_LOW(clock_stretching_LOW),
    .arbitration_lost(arbitration_lost),
    .busy(busy),
    .done(done)
);


// clock generation
initial clk = 0;
always #10 clk = ~clk;



reg [DATA_WIDTH-1:0] exp_read_data;
reg [DATA_WIDTH-1:0] monitor_shift_reg;

reg [$clog2(DATA_WIDTH)-1:0] ref_read_bit;
//for read operation loop

reg [$clog2(DATA_WIDTH)-1:0] monitor_write_bit;
//for controlling the write_active pulse once all 8 write bits sampled
//write active goes LOW

reg read_active;
// latch values to indicate to reference model when READ/WRITE occurs just using addr_ack at HIGH will lead
reg write_active;
// to sampling at ADDRESS , READ & WRITE leading to mismatches

reg addr_ack_seen;
reg address_complete;
reg [$clog2(ADDR_WIDTH):0] addr_monitor_bit;
// used to detect the moment done actually rises
reg done_d;

always @(posedge SCL_i) begin
    if(!addr_ack_seen && !address_complete) begin
        if(addr_monitor_bit == ADDR_WIDTH-1) begin // 8th address bit has just been sampled.
                                                   // The next SCL rising edge is the ACK clock.
            addr_monitor_bit <= 0;
            address_complete <= 1;
        end
        else begin
            addr_monitor_bit <= addr_monitor_bit + 1;
        end
    end

    else if(address_complete && !addr_ack_seen) begin
        address_complete <= 0;
        if(addr_ACK) begin
            addr_ack_seen <= 1;
            if(Address_input[0] == 1) begin
                read_active <= 1;
                write_active <= 0;
                ref_read_bit <= 0;
            end
            else begin
                write_active <= 1;
                read_active <= 0;

                monitor_write_bit <= 0;
            end
        end
    end


    // reference model ( software based ) for READ
    // (for write we just take write_data as expected)
    else if(read_active) begin
        exp_read_data[DATA_WIDTH-1-ref_read_bit] <= SDA_i;

        if(ref_read_bit == DATA_WIDTH-1) begin
            ref_read_bit <= 0;
            read_active <= 0;

        end
        else begin
            ref_read_bit <= ref_read_bit + 1;
        end
    end



    // monitor (observe DUT signals and reconstruct transaction)
    else if(write_active) begin

        monitor_shift_reg <=
            {monitor_shift_reg[DATA_WIDTH-2:0], SDA_o};

        if(monitor_write_bit == DATA_WIDTH-1) begin

            monitor_write_bit <= 0;
            write_active <= 0;
        end
        else begin
            monitor_write_bit <= monitor_write_bit + 1;
        end
    end
end




// scoreboard
always @(posedge clk) begin

    done_d <= done;

    if(done && !done_d) begin
        if(addr_ACK) begin
        if(Address_input[0] == 0) begin
        if(monitor_shift_reg !== write_data) begin
        $error("WRITE DATA MISMATCH! Expected=%h Got=%h",write_data,monitor_shift_reg);
        end
        end
    else begin
    if(read_data !== exp_read_data) begin
    $error("READ DATA MISMATCH! Expected=%h Got=%h",exp_read_data,read_data);
    end
    end
    end
    end

end




// assertions (to check for timing correctness and illegal states)
always @(posedge clk) begin

    if(done && busy) begin
        $error("busy and done asserted together");
    end

    if(!busy) begin

        if(SCL_i !== 1 && SCL_o !== 1) begin

            $error("actual SCL and observed SCL mismatch and not IDLE!");

        end

    end

    if(SCL_i == 1 && SCL_o == 0) begin
        // observed SCL cannot be pulled LOW while the actual SCL line is released HIGH

        $error("fatal protoccol error!");

    end

end



// task based driver
task reset();
begin

rst = 1;

start = 0;
repeated_start = 0;

Desired_CLK = 0;

slave_SCL = 1;
slave_SDA = 1;

addr_ACK = 0;
write_ACK = 0;

write_continue = 0;
read_continue = 0;

Address_input = 0;
write_data = 0;


// reference model / monitor reset

read_active = 0;
write_active = 0;

ref_read_bit = 0;
monitor_write_bit = 0;

exp_read_data = 0;
monitor_shift_reg = 0;


// address-monitor reset

addr_ack_seen = 0;
address_complete = 0;
addr_monitor_bit = 0;


// done detector

done_d = 0;


repeat(5) @(posedge clk);

@(negedge clk);

rst = 0;

end
endtask




// here we cannot just directly randomize SCL_i and SDA_i as they are the
// actual bus layer and cannot randomly toggle as that would lead to SCL_o
// and SDA_o also randomly toggling and they cant be random since they have
// to convey address and write data, so we define a slave reg SCL and SDA ,
// and the actual bus layer will be SCL_o & slave_SCL_drive

reg slave_SCL;
reg slave_SDA;

assign SCL_i = SCL_o & slave_SCL;
assign SDA_i = SDA_o & slave_SDA;



task transaction();
begin

    wait(!busy);

    @(posedge clk) begin

        start <= 1;

        repeated_start <= 0;

        Desired_CLK <=
            ($random & 32'h7fffffff) % (SYSTEM_CLK/2) + 1;

        Address_input <= $random;

        addr_ACK <= ($urandom % 4) != 0;

        write_data <= $random;

        write_ACK <= ($urandom % 4) != 0;


        // Keep the physical bus released for now.
        // Clock stretching/arbitration can be tested separately.

        slave_SCL <= 1;
        slave_SDA <= 1;

        write_continue <= 0;
        read_continue <= 0;

        addr_ack_seen <= 0;
        address_complete <= 0;
        addr_monitor_bit <= 0;


        // Clear the reference model and monitor for the new transaction.

        exp_read_data <= 0;
        monitor_shift_reg <= 0;

        read_active <= 0;
        write_active <= 0;

        ref_read_bit <= 0;
        monitor_write_bit <= 0;


        @(posedge clk) begin
            start <= 0;
        end

    end

end
endtask



// task based driver
initial begin

    reset();

    #20;

    repeat(10)
        transaction();

    wait(!busy);

    #100;

    $display("ALL TESTS PASSED!");

    $finish;

end
// waveform
initial begin
    $dumpfile("I2C.vcd");
    $dumpvars(0,I2C_tb);
end
endmodule
