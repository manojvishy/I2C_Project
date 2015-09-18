/*
i2c Master

Manoj Vishwanathan

Description:
  i2c Master
*/
module i2c_master
    #(
        parameter ADDR_BYTES = 1,
        parameter DATA_BYTES = 2,
        parameter REG_ADDR_WIDTH = 8 * ADDR_BYTES
    )(
        input  clk,            // System clock
        input  reset,          // Reset signal
        input  [11:0] clk_div, // Clock divider value to configure SCL from the system clock

        input open_drain, // Open drain

        input  sda_in,    // SDA Input
        output sda_out,   // SDA Output
        output sda_oen,   // SDA Output Enable

        input  scl_in,    // SCL Input
        output scl_out,   // SCL Output
        output scl_oen,   // SCL Output Enable

        input [6:0] chip_addr, // Slave Address
        input [REG_ADDR_WIDTH - 1:0] reg_addr, // Register address

        input write_en,    // Write enable
        input write_mode,  // Write mode (0: single, 1: multi-byte)
        input read_en,     // Read enable

        output reg [8 * DATA_BYTES - 1:0] data_out,   // Data to write to register
        input [8 * DATA_BYTES - 1:0] data_in,         // Data read from register

        output reg [ST_WIDTH - 1:0] status,
        output reg done,
        output reg busy
    );

    localparam ST_WIDTH = 1 + ADDR_BYTES + DATA_BYTES;
    localparam SR_WIDTH = 8 * ST_WIDTH;

    // State Machine States
    localparam s_idle         = 0,
               s_start_write  = 1,
               s_start_read   = 2,
               s_stop         = 3,
               s_shift_out    = 4,
               s_shift_in     = 5,
               s_send_ack     = 6,
               s_send_nack    = 7,
               s_rcv_ack      = 8;


    // Interal registers
    reg [SR_WIDTH-1:0] sr;   // Shift register
    reg [5:0] sr_count;      // Location within shift register

    wire [2:0] byte_count;	// Internal Variable to count the no. of bytes being transmitted

    reg [1:0] scl_count;	//Counting the no. of clock
    reg [11:0] clk_count;

    reg sda_reg, oen_reg, sda_s, scl_s;
    reg writing, reading, in_prog;

    // FSM state
    reg [3:0] state;

    assign sda_out = sda_reg;			//SDA reg is assigned to the out port every 
    assign sda_oen = oen_reg;
    assign scl_out = open_drain ? 1'b0 : scl_count[1];
    assign scl_oen = open_drain ? scl_count[1] : 1'b0;
    assign byte_count = sr_count[5:3];

    always @ (posedge clk or negedge reset) begin	// On Positive clock edge or on reset being pulled down we will start the State Machine
        if (~reset) begin		// Reset Mode.. Reset state Machine to IDLE state
            state     <= s_idle;

            sda_reg   <= 1'b1;
            oen_reg   <= 1'b1;

            sr_count  <= 6'b00_0000;
            sr        <= 24'hFFF;

            writing   <= 1'b1;
            reading   <= 1'b0;
            in_prog   <= 1'b0;

            status    <= 1'b0;
            done      <= 1'b0;
            busy      <= 1'b0;

            data_out  <= 1'b0;

            scl_count <= 2'b10;
            clk_count <= 12'b0000_0000_0000;
        end
        else begin				// ELSE begin by scanning the input lines This else case is where the entire state machine is evaluated
            sda_s <= sda_in;
            scl_s <= scl_in;

            if (state == s_idle) begin		// IF state is IDLE we set the final done bit to zero 
                done <= 1'b0;
                sr_count <= 6'b00_0000;		// The Shift register is set to zero based on it being not shifted out any bits

                if (~write_mode) begin		//  If not in the write Mode set the in_prog bit to zero which in the read mode
                    in_prog <= 1'b0;

                    if (in_prog) begin			// If In-prog bit is set after this point we have hit the stop state of the FSM we need to pull down the 
                        state <= s_stop;
                        sda_reg <= 1'b0;
                        oen_reg <= 1'b0;
                    end
                    else begin					// If not in progress we set the start condition on the bus to begin reading the the data.
                        sda_reg <= open_drain ? 1'b0 : 1'b1;
                        oen_reg <= 1'b1;
                        clk_count <= 0;
                    end
                end

                if (in_prog) begin			// IN Idle state if the in_prog bit is set 
                    scl_count <= 2'b00;		// SCL_COUNT is set to 00 as we need to send out the shift register

                    sr <= {data_in, {SR_WIDTH - 8 * DATA_BYTES{1'b0}}};		// Append the shift register 24 bits of zeroes
                end
                else begin					// If not in IDLE state we need to check for other conditions
                    scl_count <= 2'b10;		//set the clk to 2 and we read the data in 

                    if (ADDR_BYTES == 0) begin		// If there is only data to be sent and location has been locked in we send just the data
                        sr <= {chip_addr, 1'b0, data_in};
                    end
                    else begin
                        sr <= {chip_addr, 1'b0, reg_addr, data_in};		//The chip address and write register address in the write mode.
                    end
                end

                if (write_en) begin				// From Idle state if write_enable is set depending on weather or not we are making progress 
                    state   <= in_prog ? s_shift_out : s_start_write;	//Next state is shift _out or s_start_write
                    writing <= 1'b1;		//writing in progress
                    status  <= 1'b0;		
                    busy    <= 1'b1;		//I2C module busy for Micro controller Signal
                end
                else if (read_en) begin			//Else if read is enabled the next state is either start_read or Start_write the address to be read from in the slave
                    state    <= ADDR_BYTES == 0 ? s_start_read : s_start_write;
                    writing  <= 1'b0;
                    reading  <= 1'b0;
                    status   <= 1'b0;
                    busy     <= 1'b1;
                end
                else begin
                    busy     <= 1'b0;
                end
            end
            else begin
                if (clk_count == clk_div) begin		// Clock divider Circuit
                    clk_count <= 2'b00;
                    scl_count <= scl_count + 1'b1;

                    case (state)
                        s_start_write: begin		// From Idle state we move to start irrespective of weather we need to do a read or a write
                            state   <= s_shift_out; // Next state is to shift out on the SDA
                            sda_reg <= 1'b0;
                            oen_reg <= 1'b0;
                        end

                        s_start_read: begin			// From Start read we start to shift out the chip address to check if slave acknowledges
                            if (scl_count == 2'b10) begin
                                state    <= s_shift_out;
                                sda_reg  <= 1'b0;
                                oen_reg  <= 1'b0;
                                sr       <= {chip_addr, 1'b1, {8 * (ADDR_BYTES + DATA_BYTES){1'b0}}};
                                sr_count <= 6'b00_0000;
                                reading  <= 1'b1;
                            end
                        end

                        s_stop: begin						//DEpending on the state of the transition we can set the next state to Idle and generate the
                            if (scl_count == 2'b10) begin	//stop condition on the SDA bus.
                                state   <= s_idle;
                                sda_reg <= open_drain ? 1'b0 : 1'b1;
                                oen_reg <= 1'b1;
                                done    <= 1'b1;
                            end
                        end

                        s_shift_out: begin
                            if (scl_count == 2'b00) begin
                                if ((sr_count[2:0]) == 3'b000 && (|sr_count)) begin	//Check if we are done sending Set the next state to check for Recv and 
                                    state   <= s_rcv_ack;							// generate a stop condition on the bus
                                    sda_reg <= open_drain ? 1'b0 : 1'b1;
                                    oen_reg <= 1'b1;
                                end
                                else begin											// If we have not completed the send wet set the shift register's MSB 
                                    sda_reg  <= open_drain ? 1'b0 : sr[SR_WIDTH - 1]; // to the SDA line and we left shift nd append it with a one we also increment 
                                    oen_reg  <= open_drain ? sr[SR_WIDTH - 1] : 1'b0; //the shift register counter
                                    sr       <= {sr[SR_WIDTH - 2:0], 1'b1};
                                    sr_count <= sr_count + 1'b1;
                                end
                            end
                        end

                        s_shift_in: begin
                            if (scl_count == 2'b00) begin
                                if (sr_count == 8 * (DATA_BYTES + 1)) begin		// Data bits is equal to DATAbytes + 1 Address Byte 
                                    state   <= s_send_nack;
                                    sda_reg <= open_drain ? 1'b0 : 1'b1;
                                    oen_reg <= 1'b1;
                                end
                                else if (sr_count[2:0] == 3'b000) begin			//Hadling if no Data has been received end by checking ACK
                                    state   <= s_send_ack;
                                    sda_reg <= 1'b0;
                                    oen_reg <= 1'b0;
                                end
                            end
                            else if (scl_count == 2'b01) begin
                                data_out <= {data_out[8 * DATA_BYTES - 2:0], sda_s};	// Data is passed to the Microcontroller through the data out parallel lines
                                sda_reg  <= open_drain ? 1'b0 : 1'b1;
                                oen_reg  <= 1'b1;
                                sr_count <= sr_count + 1'b1;
                            end
                        end

                        s_send_ack: begin					//Acknowledge state
                            if (scl_count == 2'b00) begin		// Continue shifting in Data
                                state   <= s_shift_in;
                                sda_reg <= open_drain ? 1'b0 : 1'b1;
                                oen_reg <= 1'b1;
                            end
                            else if (scl_count == 2'b01) begin
                                status <= {status[ST_WIDTH - 2:0], sda_s};	//Shifting in data
                            end
                        end

                        s_send_nack: begin
                            if (scl_count == 2'b00) begin	// Nack state is set then we terminat the state Machine to stop
                                state   <= s_stop;
                                sda_reg <= 1'b0;
                                oen_reg <= 1'b0;
                            end
                            else begin			// Else we send a stop condtion to the code.
                                sda_reg <= open_drain ? 1'b0 : 1'b1;
                                oen_reg <= 1'b1;
                            end
                        end

                        s_rcv_ack: begin
                            if (scl_count == 2'b00) begin
                                if (writing && ((byte_count == DATA_BYTES + ADDR_BYTES + 1 && ~in_prog) || (byte_count == DATA_BYTES && in_prog))) begin	// Once completed writing we try and chck 
                                    if (write_mode) begin
                                        state   <= s_idle;
                                        in_prog <= 1'b1;
                                        done    <= 1'b1;
                                    end else begin
                                        state   <= s_stop;
                                        sda_reg <= 1'b0;
                                        oen_reg <= 1'b0;
                                    end
                                end
                                else if (~writing && ~reading && (byte_count == ADDR_BYTES + 1)) begin	// If we are not writing and reading along with the byte count 
                                    state <= s_start_read;
                                end
                                else if (~writing && reading) begin
                                    state <= s_shift_in;
                                end
                                else begin
                                    state    <= s_shift_out;
                                    sda_reg  <= open_drain ? 1'b0 : sr[SR_WIDTH - 1];
                                    oen_reg  <= open_drain ? sr[SR_WIDTH - 1] : 1'b0;
                                    sr       <= {sr[SR_WIDTH - 2:0], 1'b1};
                                    sr_count <= sr_count + 1'b1;
                                end
                            end
                            else if (scl_count == 2'b01) begin
                                status <= {status[ST_WIDTH - 2:0], sda_s};
                            end
                        end
                    endcase
                end
                else begin
                    if (scl_count[1] == 1'b0 || scl_s == 1'b1) begin		//Indicates completion of a full clock cycle.
                        clk_count <= clk_count + 1'b1;
                    end
                end
            end
        end
    end
endmodule
