`include "./param.v"

module systolic_ctrl(
    input CLK, RSTb,
    input [`BIT_INSTR-1:0] Instr_In,
    // For WriteBack(Read Path)
    output reg Flag_Finish_Out,
    output reg Valid_WB_Out,
    output reg [`BIT_PSUM-1:0] Data_WB_Out,
    // For SRAM Input Writing
    output reg [`PE_ROW-1:0] sram_input_we, // SRAM Write Enable
    output reg [`PE_ROW-1:0] sram_input_en , // SRAM Enable
    output reg [`PE_ROW*`BIT_ADDR-1:0] sram_input_addr, // 26112 Data, More than 15 Bit needed
    output reg [`PE_ROW*`BIT_DATA-1:0] sram_input_din, // Data to write
    // For SRAM Weight Writing
    output reg [`PE_COL-1:0] sram_weight_we , // SRAM Write Enable
    output reg [`PE_COL-1:0] sram_weight_en , // SRAM Enable
    output reg [`PE_COL*`BIT_ADDR-1:0] sram_weight_addr, // 26112 Data, More than 15 Bit needed
    output reg [`PE_COL*`BIT_DATA-1:0] sram_weight_din, // Data to write
    // For SRAM Psum Writing (A Port)
    output reg [`PE_COL-1:0] sram_psum_we_a, // SRAM Write Enable
    output reg [`PE_COL-1:0] sram_psum_en_a, // SRAM Enable
    output reg [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_a, // Address to write
    output reg [`PE_COL*`BIT_PSUM-1:0] sram_psum_din_a, // Data to write
    // For SRAM Psum Reading (B Port)
    output reg [`PE_COL-1:0] sram_psum_we_b, // SRAM Write Enable
    output reg [`PE_COL-1:0] sram_psum_en_b, // SRAM Enable
    output reg [`PE_COL*`BIT_ADDR-1:0] sram_psum_addr_b, // Address to read
    input [`PE_COL*`BIT_PSUM-1:0] sram_psum_dout_b, // Data to read
    // For Systolic Weight Control
    output reg [`BIT_ROW_ID-1:0] systolic_en_row_id, // Row ID for systolic array
    output reg [`PE_COL-1:0] systolic_en_w, // Enable for systolic array
    // For Systolic Psum In Control
    output reg [`PE_COL*`BIT_ADDR-1:0] systolic_addr_p, // Address for systolic array
    output [`PE_COL*`BIT_VALID-1:0] systolic_valid_p, // Valid for systolic array
    // For Stall
    output instr_stall,
    // For Psum SRAM get control
    output reg psum_sel_ctrl,
    // For Weight muxing
    output weight_to_systolic_sel,
    // For debug
    output [3:0] state_out // Debug output for state
    );

    // Reset signal
    wire RST;
    assign RST = ~RSTb;

    // Debug signal
    assign state_out = state; // Debug output for state

    // Stall signal
    assign instr_stall = (state != `FETCH);

    // Define the opcode, param, and data
    reg opvalid;
    reg [`BIT_OPCODE-1:0] opcode;
    reg [`BIT_PARAM-1:0] param;
    reg [`BIT_SEL-1:0] sel;
    reg [`BIT_ADDR-1:0] addr;
    reg [`BIT_DATA-1:0] data;

    // Instruction register
    // reg [`BIT_INSTR-1:0] instr_reg;

    // Define Parameters
    reg [`BIT_PARAM-1:0] Param_S;
    reg [`BIT_PARAM-1:0] Param_OC;
    reg [`BIT_PARAM-1:0] Param_IC;
    reg [`BIT_PARAM-1:0] Param_TRG;

    // State Variables
    reg [`BIT_STATE:0] state, next_state, next_task_state, previous_state;

    // For PSUM INIT
    reg [`BIT_ADDR-1:0] psum_init_addr; // Psum init count

    // IC, OC for Execute
    reg [`BIT_PARAM:0] State_IC;
    reg [`BIT_PARAM:0] State_OC;
    reg [`BIT_PARAM:0] Next_IC;
    reg [`BIT_PARAM:0] Next_OC;
    reg [`BIT_PARAM:0] Run_IC; // # of Enabled IC (PE Rows)
    reg [`BIT_PARAM:0] Run_OC; // # of Enabled OC (PE Cols)

    wire [`BIT_PARAM:0] Base_I_calc = Param_S * (State_IC / `PE_ROW);
    wire [`BIT_PARAM:0] Base_W_calc = State_IC + Param_IC * (State_OC / `PE_COL);

    // Count
    reg [3:0] load_cnt; // Load count for systolic array
    reg [7:0] run_cnt; // Run count for systolic array
    reg [31:0] clk_cnt; // Clock count for systolic array

    // 
    reg [`PE_COL*`BIT_VALID-1:0] systolic_valid_p_raw; // Valid for systolic array
    reg [`PE_COL*`BIT_VALID-1:0] systolic_valid_p_reg;
    reg [`BIT_ADDR-1:0] pattern_array [0:`PE_ROW-1]; // Pattern array for systolic array
    reg [`BIT_ADDR-1:0] pattern_array_d [0:`PE_ROW-1]; // Pattern array for systolic array
    assign weight_to_systolic_sel = (load_cnt < Run_IC+1) ? 1'b1 : 1'b0; // Weight to systolic array

    // Masking
    always @(posedge CLK) begin
        systolic_valid_p_reg <= systolic_valid_p_raw & ((1 << Run_OC) - 1);
    end

    assign systolic_valid_p = systolic_valid_p_reg;

    // Loop Variables
    integer i;

    // Output / action logic
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            // Reset all outputs to default values
            sram_input_we <= 0;
            sram_input_en <= 0;
            sram_input_addr <= 0;
            sram_input_din <= 0;
            sram_weight_we <= 0;
            sram_weight_en <= 0;
            sram_weight_addr <= 0;
            sram_weight_din <= 0;
            systolic_en_row_id <= 0;
            systolic_en_w <= 0;
            systolic_addr_p <= 0;
            // systolic_valid_p <= 0;
            // instr_stall <= 1;
            psum_sel_ctrl <= 0;
            Flag_Finish_Out <= 0;
            Valid_WB_Out <= 0;
            Data_WB_Out <= 0;
            psum_init_addr <= 0;
            load_cnt <= 0;
            run_cnt <= 0;
            State_IC <= 0;
            State_OC <= 0;
            Next_IC <= 0;
            Next_OC <= 0;
            Run_IC <= 0;
            Run_OC <= 0;
            // Base_W <= 0;
            // Base_I <= 0;
            Param_S <= 0;
            Param_OC <= 0;
            Param_IC <= 0;
            Param_TRG <= 0;
            // instr_reg <= 0;
            opcode <= 0;
            param <= 0;
            sel <= 0;
            addr <= 0;
            data <= 0;
            opvalid <= 0;
            previous_state <= 0;
            next_task_state <= 0;
            next_state <= 0;
            sram_input_we <= 0;
            sram_input_en <= 0;
            sram_input_addr <= 0;
            sram_input_din <= 0;
            sram_weight_we <= 0;
            sram_weight_en <= 0;
            sram_weight_addr <= 0;
            sram_weight_din <= 0;
            sram_psum_en_a <= 0;
            sram_psum_we_a <= 0;
            sram_psum_addr_a <= 0;
            sram_psum_din_a <= 0;
            sram_psum_en_b <= 0;
            sram_psum_we_b <= 0;
            sram_psum_addr_b <= 0;
            systolic_valid_p_raw <= 0;
        end else begin
            // main FSM action
            case(state)
                `IDLE: begin
                    // Reset all outputs to default values
                end
                `FETCH: begin
                    // Fetch instruction and update outputs accordingly
                    Valid_WB_Out <= 0; // Clear valid flag
                    sram_input_we <= 0; // Disable write to Input SRAM
                    sram_input_en <= 0; // Disable Input SRAM
                    sram_weight_en <= 0; // Disable Weight SRAM
                    sram_weight_we <= 0; // Disable write to Weight SRAM
                end
                `DECODE: begin
                    // Decode instruction and update outputs accordingly
                    // instr_reg <= Instr_In; // Store instruction in register
                    opvalid <= Instr_In[(`BIT_DATA + `BIT_PARAM + `BIT_OPCODE) +: `BIT_VALID]; // Extract valid bit from instruction
                    opcode <= Instr_In[(`BIT_DATA + `BIT_PARAM) +: `BIT_OPCODE]; // Extract opcode from instruction
                    param <= Instr_In[`BIT_DATA +: `BIT_PARAM]; // Extract param from instruction
                    sel <= Instr_In[`BIT_DATA + `BIT_ADDR +: `BIT_SEL]; // Extract select bits from instruction
                    addr <= Instr_In[`BIT_DATA +: `BIT_ADDR]; // Extract address from instruction
                    data <= Instr_In[0 +: `BIT_DATA]; // Extract data from instruction
                end
                `EXECUTE: begin
                    // Execute instruction and update outputs accordingly
                end
                `PARAM_SET: begin
                    // Set parameters and update outputs accordingly
                    case (param)
                        // `PARAM_BASE_WSRAM: begin
                        //     // Set base address for WSRAM
                        //     Base_W <= data;
                        // end
                        `PARAM_S: begin
                            // Set S parameter
                            Param_S <= data;
                        end
                        `PARAM_OC: begin
                            // Set OC parameter
                            Param_OC <= data;
                        end
                        `PARAM_IC: begin
                            // Set IC parameter
                            Param_IC <= data;
                        end
                        `PARAM_TRG: begin
                            // Set target for SRAM (Input, Weight, Psum)
                            Param_TRG <= data;
                        end
                        `PARAM_IC_WH: begin
                            // Set Higher bits of IC parameter
                            Param_IC <= {data, Param_IC[`BIT_DATA-1:0]}; // Concatenate higher bits with existing IC parameter
                        end
                        // `PARAM_BASE_WSRAM_WH: begin
                        //     // Set Higher bits of base address for WSRAM
                        //     Base_W <= {data, Base_W[`BIT_DATA-1:0]}; // Concatenate higher bits with existing base address                
                        // end
                        default: begin 
                            // Default case, do nothing or reset outputs as needed 
                        end
                    endcase
                end
                `WRITE_SRAM: begin
                    // Write to SRAM and update outputs accordingly
                    case (Param_TRG)
                        `TRG_ISRAM: begin
                            // Write to Input SRAM
                            sram_input_we <= (1 << sel); // Enable write to SRAM
                            sram_input_en <= (1 << sel); // Enable SRAM
                            sram_input_addr[sel*`BIT_ADDR+:`BIT_ADDR] <= addr; // Address to write
                            sram_input_din[sel*`BIT_DATA+:`BIT_DATA] <= data; // Data to write
                        end
                        `TRG_WSRAM: begin
                            // Write to Weight SRAM
                            sram_weight_we <= (1 << sel); // Enable write to SRAM
                            sram_weight_en <= (1 << sel); // Enable SRAM
                            sram_weight_addr[sel*`BIT_ADDR+:`BIT_ADDR] <= Base_W_calc + addr; // Address to write
                            sram_weight_din[sel*`BIT_DATA+:`BIT_DATA] <= data; // Data to write
                        end
                        `TRG_PSRAM: begin
                            // Write to Psum SRAM (if applicable)
                            sram_psum_we_a <= (1 << sel); // Enable write to SRAM
                            sram_psum_en_a <= (1 << sel); // Enable SRAM
                            sram_psum_addr_a[sel*`BIT_ADDR+:`BIT_ADDR] <= addr; // Address to write
                            sram_psum_din_a[sel*`BIT_DATA+:`BIT_DATA] <= data; // Data to write
                        end
                        default: begin 
                            // Default case, do nothing or reset outputs as needed 
                        end
                    endcase
                end
                `READ_SRAM: begin
                    // Read from SRAM and update outputs accordingly
                end
                // Execute
                `INIT_PSUM: begin
                    // Initialize Psum SRAM
                    sram_psum_we_a <= 4'b1111; // Enable write to Psum SRAM
                    sram_psum_en_a <= 4'b1111; // Enable Psum SRAM
                    sram_psum_addr_a <= {`PE_COL{psum_init_addr}}; // Address to write
                    sram_psum_din_a <= {`PE_COL{8'b0}}; // Data to write
                    psum_init_addr <= psum_init_addr + 1; // Increment the count for next address
                end
                `SET: begin
                    // Set up for execution and update outputs accordingly
                    // Reset load count and enable signals
                    load_cnt <= 0; // Reset load count
                    run_cnt <= 0; // Reset run count
                    // Load IC/OC status
                    State_IC <= Next_IC;
                    State_OC <= Next_OC;
                    // Determine Run_OC
                    if (Next_OC + `PE_COL <= Param_OC)
                        Run_OC <= `PE_COL;
                    else
                        Run_OC <= Param_OC - Next_OC;
                    // Determine Run_IC
                    if (Next_IC + `PE_ROW <= Param_IC)
                        Run_IC <= `PE_ROW;
                    else
                        Run_IC <= Param_IC - Next_IC;
                    // Next Index
                    if (Next_IC + `PE_ROW < Param_IC) begin
                        Next_IC <= Next_IC + `PE_ROW; // # of Enabled IC (PE Rows)
                        Next_OC <= Next_OC; // # of Enabled OC (PE Cols)
                    end else if (Next_OC + `PE_COL < Param_OC) begin
                        Next_IC <= 0; // # of Enabled IC (PE Rows)
                        Next_OC <= Next_OC + `PE_COL; // # of Enabled OC (PE Cols)
                    end else begin
                        Next_IC <= 0; // # of Enabled IC (PE Rows)
                        Next_OC <= 0; // # of Enabled OC (PE Cols)
                    end
                    // $display("[SET] Param_S=%0d, Param_OC=%0d, Param_IC=%0d", Param_S, Param_OC, Param_IC);
                    // $display("[SET] Base_I=%0d, Base_W=%0d, Run_IC=%0d, Run_OC=%0d", Base_I, Base_W, Run_IC, Run_OC);

                end
                `PRELOAD: begin
                    // 1. Enable Weight Read
                    sram_weight_en <= (Run_OC == 0) ? {`PE_COL{1'b0}} : ({`PE_COL{1'b1}} >> (`PE_COL - Run_OC));
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        sram_weight_addr[i*`BIT_ADDR +: `BIT_ADDR] <= (i < Run_OC) ? (Base_W_calc + load_cnt) : {`BIT_ADDR{1'b0}};
                    end
                end
                `LOAD: begin
                    // Load data and update outputs accordingly
                    load_cnt <= load_cnt + 1; // Increment load count
                    // 2. Systolic Weight Write Control
                    systolic_en_row_id <= load_cnt; // Row ID for systolic array
                    systolic_en_w <= (1 << `PE_COL) - 1; // Enable for systolic array
                    // $display("[LOAD] Base_W=%0d, load_cnt=%0d", Base_W, load_cnt);
                end
                `RUN: begin
                    // Execute the operation and update outputs accordingly
                    sram_psum_en_b <= 0; // Disable
                    // systolic_valid_p <= 0; // Disable
                    systolic_valid_p_raw <= 0; // Disable
                    //
                    if (run_cnt < Param_S + `PE_ROW + `PE_COL) begin
                        run_cnt <= run_cnt + 1; // Increment run count
                    end else begin
                        if (Next_IC == 0 && Next_OC == 0) begin
                            Flag_Finish_Out <= 1'b1; // Set finish flag
                        end else begin
                            Flag_Finish_Out <= 1'b0; // Clear finish flag
                        end
                    end
                    // 1. Enable Input SRAM Read
                    if (run_cnt < Param_S) begin
                        sram_input_we <= 0; // Disable write to Input SRAM
                        sram_input_en <= (Run_IC == 0) ? {`PE_ROW{1'b0}} : ({`PE_ROW{1'b1}} >> (`PE_ROW - Run_IC));
                        for (i = 0; i < `PE_ROW; i = i + 1) begin
                            sram_input_addr[i*`BIT_ADDR +: `BIT_ADDR] <= (i < Run_IC) ? (Base_I_calc + run_cnt) : {`BIT_ADDR{1'b0}};
                        end
                    end
                    // 2. Enable Psum SRAM Read
                    if(run_cnt < 10) begin
                        sram_psum_we_b <= 0; // Disable write to Psum SRAM
                        sram_psum_en_b <= (Run_OC == 0) ? {`PE_COL{1'b0}} : ({`PE_COL{1'b1}} >> (`PE_COL - Run_OC));
                        for (i = 0; i < `PE_COL; i = i + 1) begin
                            sram_psum_addr_b[i*`BIT_ADDR +: `BIT_ADDR] <= (i < Run_OC) ? (State_OC + pattern_array[i]) : {`BIT_ADDR{1'b0}};
                        end
                    end
                    // 3. Systolic Psum Write Control
                    for (i = 0; i < `PE_COL; i = i + 1) begin
                        systolic_addr_p[i*`BIT_ADDR +: `BIT_ADDR] <= (i < Run_OC) ? (State_OC + pattern_array_d[i]) : {`BIT_ADDR{1'b0}};
                    end                   
                    // 4. Set valid bits based on run_cnt
                    if (run_cnt < `PE_COL) begin
                        systolic_valid_p_raw <= (1 << (run_cnt + 1)) - 1;
                    end else if (run_cnt >= `PE_COL && run_cnt < 2 * `PE_COL - 1) begin
                        systolic_valid_p_raw <= ((1 << (2 * `PE_COL - run_cnt - 1)) - 1) << (run_cnt - `PE_COL + 1);
                    end else begin
                        systolic_valid_p_raw <= 0;
                    end
                
                end
                // Write Back
                `WRITE_BACK: begin
                    // Write back results and update outputs accordingly
                    sram_psum_we_b <= {`PE_COL{1'b0}}; // Disable write to Psum SRAM
                    sram_psum_en_b <= (1 << sel); // Enable Psum SRAM
                    sram_psum_addr_b[sel*`BIT_ADDR+:`BIT_ADDR] <= addr; // Address to Read
                    $display("[WRITE_BACK] sel=%0d, addr=%0d", sel, addr);
                end
                `WRITE_BACK_OUTPUT: begin
                    // Write back output data and update outputs accordingly
                    Valid_WB_Out <= 1'b1; // Set valid flag
                    Data_WB_Out <= sram_psum_dout_b[sel*`BIT_PSUM+:`BIT_PSUM]; // Output data from Psum SRAM
                    Flag_Finish_Out <= 1'b0; // Clear finish flag
                    $display("[WRITE_BACK_OUTPUT] addr=%0d, Data_WB_Out=%0d", addr, Data_WB_Out);
                end
                `WRITE_BACK_PARAM: begin
                    // Write back parameters and update outputs accordingly
                    case(param)
                        `PARAM_BASE_WSRAM: begin
                            // Write back base address for WSRAM
                            Data_WB_Out <= Base_W_calc; // Output base address
                            Valid_WB_Out <= 1'b1; // Set valid flag
                        end
                        `PARAM_S: begin
                            // Write back S parameter
                            Data_WB_Out <= Param_S; // Output S parameter
                            Valid_WB_Out <= 1'b1; // Set valid flag
                        end
                        `PARAM_OC: begin
                            // Write back OC parameter
                            Data_WB_Out <= Param_OC; // Output OC parameter
                            Valid_WB_Out <= 1'b1; // Set valid flag
                        end
                        `PARAM_IC: begin
                            // Write back IC parameter
                            Data_WB_Out <= Param_IC; // Output IC parameter
                            Valid_WB_Out <= 1'b1; // Set valid flag
                        end
                        `PARAM_TRG: begin
                            // Write back target for SRAM (Input, Weight, Psum)
                            Data_WB_Out <= Param_TRG; // Output target parameter
                            Valid_WB_Out <= 1'b1; // Set valid flag                
                        end
                        default: begin 
                            // Default case, do nothing or reset outputs as needed 
                        end
                    endcase
                end
                `SET_CTRL: begin
                    psum_sel_ctrl <= 1'b1; // Enable Psum SRAM control
                end
                `CLEAR_CTRL: begin
                    psum_sel_ctrl <= 1'b0; // Disable Psum SRAM control
                end
                default: begin 
                    // Default case, do nothing or reset outputs as needed 
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;

        if (RST) begin
            next_state = `IDLE; // Reset to IDLE state
        end else begin
            case(state)
                `IDLE                : begin
                    next_state = `FETCH; // Transition to FETCH state
                end
                `FETCH               : begin
                    next_state = `DECODE; // Transition to DECODE state
                end
                `DECODE              : begin
                    next_state = `EXECUTE; // Transition to EXECUTE state
                end
                `EXECUTE            : begin
                    if (opvalid) begin
                        case (opcode)
                            `OPCODE_NOP: begin
                                next_state = `FETCH; // Transition to IDLE state
                            end
                            `OPCODE_PARAM: begin
                                next_state = `PARAM_SET; // Transition to PARAM_SET state
                            end
                            `OPCODE_LDSRAM: begin
                                if (Param_TRG == `TRG_PSRAM) begin
                                    next_state = `SET_CTRL; // Transition to SET_CTRL state
                                    next_task_state = `WRITE_SRAM; // Transition to WRITE_SRAM state
                                end else begin
                                    next_state = `WRITE_SRAM; // Transition to WRITE_SRAM state
                                end
                            end
                            `OPCODE_STSRAM: begin
                                next_state = `READ_SRAM; // Transition to READ_SRAM state
                            end
                            `OPCODE_EX: begin
                                next_state = `SET_CTRL; // Transition to INIT_PSUM state
                                next_task_state = `INIT_PSUM; // Transition to INIT_PSUM state
                            end
                            `OPCODE_WBPSRAM: begin
                                next_state = `WRITE_BACK; // Transition to WRITE_BACK state
                            end
                            `OPCODE_WBPARAM: begin
                                next_state = `WRITE_BACK_PARAM; // Transition to WRITE_BACK_PARAM state
                            end
                            default: begin
                                next_state = `IDLE; // Default to IDLE state for invalid opcode
                            end
                        endcase
                    end else begin
                        next_state = `FETCH; // Transition to FETCH state if not valid
                    end
                end
                `PARAM_SET           : begin
                    next_state = `FETCH; // Transition to FETCH state after setting parameters
                end
                `WRITE_SRAM          : begin
                    previous_state = `WRITE_SRAM;
                    if (Param_TRG == `TRG_PSRAM) begin
                        next_state = `CLEAR_CTRL; // Transition to CLEAR_CTRL state after writing to Psum SRAM
                    end else begin
                        next_state = `FETCH; // Transition to FETCH state after writing to SRAM
                    end
                end
                `READ_SRAM           : begin
                    next_state = `FETCH; // Transition to FETCH state after reading from SRAM
                end
                // Execute
                `INIT_PSUM           : begin
                    previous_state = `INIT_PSUM;
                    if (psum_init_addr == 8) begin
                        next_state = `CLEAR_CTRL; // Transition to SET state after initializing Psum
                    end else begin
                        next_state = `INIT_PSUM; // Stay in INIT_PSUM state until done
                    end
                end
                `SET                 : begin
                    next_state = `PRELOAD; // Transition to PRELOAD state after setting up for execution
                end
                `PRELOAD             : begin
                    next_state = `LOAD; // Transition to LOAD state after preloading data
                end
                `LOAD                : begin
                    if (load_cnt == `PE_ROW -1) begin
                        next_state = `RUN; // Transition to RUN state after loading data
                    end else begin
                        next_state = `PRELOAD; // Stay in PRELOAD state until done
                    end
                end
                `RUN                 : begin
                    if (run_cnt < Param_S + `PE_ROW + `PE_COL) begin
                        next_state = `RUN; // Stay in RUN state until done
                    end else begin
                        if (Next_IC == 0 && Next_OC == 0) begin
                            next_state = `FETCH; // Transition to FETCH state after running
                        end else begin
                            next_state = `SET; // Transition to WRITE_BACK state after running
                        end
                    end
                end
                // Write Back
                `WRITE_BACK          : begin
                    next_state = `WRITE_BACK_READY; // Transition to FETCH state after write back
                end
                `WRITE_BACK_READY    : begin
                    next_state = `WRITE_BACK_OUTPUT;
                end
                `WRITE_BACK_OUTPUT   : begin
                    next_state = `FETCH; // Transition to FETCH state after write back output
                end
                `WRITE_BACK_PARAM    : begin
                    next_state = `FETCH; // Transition to FETCH state after write back parameter
                end
                `SET_CTRL            : begin
                    next_state = next_task_state; // Transition to next task state
                end
                `CLEAR_CTRL          : begin
                    if (previous_state == `INIT_PSUM) begin
                        next_state = `SET; // Transition to SET state after clearing control
                    end else begin
                        next_state = `FETCH; // Transition to FETCH state after clearing control
                    end
                end
                default: begin
                    next_state = `IDLE; // Default state
                end
            endcase
        end
    end

    // State sequence logic
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= `IDLE; // Reset to IDLE state
        end else begin
            state <= next_state; // Update state to next state
        end
    end

    // Stall logic
    // always @(posedge CLK or posedge RST) begin
    //     if (RST) begin
    //         instr_stall <= 1'b1;
    //     end else begin
    //         instr_stall <= (state != `FETCH);
    //     end
    // end

    // always @(*) begin
    //     // Calculate baseaddr of ISRAM/WSRAM
    //     Base_I = Param_S * (State_IC / `PE_ROW);
    //     Base_W = State_IC + Param_IC * (State_OC / `PE_COL);
    // end

    always @(posedge CLK) begin
        if (RST) begin
            clk_cnt <= 0; // Reset clock count
        end else begin
            clk_cnt <= clk_cnt + 1; // Increment clock count
        end
    end

    
    // Pattern array
    always @(*) begin
        case (run_cnt)
            0: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
            1: begin
                pattern_array[0] = 1;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
            2: begin
                pattern_array[0] = 2;
                pattern_array[1] = 1;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
            3: begin
                pattern_array[0] = 3;
                pattern_array[1] = 2;
                pattern_array[2] = 1;
                pattern_array[3] = 0;
            end
            4: begin
                pattern_array[0] = 0;
                pattern_array[1] = 3;
                pattern_array[2] = 2;
                pattern_array[3] = 1;
            end
            5: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 3;
                pattern_array[3] = 2;
            end
            6: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 3;
            end
            7: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
            8: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
            default: begin
                pattern_array[0] = 0;
                pattern_array[1] = 0;
                pattern_array[2] = 0;
                pattern_array[3] = 0;
            end
        endcase
    end

    always @(*) begin
        case (run_cnt)
            0: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end                
            1: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end
            2: begin
                pattern_array_d[0] = 1;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end
            3: begin
                pattern_array_d[0] = 2;
                pattern_array_d[1] = 1;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end
            4: begin
                pattern_array_d[0] = 3;
                pattern_array_d[1] = 2;
                pattern_array_d[2] = 1;
                pattern_array_d[3] = 0;
            end
            5: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 3;
                pattern_array_d[2] = 2;
                pattern_array_d[3] = 1;
            end
            6: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 3;
                pattern_array_d[3] = 2;
            end
            7: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 3;
            end
            8: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end
            default: begin
                pattern_array_d[0] = 0;
                pattern_array_d[1] = 0;
                pattern_array_d[2] = 0;
                pattern_array_d[3] = 0;
            end
        endcase
    end

    // Debug
    // always @(posedge CLK) begin
    // if (`DEBUG_MODE) begin
    //     $display("Clock = %0d, State=%0d, State_IC=%0d, State_OC=%0d, run_cnt=%0d, run_cnt_d=%0d, load_cnt=%0d", clk_cnt, state, State_IC, State_OC, run_cnt, run_cnt_d, load_cnt);
    //     // $display("  opvalid = %0d, opcode = %0d, sel = %0d, addr = %0d, data = %0d", opvalid, opcode, sel, addr, data);    
    //     $display("  Base_W = %0d, Base_I = %0d", Base_W, Base_I);
    //     $display("  Run_IC = %0d, Run_OC = %0d, Next_IC = %0d, Next_OC = %0d", Run_IC, Run_OC, Next_IC, Next_OC);
    // end
    // end

endmodule