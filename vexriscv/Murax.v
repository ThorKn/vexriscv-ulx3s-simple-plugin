// Generator : SpinalHDL v1.4.0    git head : ecb5a80b713566f417ea3ea061f9969e73770a7f
// Date      : 02/10/2020, 08:44:34
// Component : Murax


`define ShiftCtrlEnum_defaultEncoding_type [1:0]
`define ShiftCtrlEnum_defaultEncoding_DISABLE_1 2'b00
`define ShiftCtrlEnum_defaultEncoding_SLL_1 2'b01
`define ShiftCtrlEnum_defaultEncoding_SRL_1 2'b10
`define ShiftCtrlEnum_defaultEncoding_SRA_1 2'b11

`define AluCtrlEnum_defaultEncoding_type [1:0]
`define AluCtrlEnum_defaultEncoding_ADD_SUB 2'b00
`define AluCtrlEnum_defaultEncoding_SLT_SLTU 2'b01
`define AluCtrlEnum_defaultEncoding_BITWISE 2'b10

`define BranchCtrlEnum_defaultEncoding_type [1:0]
`define BranchCtrlEnum_defaultEncoding_INC 2'b00
`define BranchCtrlEnum_defaultEncoding_B 2'b01
`define BranchCtrlEnum_defaultEncoding_JAL 2'b10
`define BranchCtrlEnum_defaultEncoding_JALR 2'b11

`define AluBitwiseCtrlEnum_defaultEncoding_type [1:0]
`define AluBitwiseCtrlEnum_defaultEncoding_XOR_1 2'b00
`define AluBitwiseCtrlEnum_defaultEncoding_OR_1 2'b01
`define AluBitwiseCtrlEnum_defaultEncoding_AND_1 2'b10

`define EnvCtrlEnum_defaultEncoding_type [0:0]
`define EnvCtrlEnum_defaultEncoding_NONE 1'b0
`define EnvCtrlEnum_defaultEncoding_XRET 1'b1

`define Src2CtrlEnum_defaultEncoding_type [1:0]
`define Src2CtrlEnum_defaultEncoding_RS 2'b00
`define Src2CtrlEnum_defaultEncoding_IMI 2'b01
`define Src2CtrlEnum_defaultEncoding_IMS 2'b10
`define Src2CtrlEnum_defaultEncoding_PC 2'b11

`define Src1CtrlEnum_defaultEncoding_type [1:0]
`define Src1CtrlEnum_defaultEncoding_RS 2'b00
`define Src1CtrlEnum_defaultEncoding_IMU 2'b01
`define Src1CtrlEnum_defaultEncoding_PC_INCREMENT 2'b10
`define Src1CtrlEnum_defaultEncoding_URS1 2'b11

`define JtagState_defaultEncoding_type [3:0]
`define JtagState_defaultEncoding_RESET 4'b0000
`define JtagState_defaultEncoding_IDLE 4'b0001
`define JtagState_defaultEncoding_IR_SELECT 4'b0010
`define JtagState_defaultEncoding_IR_CAPTURE 4'b0011
`define JtagState_defaultEncoding_IR_SHIFT 4'b0100
`define JtagState_defaultEncoding_IR_EXIT1 4'b0101
`define JtagState_defaultEncoding_IR_PAUSE 4'b0110
`define JtagState_defaultEncoding_IR_EXIT2 4'b0111
`define JtagState_defaultEncoding_IR_UPDATE 4'b1000
`define JtagState_defaultEncoding_DR_SELECT 4'b1001
`define JtagState_defaultEncoding_DR_CAPTURE 4'b1010
`define JtagState_defaultEncoding_DR_SHIFT 4'b1011
`define JtagState_defaultEncoding_DR_EXIT1 4'b1100
`define JtagState_defaultEncoding_DR_PAUSE 4'b1101
`define JtagState_defaultEncoding_DR_EXIT2 4'b1110
`define JtagState_defaultEncoding_DR_UPDATE 4'b1111

`define UartStopType_defaultEncoding_type [0:0]
`define UartStopType_defaultEncoding_ONE 1'b0
`define UartStopType_defaultEncoding_TWO 1'b1

`define UartParityType_defaultEncoding_type [1:0]
`define UartParityType_defaultEncoding_NONE 2'b00
`define UartParityType_defaultEncoding_EVEN 2'b01
`define UartParityType_defaultEncoding_ODD 2'b10

`define UartCtrlTxState_defaultEncoding_type [2:0]
`define UartCtrlTxState_defaultEncoding_IDLE 3'b000
`define UartCtrlTxState_defaultEncoding_START 3'b001
`define UartCtrlTxState_defaultEncoding_DATA 3'b010
`define UartCtrlTxState_defaultEncoding_PARITY 3'b011
`define UartCtrlTxState_defaultEncoding_STOP 3'b100

`define UartCtrlRxState_defaultEncoding_type [2:0]
`define UartCtrlRxState_defaultEncoding_IDLE 3'b000
`define UartCtrlRxState_defaultEncoding_START 3'b001
`define UartCtrlRxState_defaultEncoding_DATA 3'b010
`define UartCtrlRxState_defaultEncoding_PARITY 3'b011
`define UartCtrlRxState_defaultEncoding_STOP 3'b100


module BufferCC (
  input               io_initial,
  input               io_dataIn,
  output              io_dataOut,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module BufferCC_1_ (
  input               io_dataIn,
  output              io_dataOut,
  input               io_mainClk,
  input               resetCtrl_mainClkReset 
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge io_mainClk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module UartCtrlTx (
  input      [2:0]    io_configFrame_dataLength,
  input      `UartStopType_defaultEncoding_type io_configFrame_stop,
  input      `UartParityType_defaultEncoding_type io_configFrame_parity,
  input               io_samplingTick,
  input               io_write_valid,
  output reg          io_write_ready,
  input      [7:0]    io_write_payload,
  input               io_cts,
  output              io_txd,
  input               io_break,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_2_;
  wire       [0:0]    _zz_3_;
  wire       [2:0]    _zz_4_;
  wire       [0:0]    _zz_5_;
  wire       [2:0]    _zz_6_;
  reg                 clockDivider_counter_willIncrement;
  wire                clockDivider_counter_willClear;
  reg        [2:0]    clockDivider_counter_valueNext;
  reg        [2:0]    clockDivider_counter_value;
  wire                clockDivider_counter_willOverflowIfInc;
  wire                clockDivider_counter_willOverflow;
  reg        [2:0]    tickCounter_value;
  reg        `UartCtrlTxState_defaultEncoding_type stateMachine_state;
  reg                 stateMachine_parity;
  reg                 stateMachine_txd;
  reg                 _zz_1_;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif


  assign _zz_2_ = (tickCounter_value == io_configFrame_dataLength);
  assign _zz_3_ = clockDivider_counter_willIncrement;
  assign _zz_4_ = {2'd0, _zz_3_};
  assign _zz_5_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_6_ = {2'd0, _zz_5_};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlTxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlTxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlTxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlTxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick)begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == (3'b100));
  assign clockDivider_counter_willOverflow = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @ (*) begin
    if(clockDivider_counter_willOverflow)begin
      clockDivider_counter_valueNext = (3'b000);
    end else begin
      clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_4_);
    end
    if(clockDivider_counter_willClear)begin
      clockDivider_counter_valueNext = (3'b000);
    end
  end

  always @ (*) begin
    stateMachine_txd = 1'b1;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        stateMachine_txd = 1'b0;
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_write_ready = io_break;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_counter_willOverflow)begin
          if(_zz_2_)begin
            io_write_ready = 1'b1;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
      end
      default : begin
      end
    endcase
  end

  assign io_txd = _zz_1_;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      clockDivider_counter_value <= (3'b000);
      stateMachine_state <= `UartCtrlTxState_defaultEncoding_IDLE;
      _zz_1_ <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        `UartCtrlTxState_defaultEncoding_IDLE : begin
          if(((io_write_valid && (! io_cts)) && clockDivider_counter_willOverflow))begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_START;
          end
        end
        `UartCtrlTxState_defaultEncoding_START : begin
          if(clockDivider_counter_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_DATA;
          end
        end
        `UartCtrlTxState_defaultEncoding_DATA : begin
          if(clockDivider_counter_willOverflow)begin
            if(_zz_2_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
              end else begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlTxState_defaultEncoding_PARITY : begin
          if(clockDivider_counter_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
          end
        end
        default : begin
          if(clockDivider_counter_willOverflow)begin
            if((tickCounter_value == _zz_6_))begin
              stateMachine_state <= (io_write_valid ? `UartCtrlTxState_defaultEncoding_START : `UartCtrlTxState_defaultEncoding_IDLE);
            end
          end
        end
      endcase
      _zz_1_ <= (stateMachine_txd && (! io_break));
    end
  end

  always @ (posedge io_mainClk) begin
    if(clockDivider_counter_willOverflow)begin
      tickCounter_value <= (tickCounter_value + (3'b001));
    end
    if(clockDivider_counter_willOverflow)begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        if(clockDivider_counter_willOverflow)begin
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
          tickCounter_value <= (3'b000);
        end
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_counter_willOverflow)begin
          if(_zz_2_)begin
            tickCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        if(clockDivider_counter_willOverflow)begin
          tickCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end


endmodule

module UartCtrlRx (
  input      [2:0]    io_configFrame_dataLength,
  input      `UartStopType_defaultEncoding_type io_configFrame_stop,
  input      `UartParityType_defaultEncoding_type io_configFrame_parity,
  input               io_samplingTick,
  output              io_read_valid,
  input               io_read_ready,
  output     [7:0]    io_read_payload,
  input               io_rxd,
  output              io_rts,
  output reg          io_error,
  output              io_break,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_2_;
  wire                io_rxd_buffercc_io_dataOut;
  wire                _zz_3_;
  wire                _zz_4_;
  wire                _zz_5_;
  wire                _zz_6_;
  wire                _zz_7_;
  wire       [0:0]    _zz_8_;
  wire       [2:0]    _zz_9_;
  reg                 _zz_1_;
  wire                sampler_synchroniser;
  wire                sampler_samples_0;
  reg                 sampler_samples_1;
  reg                 sampler_samples_2;
  reg                 sampler_value;
  reg                 sampler_tick;
  reg        [2:0]    bitTimer_counter;
  reg                 bitTimer_tick;
  reg        [2:0]    bitCounter_value;
  reg        [6:0]    break_counter;
  wire                break_valid;
  reg        `UartCtrlRxState_defaultEncoding_type stateMachine_state;
  reg                 stateMachine_parity;
  reg        [7:0]    stateMachine_shifter;
  reg                 stateMachine_validReg;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif


  assign _zz_3_ = (stateMachine_parity == sampler_value);
  assign _zz_4_ = (! sampler_value);
  assign _zz_5_ = (bitTimer_counter == (3'b000));
  assign _zz_6_ = ((sampler_tick && (! sampler_value)) && (! break_valid));
  assign _zz_7_ = (bitCounter_value == io_configFrame_dataLength);
  assign _zz_8_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_9_ = {2'd0, _zz_8_};
  BufferCC io_rxd_buffercc ( 
    .io_initial               (_zz_2_                      ), //i
    .io_dataIn                (io_rxd                      ), //i
    .io_dataOut               (io_rxd_buffercc_io_dataOut  ), //o
    .io_mainClk               (io_mainClk                  ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset       )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlRxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlRxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlRxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlRxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    io_error = 1'b0;
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlRxState_defaultEncoding_START : begin
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          if(! _zz_3_) begin
            io_error = 1'b1;
          end
        end
      end
      default : begin
        if(bitTimer_tick)begin
          if(_zz_4_)begin
            io_error = 1'b1;
          end
        end
      end
    endcase
  end

  assign io_rts = _zz_1_;
  assign _zz_2_ = 1'b0;
  assign sampler_synchroniser = io_rxd_buffercc_io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @ (*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick)begin
      if(_zz_5_)begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign break_valid = (break_counter == 7'h41);
  assign io_break = break_valid;
  assign io_read_valid = stateMachine_validReg;
  assign io_read_payload = stateMachine_shifter;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      break_counter <= 7'h0;
      stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      _zz_1_ <= (! io_read_ready);
      if(io_samplingTick)begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick)begin
        sampler_samples_2 <= sampler_samples_1;
      end
      sampler_value <= (((1'b0 || ((1'b1 && sampler_samples_0) && sampler_samples_1)) || ((1'b1 && sampler_samples_0) && sampler_samples_2)) || ((1'b1 && sampler_samples_1) && sampler_samples_2));
      sampler_tick <= io_samplingTick;
      if(sampler_value)begin
        break_counter <= 7'h0;
      end else begin
        if((io_samplingTick && (! break_valid)))begin
          break_counter <= (break_counter + 7'h01);
        end
      end
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        `UartCtrlRxState_defaultEncoding_IDLE : begin
          if(_zz_6_)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_START;
          end
        end
        `UartCtrlRxState_defaultEncoding_START : begin
          if(bitTimer_tick)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_DATA;
            if((sampler_value == 1'b1))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_DATA : begin
          if(bitTimer_tick)begin
            if(_zz_7_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_PARITY : begin
          if(bitTimer_tick)begin
            if(_zz_3_)begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick)begin
            if(_zz_4_)begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end else begin
              if((bitCounter_value == _zz_9_))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @ (posedge io_mainClk) begin
    if(sampler_tick)begin
      bitTimer_counter <= (bitTimer_counter - (3'b001));
      if(_zz_5_)begin
        bitTimer_counter <= (3'b100);
      end
    end
    if(bitTimer_tick)begin
      bitCounter_value <= (bitCounter_value + (3'b001));
    end
    if(bitTimer_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
        if(_zz_6_)begin
          bitTimer_counter <= (3'b001);
        end
      end
      `UartCtrlRxState_defaultEncoding_START : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
        end
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
        if(bitTimer_tick)begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(_zz_7_)begin
            bitCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end


endmodule

module StreamFifoLowLatency (
  input               io_push_valid,
  output              io_push_ready,
  input               io_push_payload_error,
  input      [31:0]   io_push_payload_inst,
  output reg          io_pop_valid,
  input               io_pop_ready,
  output reg          io_pop_payload_error,
  output reg [31:0]   io_pop_payload_inst,
  input               io_flush,
  output     [0:0]    io_occupancy,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_4_;
  wire       [0:0]    _zz_5_;
  reg                 _zz_1_;
  reg                 pushPtr_willIncrement;
  reg                 pushPtr_willClear;
  wire                pushPtr_willOverflowIfInc;
  wire                pushPtr_willOverflow;
  reg                 popPtr_willIncrement;
  reg                 popPtr_willClear;
  wire                popPtr_willOverflowIfInc;
  wire                popPtr_willOverflow;
  wire                ptrMatch;
  reg                 risingOccupancy;
  wire                empty;
  wire                full;
  wire                pushing;
  wire                popping;
  wire       [32:0]   _zz_2_;
  reg        [32:0]   _zz_3_;

  assign _zz_4_ = (! empty);
  assign _zz_5_ = _zz_2_[0 : 0];
  always @ (*) begin
    _zz_1_ = 1'b0;
    if(pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willIncrement = 1'b0;
    if(pushing)begin
      pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    pushPtr_willClear = 1'b0;
    if(io_flush)begin
      pushPtr_willClear = 1'b1;
    end
  end

  assign pushPtr_willOverflowIfInc = 1'b1;
  assign pushPtr_willOverflow = (pushPtr_willOverflowIfInc && pushPtr_willIncrement);
  always @ (*) begin
    popPtr_willIncrement = 1'b0;
    if(popping)begin
      popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    popPtr_willClear = 1'b0;
    if(io_flush)begin
      popPtr_willClear = 1'b1;
    end
  end

  assign popPtr_willOverflowIfInc = 1'b1;
  assign popPtr_willOverflow = (popPtr_willOverflowIfInc && popPtr_willIncrement);
  assign ptrMatch = 1'b1;
  assign empty = (ptrMatch && (! risingOccupancy));
  assign full = (ptrMatch && risingOccupancy);
  assign pushing = (io_push_valid && io_push_ready);
  assign popping = (io_pop_valid && io_pop_ready);
  assign io_push_ready = (! full);
  always @ (*) begin
    if(_zz_4_)begin
      io_pop_valid = 1'b1;
    end else begin
      io_pop_valid = io_push_valid;
    end
  end

  assign _zz_2_ = _zz_3_;
  always @ (*) begin
    if(_zz_4_)begin
      io_pop_payload_error = _zz_5_[0];
    end else begin
      io_pop_payload_error = io_push_payload_error;
    end
  end

  always @ (*) begin
    if(_zz_4_)begin
      io_pop_payload_inst = _zz_2_[32 : 1];
    end else begin
      io_pop_payload_inst = io_push_payload_inst;
    end
  end

  assign io_occupancy = (risingOccupancy && ptrMatch);
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      risingOccupancy <= 1'b0;
    end else begin
      if((pushing != popping))begin
        risingOccupancy <= pushing;
      end
      if(io_flush)begin
        risingOccupancy <= 1'b0;
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_1_)begin
      _zz_3_ <= {io_push_payload_inst,io_push_payload_error};
    end
  end


endmodule

module FlowCCByToggle (
  input               io_input_valid,
  input               io_input_payload_last,
  input      [0:0]    io_input_payload_fragment,
  output              io_output_valid,
  output              io_output_payload_last,
  output     [0:0]    io_output_payload_fragment,
  input               io_jtag_tck,
  input               io_mainClk,
  input               resetCtrl_mainClkReset 
);
  wire                inputArea_target_buffercc_io_dataOut;
  wire                outHitSignal;
  reg                 inputArea_target = 0;
  reg                 inputArea_data_last;
  reg        [0:0]    inputArea_data_fragment;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire                outputArea_flow_payload_last;
  wire       [0:0]    outputArea_flow_payload_fragment;
  reg                 outputArea_flow_regNext_valid;
  reg                 outputArea_flow_regNext_payload_last;
  reg        [0:0]    outputArea_flow_regNext_payload_fragment;

  BufferCC_1_ inputArea_target_buffercc ( 
    .io_dataIn                 (inputArea_target                      ), //i
    .io_dataOut                (inputArea_target_buffercc_io_dataOut  ), //o
    .io_mainClk                (io_mainClk                            ), //i
    .resetCtrl_mainClkReset    (resetCtrl_mainClkReset                )  //i
  );
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_last = inputArea_data_last;
  assign outputArea_flow_payload_fragment = inputArea_data_fragment;
  assign io_output_valid = outputArea_flow_regNext_valid;
  assign io_output_payload_last = outputArea_flow_regNext_payload_last;
  assign io_output_payload_fragment = outputArea_flow_regNext_payload_fragment;
  always @ (posedge io_jtag_tck) begin
    if(io_input_valid)begin
      inputArea_target <= (! inputArea_target);
      inputArea_data_last <= io_input_payload_last;
      inputArea_data_fragment <= io_input_payload_fragment;
    end
  end

  always @ (posedge io_mainClk) begin
    outputArea_hit <= outputArea_target;
    outputArea_flow_regNext_payload_last <= outputArea_flow_payload_last;
    outputArea_flow_regNext_payload_fragment <= outputArea_flow_payload_fragment;
  end

  always @ (posedge io_mainClk or posedge resetCtrl_mainClkReset) begin
    if (resetCtrl_mainClkReset) begin
      outputArea_flow_regNext_valid <= 1'b0;
    end else begin
      outputArea_flow_regNext_valid <= outputArea_flow_valid;
    end
  end


endmodule

module BufferCC_2_ (
  input      [31:0]   io_dataIn,
  output     [31:0]   io_dataOut,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg        [31:0]   buffers_0;
  reg        [31:0]   buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge io_mainClk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module UartCtrl (
  input      [2:0]    io_config_frame_dataLength,
  input      `UartStopType_defaultEncoding_type io_config_frame_stop,
  input      `UartParityType_defaultEncoding_type io_config_frame_parity,
  input      [19:0]   io_config_clockDivider,
  input               io_write_valid,
  output reg          io_write_ready,
  input      [7:0]    io_write_payload,
  output              io_read_valid,
  input               io_read_ready,
  output     [7:0]    io_read_payload,
  output              io_uart_txd,
  input               io_uart_rxd,
  output              io_readError,
  input               io_writeBreak,
  output              io_readBreak,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_1_;
  wire                tx_io_write_ready;
  wire                tx_io_txd;
  wire                rx_io_read_valid;
  wire       [7:0]    rx_io_read_payload;
  wire                rx_io_rts;
  wire                rx_io_error;
  wire                rx_io_break;
  reg        [19:0]   clockDivider_counter;
  wire                clockDivider_tick;
  reg                 io_write_thrown_valid;
  wire                io_write_thrown_ready;
  wire       [7:0]    io_write_thrown_payload;
  `ifndef SYNTHESIS
  reg [23:0] io_config_frame_stop_string;
  reg [31:0] io_config_frame_parity_string;
  `endif


  UartCtrlTx tx ( 
    .io_configFrame_dataLength    (io_config_frame_dataLength[2:0]  ), //i
    .io_configFrame_stop          (io_config_frame_stop             ), //i
    .io_configFrame_parity        (io_config_frame_parity[1:0]      ), //i
    .io_samplingTick              (clockDivider_tick                ), //i
    .io_write_valid               (io_write_thrown_valid            ), //i
    .io_write_ready               (tx_io_write_ready                ), //o
    .io_write_payload             (io_write_thrown_payload[7:0]     ), //i
    .io_cts                       (_zz_1_                           ), //i
    .io_txd                       (tx_io_txd                        ), //o
    .io_break                     (io_writeBreak                    ), //i
    .io_mainClk                   (io_mainClk                       ), //i
    .resetCtrl_systemReset        (resetCtrl_systemReset            )  //i
  );
  UartCtrlRx rx ( 
    .io_configFrame_dataLength    (io_config_frame_dataLength[2:0]  ), //i
    .io_configFrame_stop          (io_config_frame_stop             ), //i
    .io_configFrame_parity        (io_config_frame_parity[1:0]      ), //i
    .io_samplingTick              (clockDivider_tick                ), //i
    .io_read_valid                (rx_io_read_valid                 ), //o
    .io_read_ready                (io_read_ready                    ), //i
    .io_read_payload              (rx_io_read_payload[7:0]          ), //o
    .io_rxd                       (io_uart_rxd                      ), //i
    .io_rts                       (rx_io_rts                        ), //o
    .io_error                     (rx_io_error                      ), //o
    .io_break                     (rx_io_break                      ), //o
    .io_mainClk                   (io_mainClk                       ), //i
    .resetCtrl_systemReset        (resetCtrl_systemReset            )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_config_frame_stop)
      `UartStopType_defaultEncoding_ONE : io_config_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_config_frame_stop_string = "TWO";
      default : io_config_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_config_frame_parity)
      `UartParityType_defaultEncoding_NONE : io_config_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_config_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_config_frame_parity_string = "ODD ";
      default : io_config_frame_parity_string = "????";
    endcase
  end
  `endif

  assign clockDivider_tick = (clockDivider_counter == 20'h0);
  always @ (*) begin
    io_write_thrown_valid = io_write_valid;
    if(rx_io_break)begin
      io_write_thrown_valid = 1'b0;
    end
  end

  always @ (*) begin
    io_write_ready = io_write_thrown_ready;
    if(rx_io_break)begin
      io_write_ready = 1'b1;
    end
  end

  assign io_write_thrown_payload = io_write_payload;
  assign io_write_thrown_ready = tx_io_write_ready;
  assign io_read_valid = rx_io_read_valid;
  assign io_read_payload = rx_io_read_payload;
  assign io_uart_txd = tx_io_txd;
  assign io_readError = rx_io_error;
  assign _zz_1_ = 1'b0;
  assign io_readBreak = rx_io_break;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      clockDivider_counter <= 20'h0;
    end else begin
      clockDivider_counter <= (clockDivider_counter - 20'h00001);
      if(clockDivider_tick)begin
        clockDivider_counter <= io_config_clockDivider;
      end
    end
  end


endmodule

module StreamFifo (
  input               io_push_valid,
  output              io_push_ready,
  input      [7:0]    io_push_payload,
  output              io_pop_valid,
  input               io_pop_ready,
  output     [7:0]    io_pop_payload,
  input               io_flush,
  output     [4:0]    io_occupancy,
  output     [4:0]    io_availability,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg        [7:0]    _zz_3_;
  wire       [0:0]    _zz_4_;
  wire       [3:0]    _zz_5_;
  wire       [0:0]    _zz_6_;
  wire       [3:0]    _zz_7_;
  wire       [3:0]    _zz_8_;
  wire                _zz_9_;
  reg                 _zz_1_;
  reg                 logic_pushPtr_willIncrement;
  reg                 logic_pushPtr_willClear;
  reg        [3:0]    logic_pushPtr_valueNext;
  reg        [3:0]    logic_pushPtr_value;
  wire                logic_pushPtr_willOverflowIfInc;
  wire                logic_pushPtr_willOverflow;
  reg                 logic_popPtr_willIncrement;
  reg                 logic_popPtr_willClear;
  reg        [3:0]    logic_popPtr_valueNext;
  reg        [3:0]    logic_popPtr_value;
  wire                logic_popPtr_willOverflowIfInc;
  wire                logic_popPtr_willOverflow;
  wire                logic_ptrMatch;
  reg                 logic_risingOccupancy;
  wire                logic_pushing;
  wire                logic_popping;
  wire                logic_empty;
  wire                logic_full;
  reg                 _zz_2_;
  wire       [3:0]    logic_ptrDif;
  reg [7:0] logic_ram [0:15];

  assign _zz_4_ = logic_pushPtr_willIncrement;
  assign _zz_5_ = {3'd0, _zz_4_};
  assign _zz_6_ = logic_popPtr_willIncrement;
  assign _zz_7_ = {3'd0, _zz_6_};
  assign _zz_8_ = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9_ = 1'b1;
  always @ (posedge io_mainClk) begin
    if(_zz_9_) begin
      _zz_3_ <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_1_) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    if(logic_pushing)begin
      _zz_1_ = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == (4'b1111));
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5_);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = (4'b0000);
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == (4'b1111));
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7_);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = (4'b0000);
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2_ && (! logic_full))));
  assign io_pop_payload = _zz_3_;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8_};
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      logic_pushPtr_value <= (4'b0000);
      logic_popPtr_value <= (4'b0000);
      logic_risingOccupancy <= 1'b0;
      _zz_2_ <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2_ <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end


endmodule
//StreamFifo_1_ replaced by StreamFifo

module Prescaler (
  input               io_clear,
  input      [15:0]   io_limit,
  output              io_overflow,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg        [15:0]   counter;

  assign io_overflow = (counter == io_limit);
  always @ (posedge io_mainClk) begin
    counter <= (counter + 16'h0001);
    if((io_clear || io_overflow))begin
      counter <= 16'h0;
    end
  end


endmodule

module Timer (
  input               io_tick,
  input               io_clear,
  input      [15:0]   io_limit,
  output              io_full,
  output     [15:0]   io_value,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire       [0:0]    _zz_1_;
  wire       [15:0]   _zz_2_;
  reg        [15:0]   counter;
  wire                limitHit;
  reg                 inhibitFull;

  assign _zz_1_ = (! limitHit);
  assign _zz_2_ = {15'd0, _zz_1_};
  assign limitHit = (counter == io_limit);
  assign io_full = ((limitHit && io_tick) && (! inhibitFull));
  assign io_value = counter;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      inhibitFull <= 1'b0;
    end else begin
      if(io_tick)begin
        inhibitFull <= limitHit;
      end
      if(io_clear)begin
        inhibitFull <= 1'b0;
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(io_tick)begin
      counter <= (counter + _zz_2_);
    end
    if(io_clear)begin
      counter <= 16'h0;
    end
  end


endmodule
//Timer_1_ replaced by Timer

module InterruptCtrl (
  input      [1:0]    io_inputs,
  input      [1:0]    io_clears,
  input      [1:0]    io_masks,
  output     [1:0]    io_pendings,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg        [1:0]    pendings;

  assign io_pendings = (pendings & io_masks);
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      pendings <= (2'b00);
    end else begin
      pendings <= ((pendings & (~ io_clears)) | io_inputs);
    end
  end


endmodule

module BufferCC_3_ (
  input               io_dataIn,
  output              io_dataOut,
  input               io_mainClk 
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge io_mainClk) begin
    buffers_0 <= io_dataIn;
    buffers_1 <= buffers_0;
  end


endmodule

module MuraxMasterArbiter (
  input               io_iBus_cmd_valid,
  output reg          io_iBus_cmd_ready,
  input      [31:0]   io_iBus_cmd_payload_pc,
  output              io_iBus_rsp_valid,
  output              io_iBus_rsp_payload_error,
  output     [31:0]   io_iBus_rsp_payload_inst,
  input               io_dBus_cmd_valid,
  output reg          io_dBus_cmd_ready,
  input               io_dBus_cmd_payload_wr,
  input      [31:0]   io_dBus_cmd_payload_address,
  input      [31:0]   io_dBus_cmd_payload_data,
  input      [1:0]    io_dBus_cmd_payload_size,
  output              io_dBus_rsp_ready,
  output              io_dBus_rsp_error,
  output     [31:0]   io_dBus_rsp_data,
  output reg          io_masterBus_cmd_valid,
  input               io_masterBus_cmd_ready,
  output              io_masterBus_cmd_payload_write,
  output     [31:0]   io_masterBus_cmd_payload_address,
  output     [31:0]   io_masterBus_cmd_payload_data,
  output     [3:0]    io_masterBus_cmd_payload_mask,
  input               io_masterBus_rsp_valid,
  input      [31:0]   io_masterBus_rsp_payload_data,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_2_;
  reg        [3:0]    _zz_1_;
  reg                 rspPending;
  reg                 rspTarget;

  assign _zz_2_ = (rspPending && (! io_masterBus_rsp_valid));
  always @ (*) begin
    io_masterBus_cmd_valid = (io_iBus_cmd_valid || io_dBus_cmd_valid);
    if(_zz_2_)begin
      io_masterBus_cmd_valid = 1'b0;
    end
  end

  assign io_masterBus_cmd_payload_write = (io_dBus_cmd_valid && io_dBus_cmd_payload_wr);
  assign io_masterBus_cmd_payload_address = (io_dBus_cmd_valid ? io_dBus_cmd_payload_address : io_iBus_cmd_payload_pc);
  assign io_masterBus_cmd_payload_data = io_dBus_cmd_payload_data;
  always @ (*) begin
    case(io_dBus_cmd_payload_size)
      2'b00 : begin
        _zz_1_ = (4'b0001);
      end
      2'b01 : begin
        _zz_1_ = (4'b0011);
      end
      default : begin
        _zz_1_ = (4'b1111);
      end
    endcase
  end

  assign io_masterBus_cmd_payload_mask = (_zz_1_ <<< io_dBus_cmd_payload_address[1 : 0]);
  always @ (*) begin
    io_iBus_cmd_ready = (io_masterBus_cmd_ready && (! io_dBus_cmd_valid));
    if(_zz_2_)begin
      io_iBus_cmd_ready = 1'b0;
    end
  end

  always @ (*) begin
    io_dBus_cmd_ready = io_masterBus_cmd_ready;
    if(_zz_2_)begin
      io_dBus_cmd_ready = 1'b0;
    end
  end

  assign io_iBus_rsp_valid = (io_masterBus_rsp_valid && (! rspTarget));
  assign io_iBus_rsp_payload_inst = io_masterBus_rsp_payload_data;
  assign io_iBus_rsp_payload_error = 1'b0;
  assign io_dBus_rsp_ready = (io_masterBus_rsp_valid && rspTarget);
  assign io_dBus_rsp_data = io_masterBus_rsp_payload_data;
  assign io_dBus_rsp_error = 1'b0;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      rspPending <= 1'b0;
      rspTarget <= 1'b0;
    end else begin
      if(io_masterBus_rsp_valid)begin
        rspPending <= 1'b0;
      end
      if(((io_masterBus_cmd_valid && io_masterBus_cmd_ready) && (! io_masterBus_cmd_payload_write)))begin
        rspTarget <= io_dBus_cmd_valid;
        rspPending <= 1'b1;
      end
    end
  end


endmodule

module VexRiscv (
  output              iBus_cmd_valid,
  input               iBus_cmd_ready,
  output     [31:0]   iBus_cmd_payload_pc,
  input               iBus_rsp_valid,
  input               iBus_rsp_payload_error,
  input      [31:0]   iBus_rsp_payload_inst,
  input               timerInterrupt,
  input               externalInterrupt,
  input               softwareInterrupt,
  input               debug_bus_cmd_valid,
  output reg          debug_bus_cmd_ready,
  input               debug_bus_cmd_payload_wr,
  input      [7:0]    debug_bus_cmd_payload_address,
  input      [31:0]   debug_bus_cmd_payload_data,
  output reg [31:0]   debug_bus_rsp_data,
  output              debug_resetOut,
  output              dBus_cmd_valid,
  input               dBus_cmd_ready,
  output              dBus_cmd_payload_wr,
  output     [31:0]   dBus_cmd_payload_address,
  output     [31:0]   dBus_cmd_payload_data,
  output     [1:0]    dBus_cmd_payload_size,
  input               dBus_rsp_ready,
  input               dBus_rsp_error,
  input      [31:0]   dBus_rsp_data,
  input               io_mainClk,
  input               resetCtrl_systemReset,
  input               resetCtrl_mainClkReset 
);
  wire                _zz_115_;
  wire                _zz_116_;
  reg        [31:0]   _zz_117_;
  reg        [31:0]   _zz_118_;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  wire       [0:0]    IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy;
  wire                _zz_119_;
  wire                _zz_120_;
  wire                _zz_121_;
  wire                _zz_122_;
  wire                _zz_123_;
  wire                _zz_124_;
  wire                _zz_125_;
  wire                _zz_126_;
  wire       [1:0]    _zz_127_;
  wire                _zz_128_;
  wire                _zz_129_;
  wire                _zz_130_;
  wire                _zz_131_;
  wire                _zz_132_;
  wire                _zz_133_;
  wire       [5:0]    _zz_134_;
  wire                _zz_135_;
  wire                _zz_136_;
  wire                _zz_137_;
  wire                _zz_138_;
  wire       [1:0]    _zz_139_;
  wire                _zz_140_;
  wire       [0:0]    _zz_141_;
  wire       [0:0]    _zz_142_;
  wire       [0:0]    _zz_143_;
  wire       [0:0]    _zz_144_;
  wire       [0:0]    _zz_145_;
  wire       [0:0]    _zz_146_;
  wire       [0:0]    _zz_147_;
  wire       [0:0]    _zz_148_;
  wire       [0:0]    _zz_149_;
  wire       [0:0]    _zz_150_;
  wire       [0:0]    _zz_151_;
  wire       [0:0]    _zz_152_;
  wire       [0:0]    _zz_153_;
  wire       [31:0]   _zz_154_;
  wire       [1:0]    _zz_155_;
  wire       [1:0]    _zz_156_;
  wire       [2:0]    _zz_157_;
  wire       [31:0]   _zz_158_;
  wire       [2:0]    _zz_159_;
  wire       [0:0]    _zz_160_;
  wire       [2:0]    _zz_161_;
  wire       [0:0]    _zz_162_;
  wire       [2:0]    _zz_163_;
  wire       [0:0]    _zz_164_;
  wire       [2:0]    _zz_165_;
  wire       [0:0]    _zz_166_;
  wire       [2:0]    _zz_167_;
  wire       [4:0]    _zz_168_;
  wire       [11:0]   _zz_169_;
  wire       [11:0]   _zz_170_;
  wire       [31:0]   _zz_171_;
  wire       [31:0]   _zz_172_;
  wire       [31:0]   _zz_173_;
  wire       [31:0]   _zz_174_;
  wire       [31:0]   _zz_175_;
  wire       [31:0]   _zz_176_;
  wire       [31:0]   _zz_177_;
  wire       [31:0]   _zz_178_;
  wire       [32:0]   _zz_179_;
  wire       [19:0]   _zz_180_;
  wire       [11:0]   _zz_181_;
  wire       [11:0]   _zz_182_;
  wire       [0:0]    _zz_183_;
  wire       [0:0]    _zz_184_;
  wire       [0:0]    _zz_185_;
  wire       [0:0]    _zz_186_;
  wire       [0:0]    _zz_187_;
  wire       [0:0]    _zz_188_;
  wire                _zz_189_;
  wire                _zz_190_;
  wire       [31:0]   _zz_191_;
  wire       [31:0]   _zz_192_;
  wire       [31:0]   _zz_193_;
  wire       [0:0]    _zz_194_;
  wire       [0:0]    _zz_195_;
  wire       [1:0]    _zz_196_;
  wire       [1:0]    _zz_197_;
  wire                _zz_198_;
  wire       [0:0]    _zz_199_;
  wire       [20:0]   _zz_200_;
  wire       [31:0]   _zz_201_;
  wire       [31:0]   _zz_202_;
  wire       [31:0]   _zz_203_;
  wire                _zz_204_;
  wire                _zz_205_;
  wire       [0:0]    _zz_206_;
  wire       [1:0]    _zz_207_;
  wire       [1:0]    _zz_208_;
  wire       [1:0]    _zz_209_;
  wire                _zz_210_;
  wire       [0:0]    _zz_211_;
  wire       [17:0]   _zz_212_;
  wire       [31:0]   _zz_213_;
  wire       [31:0]   _zz_214_;
  wire       [31:0]   _zz_215_;
  wire       [31:0]   _zz_216_;
  wire       [31:0]   _zz_217_;
  wire       [31:0]   _zz_218_;
  wire       [31:0]   _zz_219_;
  wire                _zz_220_;
  wire                _zz_221_;
  wire       [0:0]    _zz_222_;
  wire       [4:0]    _zz_223_;
  wire       [1:0]    _zz_224_;
  wire       [1:0]    _zz_225_;
  wire                _zz_226_;
  wire       [0:0]    _zz_227_;
  wire       [14:0]   _zz_228_;
  wire       [31:0]   _zz_229_;
  wire       [31:0]   _zz_230_;
  wire                _zz_231_;
  wire       [0:0]    _zz_232_;
  wire       [1:0]    _zz_233_;
  wire       [31:0]   _zz_234_;
  wire       [31:0]   _zz_235_;
  wire       [31:0]   _zz_236_;
  wire       [31:0]   _zz_237_;
  wire       [31:0]   _zz_238_;
  wire       [31:0]   _zz_239_;
  wire       [0:0]    _zz_240_;
  wire       [0:0]    _zz_241_;
  wire       [0:0]    _zz_242_;
  wire       [0:0]    _zz_243_;
  wire                _zz_244_;
  wire       [0:0]    _zz_245_;
  wire       [11:0]   _zz_246_;
  wire       [31:0]   _zz_247_;
  wire       [31:0]   _zz_248_;
  wire       [31:0]   _zz_249_;
  wire       [31:0]   _zz_250_;
  wire       [31:0]   _zz_251_;
  wire       [31:0]   _zz_252_;
  wire                _zz_253_;
  wire                _zz_254_;
  wire       [0:0]    _zz_255_;
  wire       [0:0]    _zz_256_;
  wire                _zz_257_;
  wire       [0:0]    _zz_258_;
  wire       [8:0]    _zz_259_;
  wire       [31:0]   _zz_260_;
  wire       [31:0]   _zz_261_;
  wire       [31:0]   _zz_262_;
  wire       [0:0]    _zz_263_;
  wire       [0:0]    _zz_264_;
  wire       [0:0]    _zz_265_;
  wire       [0:0]    _zz_266_;
  wire                _zz_267_;
  wire       [0:0]    _zz_268_;
  wire       [5:0]    _zz_269_;
  wire       [31:0]   _zz_270_;
  wire                _zz_271_;
  wire                _zz_272_;
  wire       [0:0]    _zz_273_;
  wire       [2:0]    _zz_274_;
  wire       [0:0]    _zz_275_;
  wire       [0:0]    _zz_276_;
  wire                _zz_277_;
  wire       [0:0]    _zz_278_;
  wire       [1:0]    _zz_279_;
  wire       [31:0]   _zz_280_;
  wire       [31:0]   _zz_281_;
  wire       [31:0]   _zz_282_;
  wire                _zz_283_;
  wire       [31:0]   _zz_284_;
  wire                _zz_285_;
  wire       [0:0]    _zz_286_;
  wire       [0:0]    _zz_287_;
  wire       [2:0]    _zz_288_;
  wire       [2:0]    _zz_289_;
  wire       [1:0]    _zz_290_;
  wire       [1:0]    _zz_291_;
  wire       [31:0]   _zz_292_;
  wire       [31:0]   _zz_293_;
  wire       [31:0]   _zz_294_;
  wire       [31:0]   _zz_295_;
  wire       [31:0]   _zz_296_;
  wire       [31:0]   _zz_297_;
  wire                decode_IS_SIMD_ADD;
  wire                decode_SRC_LESS_UNSIGNED;
  wire       `ShiftCtrlEnum_defaultEncoding_type decode_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_1_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_2_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_3_;
  wire                decode_BYPASSABLE_EXECUTE_STAGE;
  wire       `AluCtrlEnum_defaultEncoding_type decode_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_4_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_5_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_6_;
  wire                decode_CSR_READ_OPCODE;
  wire       `BranchCtrlEnum_defaultEncoding_type decode_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_7_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_8_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_9_;
  wire                decode_MEMORY_STORE;
  wire       [31:0]   decode_SRC2;
  wire                decode_MEMORY_ENABLE;
  wire       [31:0]   writeBack_FORMAL_PC_NEXT;
  wire       [31:0]   memory_FORMAL_PC_NEXT;
  wire       [31:0]   execute_FORMAL_PC_NEXT;
  wire       [31:0]   decode_FORMAL_PC_NEXT;
  wire                execute_BYPASSABLE_MEMORY_STAGE;
  wire                decode_BYPASSABLE_MEMORY_STAGE;
  wire       [1:0]    memory_MEMORY_ADDRESS_LOW;
  wire       [1:0]    execute_MEMORY_ADDRESS_LOW;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type decode_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_10_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_11_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_12_;
  wire                decode_CSR_WRITE_OPCODE;
  wire       [31:0]   writeBack_REGFILE_WRITE_DATA;
  wire       [31:0]   execute_REGFILE_WRITE_DATA;
  wire       [31:0]   memory_MEMORY_READ_DATA;
  wire                decode_SRC2_FORCE_ZERO;
  wire                execute_BRANCH_DO;
  wire       [31:0]   memory_PC;
  wire       [31:0]   decode_RS1;
  wire                decode_IS_CSR;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_13_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_14_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_15_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_16_;
  wire       `EnvCtrlEnum_defaultEncoding_type decode_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_17_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_18_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_19_;
  wire       [31:0]   execute_BRANCH_CALC;
  wire       [31:0]   decode_SRC1;
  wire                decode_DO_EBREAK;
  wire       [31:0]   decode_RS2;
  wire                execute_DO_EBREAK;
  wire                decode_IS_EBREAK;
  wire       [31:0]   memory_BRANCH_CALC;
  wire                memory_BRANCH_DO;
  wire       [31:0]   execute_PC;
  wire       `BranchCtrlEnum_defaultEncoding_type execute_BRANCH_CTRL;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_20_;
  wire                decode_RS2_USE;
  wire                decode_RS1_USE;
  wire                execute_REGFILE_WRITE_VALID;
  wire                execute_BYPASSABLE_EXECUTE_STAGE;
  wire                memory_REGFILE_WRITE_VALID;
  wire       [31:0]   memory_INSTRUCTION;
  wire                memory_BYPASSABLE_MEMORY_STAGE;
  wire                writeBack_REGFILE_WRITE_VALID;
  wire       [31:0]   memory_REGFILE_WRITE_DATA;
  wire       `ShiftCtrlEnum_defaultEncoding_type execute_SHIFT_CTRL;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_21_;
  wire                execute_SRC_LESS_UNSIGNED;
  wire                execute_SRC2_FORCE_ZERO;
  wire                execute_SRC_USE_SUB_LESS;
  wire       [31:0]   _zz_22_;
  wire       [31:0]   _zz_23_;
  wire       `Src2CtrlEnum_defaultEncoding_type decode_SRC2_CTRL;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_24_;
  wire       [31:0]   _zz_25_;
  wire       `Src1CtrlEnum_defaultEncoding_type decode_SRC1_CTRL;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_26_;
  wire                decode_SRC_USE_SUB_LESS;
  wire                decode_SRC_ADD_ZERO;
  wire       [31:0]   execute_SRC_ADD_SUB;
  wire                execute_SRC_LESS;
  wire       `AluCtrlEnum_defaultEncoding_type execute_ALU_CTRL;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_27_;
  wire       [31:0]   execute_SRC2;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type execute_ALU_BITWISE_CTRL;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_28_;
  wire       [31:0]   _zz_29_;
  wire                _zz_30_;
  reg                 _zz_31_;
  wire       [31:0]   decode_INSTRUCTION_ANTICIPATED;
  reg                 decode_REGFILE_WRITE_VALID;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_32_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_33_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_34_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_35_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_36_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_37_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_38_;
  wire       [31:0]   execute_SRC1;
  wire                execute_CSR_READ_OPCODE;
  wire                execute_CSR_WRITE_OPCODE;
  wire                execute_IS_CSR;
  wire       `EnvCtrlEnum_defaultEncoding_type memory_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_39_;
  wire       `EnvCtrlEnum_defaultEncoding_type execute_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_40_;
  wire       `EnvCtrlEnum_defaultEncoding_type writeBack_ENV_CTRL;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_41_;
  wire                writeBack_MEMORY_STORE;
  reg        [31:0]   _zz_42_;
  wire                writeBack_MEMORY_ENABLE;
  wire       [1:0]    writeBack_MEMORY_ADDRESS_LOW;
  wire       [31:0]   writeBack_MEMORY_READ_DATA;
  wire                memory_MEMORY_STORE;
  wire                memory_MEMORY_ENABLE;
  wire       [31:0]   execute_SRC_ADD;
  wire       [31:0]   execute_INSTRUCTION;
  wire                execute_MEMORY_STORE;
  wire                execute_MEMORY_ENABLE;
  wire                execute_ALIGNEMENT_FAULT;
  reg        [31:0]   _zz_43_;
  wire       [31:0]   decode_PC;
  wire       [31:0]   decode_INSTRUCTION;
  reg        [31:0]   _zz_44_;
  wire                execute_IS_SIMD_ADD;
  wire       [31:0]   execute_RS2;
  wire       [31:0]   execute_RS1;
  wire       [31:0]   writeBack_PC;
  wire       [31:0]   writeBack_INSTRUCTION;
  reg                 decode_arbitration_haltItself;
  reg                 decode_arbitration_haltByOther;
  reg                 decode_arbitration_removeIt;
  wire                decode_arbitration_flushIt;
  wire                decode_arbitration_flushNext;
  reg                 decode_arbitration_isValid;
  wire                decode_arbitration_isStuck;
  wire                decode_arbitration_isStuckByOthers;
  wire                decode_arbitration_isFlushed;
  wire                decode_arbitration_isMoving;
  wire                decode_arbitration_isFiring;
  reg                 execute_arbitration_haltItself;
  reg                 execute_arbitration_haltByOther;
  reg                 execute_arbitration_removeIt;
  reg                 execute_arbitration_flushIt;
  reg                 execute_arbitration_flushNext;
  reg                 execute_arbitration_isValid;
  wire                execute_arbitration_isStuck;
  wire                execute_arbitration_isStuckByOthers;
  wire                execute_arbitration_isFlushed;
  wire                execute_arbitration_isMoving;
  wire                execute_arbitration_isFiring;
  reg                 memory_arbitration_haltItself;
  wire                memory_arbitration_haltByOther;
  reg                 memory_arbitration_removeIt;
  wire                memory_arbitration_flushIt;
  reg                 memory_arbitration_flushNext;
  reg                 memory_arbitration_isValid;
  wire                memory_arbitration_isStuck;
  wire                memory_arbitration_isStuckByOthers;
  wire                memory_arbitration_isFlushed;
  wire                memory_arbitration_isMoving;
  wire                memory_arbitration_isFiring;
  wire                writeBack_arbitration_haltItself;
  wire                writeBack_arbitration_haltByOther;
  reg                 writeBack_arbitration_removeIt;
  wire                writeBack_arbitration_flushIt;
  reg                 writeBack_arbitration_flushNext;
  reg                 writeBack_arbitration_isValid;
  wire                writeBack_arbitration_isStuck;
  wire                writeBack_arbitration_isStuckByOthers;
  wire                writeBack_arbitration_isFlushed;
  wire                writeBack_arbitration_isMoving;
  wire                writeBack_arbitration_isFiring;
  wire       [31:0]   lastStageInstruction /* verilator public */ ;
  wire       [31:0]   lastStagePc /* verilator public */ ;
  wire                lastStageIsValid /* verilator public */ ;
  wire                lastStageIsFiring /* verilator public */ ;
  reg                 IBusSimplePlugin_fetcherHalt;
  reg                 IBusSimplePlugin_incomingInstruction;
  wire                IBusSimplePlugin_pcValids_0;
  wire                IBusSimplePlugin_pcValids_1;
  wire                IBusSimplePlugin_pcValids_2;
  wire                IBusSimplePlugin_pcValids_3;
  wire                CsrPlugin_inWfi /* verilator public */ ;
  reg                 CsrPlugin_thirdPartyWake;
  reg                 CsrPlugin_jumpInterface_valid;
  reg        [31:0]   CsrPlugin_jumpInterface_payload;
  wire                CsrPlugin_exceptionPendings_0;
  wire                CsrPlugin_exceptionPendings_1;
  wire                CsrPlugin_exceptionPendings_2;
  wire                CsrPlugin_exceptionPendings_3;
  wire                contextSwitching;
  reg        [1:0]    CsrPlugin_privilege;
  reg                 CsrPlugin_forceMachineWire;
  reg                 CsrPlugin_allowInterrupts;
  reg                 CsrPlugin_allowException;
  wire                BranchPlugin_jumpInterface_valid;
  wire       [31:0]   BranchPlugin_jumpInterface_payload;
  reg                 IBusSimplePlugin_injectionPort_valid;
  reg                 IBusSimplePlugin_injectionPort_ready;
  wire       [31:0]   IBusSimplePlugin_injectionPort_payload;
  wire       [31:0]   execute_SimdAddPlugin_rs1;
  wire       [31:0]   execute_SimdAddPlugin_rs2;
  wire       [31:0]   execute_SimdAddPlugin_rd;
  wire                IBusSimplePlugin_externalFlush;
  wire                IBusSimplePlugin_jump_pcLoad_valid;
  wire       [31:0]   IBusSimplePlugin_jump_pcLoad_payload;
  wire       [1:0]    _zz_45_;
  wire                IBusSimplePlugin_fetchPc_output_valid;
  wire                IBusSimplePlugin_fetchPc_output_ready;
  wire       [31:0]   IBusSimplePlugin_fetchPc_output_payload;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pcReg /* verilator public */ ;
  reg                 IBusSimplePlugin_fetchPc_correction;
  reg                 IBusSimplePlugin_fetchPc_correctionReg;
  wire                IBusSimplePlugin_fetchPc_corrected;
  reg                 IBusSimplePlugin_fetchPc_pcRegPropagate;
  reg                 IBusSimplePlugin_fetchPc_booted;
  reg                 IBusSimplePlugin_fetchPc_inc;
  reg        [31:0]   IBusSimplePlugin_fetchPc_pc;
  reg                 IBusSimplePlugin_fetchPc_flushed;
  wire                IBusSimplePlugin_iBusRsp_redoFetch;
  wire                IBusSimplePlugin_iBusRsp_stages_0_input_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_0_output_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_0_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_0_output_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_0_halt;
  wire                IBusSimplePlugin_iBusRsp_stages_1_input_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_1_input_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_1_output_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_1_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_1_output_payload;
  reg                 IBusSimplePlugin_iBusRsp_stages_1_halt;
  wire                IBusSimplePlugin_iBusRsp_stages_2_input_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_2_input_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_2_output_valid;
  wire                IBusSimplePlugin_iBusRsp_stages_2_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  wire                IBusSimplePlugin_iBusRsp_stages_2_halt;
  wire                _zz_46_;
  wire                _zz_47_;
  wire                _zz_48_;
  wire                IBusSimplePlugin_iBusRsp_flush;
  wire                _zz_49_;
  wire                _zz_50_;
  reg                 _zz_51_;
  wire                _zz_52_;
  reg                 _zz_53_;
  reg        [31:0]   _zz_54_;
  reg                 IBusSimplePlugin_iBusRsp_readyForError;
  wire                IBusSimplePlugin_iBusRsp_output_valid;
  wire                IBusSimplePlugin_iBusRsp_output_ready;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_pc;
  wire                IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
  wire                IBusSimplePlugin_iBusRsp_output_payload_isRvc;
  wire                IBusSimplePlugin_injector_decodeInput_valid;
  wire                IBusSimplePlugin_injector_decodeInput_ready;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_pc;
  wire                IBusSimplePlugin_injector_decodeInput_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  wire                IBusSimplePlugin_injector_decodeInput_payload_isRvc;
  reg                 _zz_55_;
  reg        [31:0]   _zz_56_;
  reg                 _zz_57_;
  reg        [31:0]   _zz_58_;
  reg                 _zz_59_;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_0;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_1;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_2;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_3;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_4;
  reg                 IBusSimplePlugin_injector_nextPcCalc_valids_5;
  reg        [31:0]   IBusSimplePlugin_injector_formal_rawInDecode;
  wire                IBusSimplePlugin_cmd_valid;
  wire                IBusSimplePlugin_cmd_ready;
  wire       [31:0]   IBusSimplePlugin_cmd_payload_pc;
  wire                IBusSimplePlugin_pending_inc;
  wire                IBusSimplePlugin_pending_dec;
  reg        [2:0]    IBusSimplePlugin_pending_value;
  wire       [2:0]    IBusSimplePlugin_pending_next;
  wire                IBusSimplePlugin_cmdFork_canEmit;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_valid;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_ready;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst;
  reg        [2:0]    IBusSimplePlugin_rspJoin_rspBuffer_discardCounter;
  wire                IBusSimplePlugin_rspJoin_rspBuffer_flush;
  wire       [31:0]   IBusSimplePlugin_rspJoin_fetchRsp_pc;
  reg                 IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  wire                IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  wire                IBusSimplePlugin_rspJoin_join_valid;
  wire                IBusSimplePlugin_rspJoin_join_ready;
  wire       [31:0]   IBusSimplePlugin_rspJoin_join_payload_pc;
  wire                IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  wire       [31:0]   IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  wire                IBusSimplePlugin_rspJoin_join_payload_isRvc;
  wire                IBusSimplePlugin_rspJoin_exceptionDetected;
  wire                _zz_60_;
  wire                _zz_61_;
  reg                 execute_DBusSimplePlugin_skipCmd;
  reg        [31:0]   _zz_62_;
  reg        [3:0]    _zz_63_;
  wire       [3:0]    execute_DBusSimplePlugin_formalMask;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspShifted;
  wire                _zz_64_;
  reg        [31:0]   _zz_65_;
  wire                _zz_66_;
  reg        [31:0]   _zz_67_;
  reg        [31:0]   writeBack_DBusSimplePlugin_rspFormated;
  wire       [1:0]    CsrPlugin_misa_base;
  wire       [25:0]   CsrPlugin_misa_extensions;
  wire       [1:0]    CsrPlugin_mtvec_mode;
  wire       [29:0]   CsrPlugin_mtvec_base;
  reg        [31:0]   CsrPlugin_mepc;
  reg                 CsrPlugin_mstatus_MIE;
  reg                 CsrPlugin_mstatus_MPIE;
  reg        [1:0]    CsrPlugin_mstatus_MPP;
  reg                 CsrPlugin_mip_MEIP;
  reg                 CsrPlugin_mip_MTIP;
  reg                 CsrPlugin_mip_MSIP;
  reg                 CsrPlugin_mie_MEIE;
  reg                 CsrPlugin_mie_MTIE;
  reg                 CsrPlugin_mie_MSIE;
  reg                 CsrPlugin_mcause_interrupt;
  reg        [3:0]    CsrPlugin_mcause_exceptionCode;
  reg        [31:0]   CsrPlugin_mtval;
  reg        [63:0]   CsrPlugin_mcycle = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  reg        [63:0]   CsrPlugin_minstret = 64'b0000000000000000000000000000000000000000000000000000000000000000;
  wire                _zz_68_;
  wire                _zz_69_;
  wire                _zz_70_;
  reg                 CsrPlugin_interrupt_valid;
  reg        [3:0]    CsrPlugin_interrupt_code /* verilator public */ ;
  reg        [1:0]    CsrPlugin_interrupt_targetPrivilege;
  wire                CsrPlugin_exception;
  wire                CsrPlugin_lastStageWasWfi;
  reg                 CsrPlugin_pipelineLiberator_pcValids_0;
  reg                 CsrPlugin_pipelineLiberator_pcValids_1;
  reg                 CsrPlugin_pipelineLiberator_pcValids_2;
  wire                CsrPlugin_pipelineLiberator_active;
  reg                 CsrPlugin_pipelineLiberator_done;
  wire                CsrPlugin_interruptJump /* verilator public */ ;
  reg                 CsrPlugin_hadException;
  wire       [1:0]    CsrPlugin_targetPrivilege;
  wire       [3:0]    CsrPlugin_trapCause;
  reg        [1:0]    CsrPlugin_xtvec_mode;
  reg        [29:0]   CsrPlugin_xtvec_base;
  reg                 execute_CsrPlugin_wfiWake;
  wire                execute_CsrPlugin_blockedBySideEffects;
  reg                 execute_CsrPlugin_illegalAccess;
  reg                 execute_CsrPlugin_illegalInstruction;
  wire       [31:0]   execute_CsrPlugin_readData;
  wire                execute_CsrPlugin_writeInstruction;
  wire                execute_CsrPlugin_readInstruction;
  wire                execute_CsrPlugin_writeEnable;
  wire                execute_CsrPlugin_readEnable;
  wire       [31:0]   execute_CsrPlugin_readToWriteData;
  reg        [31:0]   execute_CsrPlugin_writeData;
  wire       [11:0]   execute_CsrPlugin_csrAddress;
  wire       [26:0]   _zz_71_;
  wire                _zz_72_;
  wire                _zz_73_;
  wire                _zz_74_;
  wire                _zz_75_;
  wire                _zz_76_;
  wire                _zz_77_;
  wire                _zz_78_;
  wire       `Src1CtrlEnum_defaultEncoding_type _zz_79_;
  wire       `ShiftCtrlEnum_defaultEncoding_type _zz_80_;
  wire       `AluCtrlEnum_defaultEncoding_type _zz_81_;
  wire       `BranchCtrlEnum_defaultEncoding_type _zz_82_;
  wire       `EnvCtrlEnum_defaultEncoding_type _zz_83_;
  wire       `Src2CtrlEnum_defaultEncoding_type _zz_84_;
  wire       `AluBitwiseCtrlEnum_defaultEncoding_type _zz_85_;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress1;
  wire       [4:0]    decode_RegFilePlugin_regFileReadAddress2;
  wire       [31:0]   decode_RegFilePlugin_rs1Data;
  wire       [31:0]   decode_RegFilePlugin_rs2Data;
  reg                 lastStageRegFileWrite_valid /* verilator public */ ;
  wire       [4:0]    lastStageRegFileWrite_payload_address /* verilator public */ ;
  wire       [31:0]   lastStageRegFileWrite_payload_data /* verilator public */ ;
  reg                 _zz_86_;
  reg        [31:0]   execute_IntAluPlugin_bitwise;
  reg        [31:0]   _zz_87_;
  reg        [31:0]   _zz_88_;
  wire                _zz_89_;
  reg        [19:0]   _zz_90_;
  wire                _zz_91_;
  reg        [19:0]   _zz_92_;
  reg        [31:0]   _zz_93_;
  reg        [31:0]   execute_SrcPlugin_addSub;
  wire                execute_SrcPlugin_less;
  reg                 execute_LightShifterPlugin_isActive;
  wire                execute_LightShifterPlugin_isShift;
  reg        [4:0]    execute_LightShifterPlugin_amplitudeReg;
  wire       [4:0]    execute_LightShifterPlugin_amplitude;
  wire       [31:0]   execute_LightShifterPlugin_shiftInput;
  wire                execute_LightShifterPlugin_done;
  reg        [31:0]   _zz_94_;
  reg                 _zz_95_;
  reg                 _zz_96_;
  reg                 _zz_97_;
  reg        [4:0]    _zz_98_;
  wire                execute_BranchPlugin_eq;
  wire       [2:0]    _zz_99_;
  reg                 _zz_100_;
  reg                 _zz_101_;
  wire       [31:0]   execute_BranchPlugin_branch_src1;
  wire                _zz_102_;
  reg        [10:0]   _zz_103_;
  wire                _zz_104_;
  reg        [19:0]   _zz_105_;
  wire                _zz_106_;
  reg        [18:0]   _zz_107_;
  reg        [31:0]   _zz_108_;
  wire       [31:0]   execute_BranchPlugin_branch_src2;
  wire       [31:0]   execute_BranchPlugin_branchAdder;
  reg                 DebugPlugin_firstCycle;
  reg                 DebugPlugin_secondCycle;
  reg                 DebugPlugin_resetIt;
  reg                 DebugPlugin_haltIt;
  reg                 DebugPlugin_stepIt;
  reg                 DebugPlugin_isPipBusy;
  reg                 DebugPlugin_godmode;
  reg                 DebugPlugin_haltedByBreak;
  reg        [31:0]   DebugPlugin_busReadDataReg;
  reg                 _zz_109_;
  reg                 DebugPlugin_resetIt_regNext;
  reg        [31:0]   decode_to_execute_RS2;
  reg                 decode_to_execute_DO_EBREAK;
  reg        [31:0]   decode_to_execute_SRC1;
  reg        [31:0]   execute_to_memory_BRANCH_CALC;
  reg        `EnvCtrlEnum_defaultEncoding_type decode_to_execute_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type execute_to_memory_ENV_CTRL;
  reg        `EnvCtrlEnum_defaultEncoding_type memory_to_writeBack_ENV_CTRL;
  reg                 decode_to_execute_IS_CSR;
  reg        [31:0]   decode_to_execute_RS1;
  reg        [31:0]   decode_to_execute_PC;
  reg        [31:0]   execute_to_memory_PC;
  reg        [31:0]   memory_to_writeBack_PC;
  reg                 execute_to_memory_BRANCH_DO;
  reg                 decode_to_execute_SRC2_FORCE_ZERO;
  reg        [31:0]   memory_to_writeBack_MEMORY_READ_DATA;
  reg        [31:0]   execute_to_memory_REGFILE_WRITE_DATA;
  reg        [31:0]   memory_to_writeBack_REGFILE_WRITE_DATA;
  reg                 decode_to_execute_CSR_WRITE_OPCODE;
  reg                 decode_to_execute_SRC_USE_SUB_LESS;
  reg        `AluBitwiseCtrlEnum_defaultEncoding_type decode_to_execute_ALU_BITWISE_CTRL;
  reg        [1:0]    execute_to_memory_MEMORY_ADDRESS_LOW;
  reg        [1:0]    memory_to_writeBack_MEMORY_ADDRESS_LOW;
  reg                 decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  reg                 execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  reg                 decode_to_execute_REGFILE_WRITE_VALID;
  reg                 execute_to_memory_REGFILE_WRITE_VALID;
  reg                 memory_to_writeBack_REGFILE_WRITE_VALID;
  reg        [31:0]   decode_to_execute_FORMAL_PC_NEXT;
  reg        [31:0]   execute_to_memory_FORMAL_PC_NEXT;
  reg        [31:0]   memory_to_writeBack_FORMAL_PC_NEXT;
  reg                 decode_to_execute_MEMORY_ENABLE;
  reg                 execute_to_memory_MEMORY_ENABLE;
  reg                 memory_to_writeBack_MEMORY_ENABLE;
  reg        [31:0]   decode_to_execute_SRC2;
  reg                 decode_to_execute_MEMORY_STORE;
  reg                 execute_to_memory_MEMORY_STORE;
  reg                 memory_to_writeBack_MEMORY_STORE;
  reg        `BranchCtrlEnum_defaultEncoding_type decode_to_execute_BRANCH_CTRL;
  reg                 decode_to_execute_CSR_READ_OPCODE;
  reg        `AluCtrlEnum_defaultEncoding_type decode_to_execute_ALU_CTRL;
  reg                 decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  reg        `ShiftCtrlEnum_defaultEncoding_type decode_to_execute_SHIFT_CTRL;
  reg                 decode_to_execute_SRC_LESS_UNSIGNED;
  reg                 decode_to_execute_IS_SIMD_ADD;
  reg        [31:0]   decode_to_execute_INSTRUCTION;
  reg        [31:0]   execute_to_memory_INSTRUCTION;
  reg        [31:0]   memory_to_writeBack_INSTRUCTION;
  reg        [2:0]    _zz_110_;
  reg                 execute_CsrPlugin_csr_768;
  reg                 execute_CsrPlugin_csr_836;
  reg                 execute_CsrPlugin_csr_772;
  reg                 execute_CsrPlugin_csr_834;
  reg        [31:0]   _zz_111_;
  reg        [31:0]   _zz_112_;
  reg        [31:0]   _zz_113_;
  reg        [31:0]   _zz_114_;
  `ifndef SYNTHESIS
  reg [71:0] decode_SHIFT_CTRL_string;
  reg [71:0] _zz_1__string;
  reg [71:0] _zz_2__string;
  reg [71:0] _zz_3__string;
  reg [63:0] decode_ALU_CTRL_string;
  reg [63:0] _zz_4__string;
  reg [63:0] _zz_5__string;
  reg [63:0] _zz_6__string;
  reg [31:0] decode_BRANCH_CTRL_string;
  reg [31:0] _zz_7__string;
  reg [31:0] _zz_8__string;
  reg [31:0] _zz_9__string;
  reg [39:0] decode_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_10__string;
  reg [39:0] _zz_11__string;
  reg [39:0] _zz_12__string;
  reg [31:0] _zz_13__string;
  reg [31:0] _zz_14__string;
  reg [31:0] _zz_15__string;
  reg [31:0] _zz_16__string;
  reg [31:0] decode_ENV_CTRL_string;
  reg [31:0] _zz_17__string;
  reg [31:0] _zz_18__string;
  reg [31:0] _zz_19__string;
  reg [31:0] execute_BRANCH_CTRL_string;
  reg [31:0] _zz_20__string;
  reg [71:0] execute_SHIFT_CTRL_string;
  reg [71:0] _zz_21__string;
  reg [23:0] decode_SRC2_CTRL_string;
  reg [23:0] _zz_24__string;
  reg [95:0] decode_SRC1_CTRL_string;
  reg [95:0] _zz_26__string;
  reg [63:0] execute_ALU_CTRL_string;
  reg [63:0] _zz_27__string;
  reg [39:0] execute_ALU_BITWISE_CTRL_string;
  reg [39:0] _zz_28__string;
  reg [39:0] _zz_32__string;
  reg [23:0] _zz_33__string;
  reg [31:0] _zz_34__string;
  reg [31:0] _zz_35__string;
  reg [63:0] _zz_36__string;
  reg [71:0] _zz_37__string;
  reg [95:0] _zz_38__string;
  reg [31:0] memory_ENV_CTRL_string;
  reg [31:0] _zz_39__string;
  reg [31:0] execute_ENV_CTRL_string;
  reg [31:0] _zz_40__string;
  reg [31:0] writeBack_ENV_CTRL_string;
  reg [31:0] _zz_41__string;
  reg [95:0] _zz_79__string;
  reg [71:0] _zz_80__string;
  reg [63:0] _zz_81__string;
  reg [31:0] _zz_82__string;
  reg [31:0] _zz_83__string;
  reg [23:0] _zz_84__string;
  reg [39:0] _zz_85__string;
  reg [31:0] decode_to_execute_ENV_CTRL_string;
  reg [31:0] execute_to_memory_ENV_CTRL_string;
  reg [31:0] memory_to_writeBack_ENV_CTRL_string;
  reg [39:0] decode_to_execute_ALU_BITWISE_CTRL_string;
  reg [31:0] decode_to_execute_BRANCH_CTRL_string;
  reg [63:0] decode_to_execute_ALU_CTRL_string;
  reg [71:0] decode_to_execute_SHIFT_CTRL_string;
  `endif

  reg [31:0] RegFilePlugin_regFile [0:31] /* verilator public */ ;

  assign _zz_119_ = (execute_arbitration_isValid && execute_IS_CSR);
  assign _zz_120_ = ((execute_arbitration_isValid && execute_LightShifterPlugin_isShift) && (execute_SRC2[4 : 0] != 5'h0));
  assign _zz_121_ = (! execute_arbitration_isStuckByOthers);
  assign _zz_122_ = (execute_arbitration_isValid && execute_DO_EBREAK);
  assign _zz_123_ = (({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00)) == 1'b0);
  assign _zz_124_ = (CsrPlugin_hadException || CsrPlugin_interruptJump);
  assign _zz_125_ = (writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET));
  assign _zz_126_ = (DebugPlugin_stepIt && IBusSimplePlugin_incomingInstruction);
  assign _zz_127_ = writeBack_INSTRUCTION[29 : 28];
  assign _zz_128_ = (writeBack_arbitration_isValid && writeBack_REGFILE_WRITE_VALID);
  assign _zz_129_ = (1'b1 || (! 1'b1));
  assign _zz_130_ = (memory_arbitration_isValid && memory_REGFILE_WRITE_VALID);
  assign _zz_131_ = (1'b1 || (! memory_BYPASSABLE_MEMORY_STAGE));
  assign _zz_132_ = (execute_arbitration_isValid && execute_REGFILE_WRITE_VALID);
  assign _zz_133_ = (1'b1 || (! execute_BYPASSABLE_EXECUTE_STAGE));
  assign _zz_134_ = debug_bus_cmd_payload_address[7 : 2];
  assign _zz_135_ = (CsrPlugin_mstatus_MIE || (CsrPlugin_privilege < (2'b11)));
  assign _zz_136_ = ((_zz_68_ && 1'b1) && (! 1'b0));
  assign _zz_137_ = ((_zz_69_ && 1'b1) && (! 1'b0));
  assign _zz_138_ = ((_zz_70_ && 1'b1) && (! 1'b0));
  assign _zz_139_ = writeBack_INSTRUCTION[13 : 12];
  assign _zz_140_ = execute_INSTRUCTION[13];
  assign _zz_141_ = _zz_71_[8 : 8];
  assign _zz_142_ = _zz_71_[19 : 19];
  assign _zz_143_ = _zz_71_[9 : 9];
  assign _zz_144_ = _zz_71_[10 : 10];
  assign _zz_145_ = _zz_71_[11 : 11];
  assign _zz_146_ = _zz_71_[20 : 20];
  assign _zz_147_ = _zz_71_[0 : 0];
  assign _zz_148_ = _zz_71_[4 : 4];
  assign _zz_149_ = _zz_71_[1 : 1];
  assign _zz_150_ = _zz_71_[5 : 5];
  assign _zz_151_ = _zz_71_[21 : 21];
  assign _zz_152_ = _zz_71_[17 : 17];
  assign _zz_153_ = _zz_71_[18 : 18];
  assign _zz_154_ = (execute_SimdAddPlugin_rs1 + execute_SimdAddPlugin_rs2);
  assign _zz_155_ = (_zz_45_ & (~ _zz_156_));
  assign _zz_156_ = (_zz_45_ - (2'b01));
  assign _zz_157_ = {IBusSimplePlugin_fetchPc_inc,(2'b00)};
  assign _zz_158_ = {29'd0, _zz_157_};
  assign _zz_159_ = (IBusSimplePlugin_pending_value + _zz_161_);
  assign _zz_160_ = IBusSimplePlugin_pending_inc;
  assign _zz_161_ = {2'd0, _zz_160_};
  assign _zz_162_ = IBusSimplePlugin_pending_dec;
  assign _zz_163_ = {2'd0, _zz_162_};
  assign _zz_164_ = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)));
  assign _zz_165_ = {2'd0, _zz_164_};
  assign _zz_166_ = execute_SRC_LESS;
  assign _zz_167_ = (3'b100);
  assign _zz_168_ = decode_INSTRUCTION[19 : 15];
  assign _zz_169_ = decode_INSTRUCTION[31 : 20];
  assign _zz_170_ = {decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]};
  assign _zz_171_ = ($signed(_zz_172_) + $signed(_zz_175_));
  assign _zz_172_ = ($signed(_zz_173_) + $signed(_zz_174_));
  assign _zz_173_ = execute_SRC1;
  assign _zz_174_ = (execute_SRC_USE_SUB_LESS ? (~ execute_SRC2) : execute_SRC2);
  assign _zz_175_ = (execute_SRC_USE_SUB_LESS ? _zz_176_ : _zz_177_);
  assign _zz_176_ = 32'h00000001;
  assign _zz_177_ = 32'h0;
  assign _zz_178_ = (_zz_179_ >>> 1);
  assign _zz_179_ = {((execute_SHIFT_CTRL == `ShiftCtrlEnum_defaultEncoding_SRA_1) && execute_LightShifterPlugin_shiftInput[31]),execute_LightShifterPlugin_shiftInput};
  assign _zz_180_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]};
  assign _zz_181_ = execute_INSTRUCTION[31 : 20];
  assign _zz_182_ = {{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]};
  assign _zz_183_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_184_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_185_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_186_ = execute_CsrPlugin_writeData[11 : 11];
  assign _zz_187_ = execute_CsrPlugin_writeData[7 : 7];
  assign _zz_188_ = execute_CsrPlugin_writeData[3 : 3];
  assign _zz_189_ = 1'b1;
  assign _zz_190_ = 1'b1;
  assign _zz_191_ = 32'h00001000;
  assign _zz_192_ = (decode_INSTRUCTION & 32'h00003000);
  assign _zz_193_ = 32'h00002000;
  assign _zz_194_ = _zz_78_;
  assign _zz_195_ = ((decode_INSTRUCTION & _zz_201_) == 32'h00000020);
  assign _zz_196_ = {_zz_78_,(_zz_202_ == _zz_203_)};
  assign _zz_197_ = (2'b00);
  assign _zz_198_ = ({_zz_204_,_zz_205_} != (2'b00));
  assign _zz_199_ = ({_zz_206_,_zz_207_} != (3'b000));
  assign _zz_200_ = {(_zz_208_ != _zz_209_),{_zz_210_,{_zz_211_,_zz_212_}}};
  assign _zz_201_ = 32'h00000070;
  assign _zz_202_ = (decode_INSTRUCTION & 32'h00000020);
  assign _zz_203_ = 32'h0;
  assign _zz_204_ = ((decode_INSTRUCTION & 32'h00000050) == 32'h00000040);
  assign _zz_205_ = ((decode_INSTRUCTION & 32'h00103040) == 32'h00000040);
  assign _zz_206_ = ((decode_INSTRUCTION & _zz_213_) == 32'h00000040);
  assign _zz_207_ = {(_zz_214_ == _zz_215_),(_zz_216_ == _zz_217_)};
  assign _zz_208_ = {_zz_76_,(_zz_218_ == _zz_219_)};
  assign _zz_209_ = (2'b00);
  assign _zz_210_ = ({_zz_220_,_zz_221_} != (2'b00));
  assign _zz_211_ = ({_zz_222_,_zz_223_} != 6'h0);
  assign _zz_212_ = {(_zz_224_ != _zz_225_),{_zz_226_,{_zz_227_,_zz_228_}}};
  assign _zz_213_ = 32'h00000044;
  assign _zz_214_ = (decode_INSTRUCTION & 32'h00002014);
  assign _zz_215_ = 32'h00002010;
  assign _zz_216_ = (decode_INSTRUCTION & 32'h40004034);
  assign _zz_217_ = 32'h40000030;
  assign _zz_218_ = (decode_INSTRUCTION & 32'h00000010);
  assign _zz_219_ = 32'h00000010;
  assign _zz_220_ = ((decode_INSTRUCTION & 32'h00002010) == 32'h00002000);
  assign _zz_221_ = ((decode_INSTRUCTION & 32'h00005000) == 32'h00001000);
  assign _zz_222_ = _zz_77_;
  assign _zz_223_ = {(_zz_229_ == _zz_230_),{_zz_231_,{_zz_232_,_zz_233_}}};
  assign _zz_224_ = {(_zz_234_ == _zz_235_),(_zz_236_ == _zz_237_)};
  assign _zz_225_ = (2'b00);
  assign _zz_226_ = ((_zz_238_ == _zz_239_) != (1'b0));
  assign _zz_227_ = ({_zz_240_,_zz_241_} != (2'b00));
  assign _zz_228_ = {(_zz_242_ != _zz_243_),{_zz_244_,{_zz_245_,_zz_246_}}};
  assign _zz_229_ = (decode_INSTRUCTION & 32'h00001010);
  assign _zz_230_ = 32'h00001010;
  assign _zz_231_ = ((decode_INSTRUCTION & 32'h00002010) == 32'h00002010);
  assign _zz_232_ = _zz_75_;
  assign _zz_233_ = {(_zz_247_ == _zz_248_),(_zz_249_ == _zz_250_)};
  assign _zz_234_ = (decode_INSTRUCTION & 32'h00000064);
  assign _zz_235_ = 32'h00000024;
  assign _zz_236_ = (decode_INSTRUCTION & 32'h00003054);
  assign _zz_237_ = 32'h00001010;
  assign _zz_238_ = (decode_INSTRUCTION & 32'h00103050);
  assign _zz_239_ = 32'h00000050;
  assign _zz_240_ = _zz_77_;
  assign _zz_241_ = ((decode_INSTRUCTION & _zz_251_) == 32'h00000004);
  assign _zz_242_ = ((decode_INSTRUCTION & _zz_252_) == 32'h00000040);
  assign _zz_243_ = (1'b0);
  assign _zz_244_ = ({_zz_253_,_zz_254_} != (2'b00));
  assign _zz_245_ = (_zz_74_ != (1'b0));
  assign _zz_246_ = {(_zz_255_ != _zz_256_),{_zz_257_,{_zz_258_,_zz_259_}}};
  assign _zz_247_ = (decode_INSTRUCTION & 32'h0000000c);
  assign _zz_248_ = 32'h00000004;
  assign _zz_249_ = (decode_INSTRUCTION & 32'h00000024);
  assign _zz_250_ = 32'h0;
  assign _zz_251_ = 32'h0000001c;
  assign _zz_252_ = 32'h00000058;
  assign _zz_253_ = ((decode_INSTRUCTION & 32'h00006004) == 32'h00006000);
  assign _zz_254_ = ((decode_INSTRUCTION & 32'h00005004) == 32'h00004000);
  assign _zz_255_ = ((decode_INSTRUCTION & _zz_260_) == 32'h0);
  assign _zz_256_ = (1'b0);
  assign _zz_257_ = ((_zz_261_ == _zz_262_) != (1'b0));
  assign _zz_258_ = ({_zz_263_,_zz_264_} != (2'b00));
  assign _zz_259_ = {(_zz_265_ != _zz_266_),{_zz_267_,{_zz_268_,_zz_269_}}};
  assign _zz_260_ = 32'h00000058;
  assign _zz_261_ = (decode_INSTRUCTION & 32'h00000020);
  assign _zz_262_ = 32'h00000020;
  assign _zz_263_ = _zz_76_;
  assign _zz_264_ = _zz_75_;
  assign _zz_265_ = _zz_72_;
  assign _zz_266_ = (1'b0);
  assign _zz_267_ = (((decode_INSTRUCTION & _zz_270_) == 32'h00005010) != (1'b0));
  assign _zz_268_ = ({_zz_271_,_zz_272_} != (2'b00));
  assign _zz_269_ = {({_zz_273_,_zz_274_} != (4'b0000)),{(_zz_275_ != _zz_276_),{_zz_277_,{_zz_278_,_zz_279_}}}};
  assign _zz_270_ = 32'h00007054;
  assign _zz_271_ = ((decode_INSTRUCTION & 32'h40003054) == 32'h40001010);
  assign _zz_272_ = ((decode_INSTRUCTION & 32'h00007054) == 32'h00001010);
  assign _zz_273_ = ((decode_INSTRUCTION & _zz_280_) == 32'h0);
  assign _zz_274_ = {(_zz_281_ == _zz_282_),{_zz_74_,_zz_283_}};
  assign _zz_275_ = ((decode_INSTRUCTION & _zz_284_) == 32'h00000050);
  assign _zz_276_ = (1'b0);
  assign _zz_277_ = ({_zz_285_,_zz_73_} != (2'b00));
  assign _zz_278_ = ({_zz_286_,_zz_287_} != (2'b00));
  assign _zz_279_ = {(_zz_288_ != _zz_289_),(_zz_290_ != _zz_291_)};
  assign _zz_280_ = 32'h00000044;
  assign _zz_281_ = (decode_INSTRUCTION & 32'h00000018);
  assign _zz_282_ = 32'h0;
  assign _zz_283_ = ((decode_INSTRUCTION & 32'h00005004) == 32'h00001000);
  assign _zz_284_ = 32'h10003050;
  assign _zz_285_ = ((decode_INSTRUCTION & 32'h00000014) == 32'h00000004);
  assign _zz_286_ = ((decode_INSTRUCTION & 32'h00000044) == 32'h00000004);
  assign _zz_287_ = _zz_73_;
  assign _zz_288_ = {_zz_72_,{(_zz_292_ == _zz_293_),(_zz_294_ == _zz_295_)}};
  assign _zz_289_ = (3'b000);
  assign _zz_290_ = {((decode_INSTRUCTION & _zz_296_) == 32'h00001050),((decode_INSTRUCTION & _zz_297_) == 32'h00002050)};
  assign _zz_291_ = (2'b00);
  assign _zz_292_ = (decode_INSTRUCTION & 32'h00000034);
  assign _zz_293_ = 32'h00000020;
  assign _zz_294_ = (decode_INSTRUCTION & 32'h00000064);
  assign _zz_295_ = 32'h00000020;
  assign _zz_296_ = 32'h00001050;
  assign _zz_297_ = 32'h00002050;
  always @ (posedge io_mainClk) begin
    if(_zz_189_) begin
      _zz_117_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress1];
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_190_) begin
      _zz_118_ <= RegFilePlugin_regFile[decode_RegFilePlugin_regFileReadAddress2];
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_31_) begin
      RegFilePlugin_regFile[lastStageRegFileWrite_payload_address] <= lastStageRegFileWrite_payload_data;
    end
  end

  StreamFifoLowLatency IBusSimplePlugin_rspJoin_rspBuffer_c ( 
    .io_push_valid            (iBus_rsp_valid                                                  ), //i
    .io_push_ready            (IBusSimplePlugin_rspJoin_rspBuffer_c_io_push_ready              ), //o
    .io_push_payload_error    (iBus_rsp_payload_error                                          ), //i
    .io_push_payload_inst     (iBus_rsp_payload_inst[31:0]                                     ), //i
    .io_pop_valid             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid               ), //o
    .io_pop_ready             (_zz_115_                                                        ), //i
    .io_pop_payload_error     (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error       ), //o
    .io_pop_payload_inst      (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst[31:0]  ), //o
    .io_flush                 (_zz_116_                                                        ), //i
    .io_occupancy             (IBusSimplePlugin_rspJoin_rspBuffer_c_io_occupancy               ), //o
    .io_mainClk               (io_mainClk                                                      ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                                           )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(decode_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_1__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_1__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_1__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_1__string = "SRA_1    ";
      default : _zz_1__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_2_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_2__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_2__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_2__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_2__string = "SRA_1    ";
      default : _zz_2__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_3_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_3__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_3__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_3__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_3__string = "SRA_1    ";
      default : _zz_3__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_ALU_CTRL_string = "BITWISE ";
      default : decode_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_4_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_4__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_4__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_4__string = "BITWISE ";
      default : _zz_4__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_5_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_5__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_5__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_5__string = "BITWISE ";
      default : _zz_5__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_6_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_6__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_6__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_6__string = "BITWISE ";
      default : _zz_6__string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_BRANCH_CTRL_string = "JALR";
      default : decode_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_7_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_7__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_7__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_7__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_7__string = "JALR";
      default : _zz_7__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_8_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_8__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_8__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_8__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_8__string = "JALR";
      default : _zz_8__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_9_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_9__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_9__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_9__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_9__string = "JALR";
      default : _zz_9__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_10_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_10__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_10__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_10__string = "AND_1";
      default : _zz_10__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_11_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_11__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_11__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_11__string = "AND_1";
      default : _zz_11__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_12_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_12__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_12__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_12__string = "AND_1";
      default : _zz_12__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_13_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_13__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_13__string = "XRET";
      default : _zz_13__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_14_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_14__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_14__string = "XRET";
      default : _zz_14__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_15_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_15__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_15__string = "XRET";
      default : _zz_15__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_16_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_16__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_16__string = "XRET";
      default : _zz_16__string = "????";
    endcase
  end
  always @(*) begin
    case(decode_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_ENV_CTRL_string = "XRET";
      default : decode_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_17_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_17__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_17__string = "XRET";
      default : _zz_17__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_18_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_18__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_18__string = "XRET";
      default : _zz_18__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_19_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_19__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_19__string = "XRET";
      default : _zz_19__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : execute_BRANCH_CTRL_string = "JALR";
      default : execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_20_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_20__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_20__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_20__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_20__string = "JALR";
      default : _zz_20__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : execute_SHIFT_CTRL_string = "SRA_1    ";
      default : execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_21_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_21__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_21__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_21__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_21__string = "SRA_1    ";
      default : _zz_21__string = "?????????";
    endcase
  end
  always @(*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : decode_SRC2_CTRL_string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : decode_SRC2_CTRL_string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : decode_SRC2_CTRL_string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : decode_SRC2_CTRL_string = "PC ";
      default : decode_SRC2_CTRL_string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_24_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_24__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_24__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_24__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_24__string = "PC ";
      default : _zz_24__string = "???";
    endcase
  end
  always @(*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : decode_SRC1_CTRL_string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : decode_SRC1_CTRL_string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : decode_SRC1_CTRL_string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : decode_SRC1_CTRL_string = "URS1        ";
      default : decode_SRC1_CTRL_string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_26_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_26__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_26__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_26__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_26__string = "URS1        ";
      default : _zz_26__string = "????????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : execute_ALU_CTRL_string = "BITWISE ";
      default : execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_27_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_27__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_27__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_27__string = "BITWISE ";
      default : _zz_27__string = "????????";
    endcase
  end
  always @(*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_28_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_28__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_28__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_28__string = "AND_1";
      default : _zz_28__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_32_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_32__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_32__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_32__string = "AND_1";
      default : _zz_32__string = "?????";
    endcase
  end
  always @(*) begin
    case(_zz_33_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_33__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_33__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_33__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_33__string = "PC ";
      default : _zz_33__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_34_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_34__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_34__string = "XRET";
      default : _zz_34__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_35_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_35__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_35__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_35__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_35__string = "JALR";
      default : _zz_35__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_36_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_36__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_36__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_36__string = "BITWISE ";
      default : _zz_36__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_37_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_37__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_37__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_37__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_37__string = "SRA_1    ";
      default : _zz_37__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_38_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_38__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_38__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_38__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_38__string = "URS1        ";
      default : _zz_38__string = "????????????";
    endcase
  end
  always @(*) begin
    case(memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_ENV_CTRL_string = "XRET";
      default : memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_39_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_39__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_39__string = "XRET";
      default : _zz_39__string = "????";
    endcase
  end
  always @(*) begin
    case(execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_ENV_CTRL_string = "XRET";
      default : execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_40_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_40__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_40__string = "XRET";
      default : _zz_40__string = "????";
    endcase
  end
  always @(*) begin
    case(writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : writeBack_ENV_CTRL_string = "XRET";
      default : writeBack_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_41_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_41__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_41__string = "XRET";
      default : _zz_41__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_79_)
      `Src1CtrlEnum_defaultEncoding_RS : _zz_79__string = "RS          ";
      `Src1CtrlEnum_defaultEncoding_IMU : _zz_79__string = "IMU         ";
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : _zz_79__string = "PC_INCREMENT";
      `Src1CtrlEnum_defaultEncoding_URS1 : _zz_79__string = "URS1        ";
      default : _zz_79__string = "????????????";
    endcase
  end
  always @(*) begin
    case(_zz_80_)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : _zz_80__string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : _zz_80__string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : _zz_80__string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : _zz_80__string = "SRA_1    ";
      default : _zz_80__string = "?????????";
    endcase
  end
  always @(*) begin
    case(_zz_81_)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : _zz_81__string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : _zz_81__string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : _zz_81__string = "BITWISE ";
      default : _zz_81__string = "????????";
    endcase
  end
  always @(*) begin
    case(_zz_82_)
      `BranchCtrlEnum_defaultEncoding_INC : _zz_82__string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : _zz_82__string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : _zz_82__string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : _zz_82__string = "JALR";
      default : _zz_82__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_83_)
      `EnvCtrlEnum_defaultEncoding_NONE : _zz_83__string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : _zz_83__string = "XRET";
      default : _zz_83__string = "????";
    endcase
  end
  always @(*) begin
    case(_zz_84_)
      `Src2CtrlEnum_defaultEncoding_RS : _zz_84__string = "RS ";
      `Src2CtrlEnum_defaultEncoding_IMI : _zz_84__string = "IMI";
      `Src2CtrlEnum_defaultEncoding_IMS : _zz_84__string = "IMS";
      `Src2CtrlEnum_defaultEncoding_PC : _zz_84__string = "PC ";
      default : _zz_84__string = "???";
    endcase
  end
  always @(*) begin
    case(_zz_85_)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : _zz_85__string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : _zz_85__string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : _zz_85__string = "AND_1";
      default : _zz_85__string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : decode_to_execute_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : decode_to_execute_ENV_CTRL_string = "XRET";
      default : decode_to_execute_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(execute_to_memory_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : execute_to_memory_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : execute_to_memory_ENV_CTRL_string = "XRET";
      default : execute_to_memory_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(memory_to_writeBack_ENV_CTRL)
      `EnvCtrlEnum_defaultEncoding_NONE : memory_to_writeBack_ENV_CTRL_string = "NONE";
      `EnvCtrlEnum_defaultEncoding_XRET : memory_to_writeBack_ENV_CTRL_string = "XRET";
      default : memory_to_writeBack_ENV_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_XOR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "XOR_1";
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "OR_1 ";
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : decode_to_execute_ALU_BITWISE_CTRL_string = "AND_1";
      default : decode_to_execute_ALU_BITWISE_CTRL_string = "?????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : decode_to_execute_BRANCH_CTRL_string = "INC ";
      `BranchCtrlEnum_defaultEncoding_B : decode_to_execute_BRANCH_CTRL_string = "B   ";
      `BranchCtrlEnum_defaultEncoding_JAL : decode_to_execute_BRANCH_CTRL_string = "JAL ";
      `BranchCtrlEnum_defaultEncoding_JALR : decode_to_execute_BRANCH_CTRL_string = "JALR";
      default : decode_to_execute_BRANCH_CTRL_string = "????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_ADD_SUB : decode_to_execute_ALU_CTRL_string = "ADD_SUB ";
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : decode_to_execute_ALU_CTRL_string = "SLT_SLTU";
      `AluCtrlEnum_defaultEncoding_BITWISE : decode_to_execute_ALU_CTRL_string = "BITWISE ";
      default : decode_to_execute_ALU_CTRL_string = "????????";
    endcase
  end
  always @(*) begin
    case(decode_to_execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_DISABLE_1 : decode_to_execute_SHIFT_CTRL_string = "DISABLE_1";
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : decode_to_execute_SHIFT_CTRL_string = "SLL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRL_1 : decode_to_execute_SHIFT_CTRL_string = "SRL_1    ";
      `ShiftCtrlEnum_defaultEncoding_SRA_1 : decode_to_execute_SHIFT_CTRL_string = "SRA_1    ";
      default : decode_to_execute_SHIFT_CTRL_string = "?????????";
    endcase
  end
  `endif

  assign decode_IS_SIMD_ADD = _zz_141_[0];
  assign decode_SRC_LESS_UNSIGNED = _zz_142_[0];
  assign decode_SHIFT_CTRL = _zz_1_;
  assign _zz_2_ = _zz_3_;
  assign decode_BYPASSABLE_EXECUTE_STAGE = _zz_143_[0];
  assign decode_ALU_CTRL = _zz_4_;
  assign _zz_5_ = _zz_6_;
  assign decode_CSR_READ_OPCODE = (decode_INSTRUCTION[13 : 7] != 7'h20);
  assign decode_BRANCH_CTRL = _zz_7_;
  assign _zz_8_ = _zz_9_;
  assign decode_MEMORY_STORE = _zz_144_[0];
  assign decode_SRC2 = _zz_93_;
  assign decode_MEMORY_ENABLE = _zz_145_[0];
  assign writeBack_FORMAL_PC_NEXT = memory_to_writeBack_FORMAL_PC_NEXT;
  assign memory_FORMAL_PC_NEXT = execute_to_memory_FORMAL_PC_NEXT;
  assign execute_FORMAL_PC_NEXT = decode_to_execute_FORMAL_PC_NEXT;
  assign decode_FORMAL_PC_NEXT = (decode_PC + 32'h00000004);
  assign execute_BYPASSABLE_MEMORY_STAGE = decode_to_execute_BYPASSABLE_MEMORY_STAGE;
  assign decode_BYPASSABLE_MEMORY_STAGE = _zz_146_[0];
  assign memory_MEMORY_ADDRESS_LOW = execute_to_memory_MEMORY_ADDRESS_LOW;
  assign execute_MEMORY_ADDRESS_LOW = dBus_cmd_payload_address[1 : 0];
  assign decode_ALU_BITWISE_CTRL = _zz_10_;
  assign _zz_11_ = _zz_12_;
  assign decode_CSR_WRITE_OPCODE = (! (((decode_INSTRUCTION[14 : 13] == (2'b01)) && (decode_INSTRUCTION[19 : 15] == 5'h0)) || ((decode_INSTRUCTION[14 : 13] == (2'b11)) && (decode_INSTRUCTION[19 : 15] == 5'h0))));
  assign writeBack_REGFILE_WRITE_DATA = memory_to_writeBack_REGFILE_WRITE_DATA;
  assign execute_REGFILE_WRITE_DATA = _zz_87_;
  assign memory_MEMORY_READ_DATA = dBus_rsp_data;
  assign decode_SRC2_FORCE_ZERO = (decode_SRC_ADD_ZERO && (! decode_SRC_USE_SUB_LESS));
  assign execute_BRANCH_DO = _zz_101_;
  assign memory_PC = execute_to_memory_PC;
  assign decode_RS1 = decode_RegFilePlugin_rs1Data;
  assign decode_IS_CSR = _zz_147_[0];
  assign _zz_13_ = _zz_14_;
  assign _zz_15_ = _zz_16_;
  assign decode_ENV_CTRL = _zz_17_;
  assign _zz_18_ = _zz_19_;
  assign execute_BRANCH_CALC = {execute_BranchPlugin_branchAdder[31 : 1],(1'b0)};
  assign decode_SRC1 = _zz_88_;
  assign decode_DO_EBREAK = ((! DebugPlugin_haltIt) && (decode_IS_EBREAK || 1'b0));
  assign decode_RS2 = decode_RegFilePlugin_rs2Data;
  assign execute_DO_EBREAK = decode_to_execute_DO_EBREAK;
  assign decode_IS_EBREAK = _zz_148_[0];
  assign memory_BRANCH_CALC = execute_to_memory_BRANCH_CALC;
  assign memory_BRANCH_DO = execute_to_memory_BRANCH_DO;
  assign execute_PC = decode_to_execute_PC;
  assign execute_BRANCH_CTRL = _zz_20_;
  assign decode_RS2_USE = _zz_149_[0];
  assign decode_RS1_USE = _zz_150_[0];
  assign execute_REGFILE_WRITE_VALID = decode_to_execute_REGFILE_WRITE_VALID;
  assign execute_BYPASSABLE_EXECUTE_STAGE = decode_to_execute_BYPASSABLE_EXECUTE_STAGE;
  assign memory_REGFILE_WRITE_VALID = execute_to_memory_REGFILE_WRITE_VALID;
  assign memory_INSTRUCTION = execute_to_memory_INSTRUCTION;
  assign memory_BYPASSABLE_MEMORY_STAGE = execute_to_memory_BYPASSABLE_MEMORY_STAGE;
  assign writeBack_REGFILE_WRITE_VALID = memory_to_writeBack_REGFILE_WRITE_VALID;
  assign memory_REGFILE_WRITE_DATA = execute_to_memory_REGFILE_WRITE_DATA;
  assign execute_SHIFT_CTRL = _zz_21_;
  assign execute_SRC_LESS_UNSIGNED = decode_to_execute_SRC_LESS_UNSIGNED;
  assign execute_SRC2_FORCE_ZERO = decode_to_execute_SRC2_FORCE_ZERO;
  assign execute_SRC_USE_SUB_LESS = decode_to_execute_SRC_USE_SUB_LESS;
  assign _zz_22_ = decode_PC;
  assign _zz_23_ = decode_RS2;
  assign decode_SRC2_CTRL = _zz_24_;
  assign _zz_25_ = decode_RS1;
  assign decode_SRC1_CTRL = _zz_26_;
  assign decode_SRC_USE_SUB_LESS = _zz_151_[0];
  assign decode_SRC_ADD_ZERO = _zz_152_[0];
  assign execute_SRC_ADD_SUB = execute_SrcPlugin_addSub;
  assign execute_SRC_LESS = execute_SrcPlugin_less;
  assign execute_ALU_CTRL = _zz_27_;
  assign execute_SRC2 = decode_to_execute_SRC2;
  assign execute_ALU_BITWISE_CTRL = _zz_28_;
  assign _zz_29_ = writeBack_INSTRUCTION;
  assign _zz_30_ = writeBack_REGFILE_WRITE_VALID;
  always @ (*) begin
    _zz_31_ = 1'b0;
    if(lastStageRegFileWrite_valid)begin
      _zz_31_ = 1'b1;
    end
  end

  assign decode_INSTRUCTION_ANTICIPATED = (decode_arbitration_isStuck ? decode_INSTRUCTION : IBusSimplePlugin_iBusRsp_output_payload_rsp_inst);
  always @ (*) begin
    decode_REGFILE_WRITE_VALID = _zz_153_[0];
    if((decode_INSTRUCTION[11 : 7] == 5'h0))begin
      decode_REGFILE_WRITE_VALID = 1'b0;
    end
  end

  assign execute_SRC1 = decode_to_execute_SRC1;
  assign execute_CSR_READ_OPCODE = decode_to_execute_CSR_READ_OPCODE;
  assign execute_CSR_WRITE_OPCODE = decode_to_execute_CSR_WRITE_OPCODE;
  assign execute_IS_CSR = decode_to_execute_IS_CSR;
  assign memory_ENV_CTRL = _zz_39_;
  assign execute_ENV_CTRL = _zz_40_;
  assign writeBack_ENV_CTRL = _zz_41_;
  assign writeBack_MEMORY_STORE = memory_to_writeBack_MEMORY_STORE;
  always @ (*) begin
    _zz_42_ = writeBack_REGFILE_WRITE_DATA;
    if((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE))begin
      _zz_42_ = writeBack_DBusSimplePlugin_rspFormated;
    end
  end

  assign writeBack_MEMORY_ENABLE = memory_to_writeBack_MEMORY_ENABLE;
  assign writeBack_MEMORY_ADDRESS_LOW = memory_to_writeBack_MEMORY_ADDRESS_LOW;
  assign writeBack_MEMORY_READ_DATA = memory_to_writeBack_MEMORY_READ_DATA;
  assign memory_MEMORY_STORE = execute_to_memory_MEMORY_STORE;
  assign memory_MEMORY_ENABLE = execute_to_memory_MEMORY_ENABLE;
  assign execute_SRC_ADD = execute_SrcPlugin_addSub;
  assign execute_INSTRUCTION = decode_to_execute_INSTRUCTION;
  assign execute_MEMORY_STORE = decode_to_execute_MEMORY_STORE;
  assign execute_MEMORY_ENABLE = decode_to_execute_MEMORY_ENABLE;
  assign execute_ALIGNEMENT_FAULT = 1'b0;
  always @ (*) begin
    _zz_43_ = memory_FORMAL_PC_NEXT;
    if(BranchPlugin_jumpInterface_valid)begin
      _zz_43_ = BranchPlugin_jumpInterface_payload;
    end
  end

  assign decode_PC = IBusSimplePlugin_injector_decodeInput_payload_pc;
  assign decode_INSTRUCTION = IBusSimplePlugin_injector_decodeInput_payload_rsp_inst;
  always @ (*) begin
    _zz_44_ = execute_REGFILE_WRITE_DATA;
    if(execute_IS_SIMD_ADD)begin
      _zz_44_ = execute_SimdAddPlugin_rd;
    end
    if(_zz_119_)begin
      _zz_44_ = execute_CsrPlugin_readData;
    end
    if(_zz_120_)begin
      _zz_44_ = _zz_94_;
    end
  end

  assign execute_IS_SIMD_ADD = decode_to_execute_IS_SIMD_ADD;
  assign execute_RS2 = decode_to_execute_RS2;
  assign execute_RS1 = decode_to_execute_RS1;
  assign writeBack_PC = memory_to_writeBack_PC;
  assign writeBack_INSTRUCTION = memory_to_writeBack_INSTRUCTION;
  always @ (*) begin
    decode_arbitration_haltItself = 1'b0;
    case(_zz_110_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_haltItself = 1'b1;
      end
      3'b011 : begin
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    decode_arbitration_haltByOther = 1'b0;
    if(CsrPlugin_pipelineLiberator_active)begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if(({(writeBack_arbitration_isValid && (writeBack_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),{(memory_arbitration_isValid && (memory_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)),(execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET))}} != (3'b000)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
    if((decode_arbitration_isValid && (_zz_95_ || _zz_96_)))begin
      decode_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    decode_arbitration_removeIt = 1'b0;
    if(decode_arbitration_isFlushed)begin
      decode_arbitration_removeIt = 1'b1;
    end
  end

  assign decode_arbitration_flushIt = 1'b0;
  assign decode_arbitration_flushNext = 1'b0;
  always @ (*) begin
    execute_arbitration_haltItself = 1'b0;
    if(((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! dBus_cmd_ready)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_61_)))begin
      execute_arbitration_haltItself = 1'b1;
    end
    if(_zz_119_)begin
      if(execute_CsrPlugin_blockedBySideEffects)begin
        execute_arbitration_haltItself = 1'b1;
      end
    end
    if(_zz_120_)begin
      if(_zz_121_)begin
        if(! execute_LightShifterPlugin_done) begin
          execute_arbitration_haltItself = 1'b1;
        end
      end
    end
  end

  always @ (*) begin
    execute_arbitration_haltByOther = 1'b0;
    if(_zz_122_)begin
      execute_arbitration_haltByOther = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_removeIt = 1'b0;
    if(execute_arbitration_isFlushed)begin
      execute_arbitration_removeIt = 1'b1;
    end
  end

  always @ (*) begin
    execute_arbitration_flushIt = 1'b0;
    if(_zz_122_)begin
      if(_zz_123_)begin
        execute_arbitration_flushIt = 1'b1;
      end
    end
  end

  always @ (*) begin
    execute_arbitration_flushNext = 1'b0;
    if(_zz_122_)begin
      if(_zz_123_)begin
        execute_arbitration_flushNext = 1'b1;
      end
    end
  end

  always @ (*) begin
    memory_arbitration_haltItself = 1'b0;
    if((((memory_arbitration_isValid && memory_MEMORY_ENABLE) && (! memory_MEMORY_STORE)) && ((! dBus_rsp_ready) || 1'b0)))begin
      memory_arbitration_haltItself = 1'b1;
    end
  end

  assign memory_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    memory_arbitration_removeIt = 1'b0;
    if(memory_arbitration_isFlushed)begin
      memory_arbitration_removeIt = 1'b1;
    end
  end

  assign memory_arbitration_flushIt = 1'b0;
  always @ (*) begin
    memory_arbitration_flushNext = 1'b0;
    if(BranchPlugin_jumpInterface_valid)begin
      memory_arbitration_flushNext = 1'b1;
    end
  end

  assign writeBack_arbitration_haltItself = 1'b0;
  assign writeBack_arbitration_haltByOther = 1'b0;
  always @ (*) begin
    writeBack_arbitration_removeIt = 1'b0;
    if(writeBack_arbitration_isFlushed)begin
      writeBack_arbitration_removeIt = 1'b1;
    end
  end

  assign writeBack_arbitration_flushIt = 1'b0;
  always @ (*) begin
    writeBack_arbitration_flushNext = 1'b0;
    if(_zz_124_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
    if(_zz_125_)begin
      writeBack_arbitration_flushNext = 1'b1;
    end
  end

  assign lastStageInstruction = writeBack_INSTRUCTION;
  assign lastStagePc = writeBack_PC;
  assign lastStageIsValid = writeBack_arbitration_isValid;
  assign lastStageIsFiring = writeBack_arbitration_isFiring;
  always @ (*) begin
    IBusSimplePlugin_fetcherHalt = 1'b0;
    if(_zz_124_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_125_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_122_)begin
      if(_zz_123_)begin
        IBusSimplePlugin_fetcherHalt = 1'b1;
      end
    end
    if(DebugPlugin_haltIt)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
    if(_zz_126_)begin
      IBusSimplePlugin_fetcherHalt = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_incomingInstruction = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid || IBusSimplePlugin_iBusRsp_stages_2_input_valid))begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_incomingInstruction = 1'b1;
    end
  end

  assign CsrPlugin_inWfi = 1'b0;
  always @ (*) begin
    CsrPlugin_thirdPartyWake = 1'b0;
    if(DebugPlugin_haltIt)begin
      CsrPlugin_thirdPartyWake = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_valid = 1'b0;
    if(_zz_124_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
    if(_zz_125_)begin
      CsrPlugin_jumpInterface_valid = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_jumpInterface_payload = 32'h0;
    if(_zz_124_)begin
      CsrPlugin_jumpInterface_payload = {CsrPlugin_xtvec_base,(2'b00)};
    end
    if(_zz_125_)begin
      case(_zz_127_)
        2'b11 : begin
          CsrPlugin_jumpInterface_payload = CsrPlugin_mepc;
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    CsrPlugin_forceMachineWire = 1'b0;
    if(DebugPlugin_godmode)begin
      CsrPlugin_forceMachineWire = 1'b1;
    end
  end

  always @ (*) begin
    CsrPlugin_allowInterrupts = 1'b1;
    if((DebugPlugin_haltIt || DebugPlugin_stepIt))begin
      CsrPlugin_allowInterrupts = 1'b0;
    end
  end

  always @ (*) begin
    CsrPlugin_allowException = 1'b1;
    if(DebugPlugin_godmode)begin
      CsrPlugin_allowException = 1'b0;
    end
  end

  assign execute_SimdAddPlugin_rs1 = execute_RS1;
  assign execute_SimdAddPlugin_rs2 = execute_RS2;
  assign execute_SimdAddPlugin_rd = (_zz_154_ + 32'h00000002);
  assign IBusSimplePlugin_externalFlush = ({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,{execute_arbitration_flushNext,decode_arbitration_flushNext}}} != (4'b0000));
  assign IBusSimplePlugin_jump_pcLoad_valid = ({BranchPlugin_jumpInterface_valid,CsrPlugin_jumpInterface_valid} != (2'b00));
  assign _zz_45_ = {BranchPlugin_jumpInterface_valid,CsrPlugin_jumpInterface_valid};
  assign IBusSimplePlugin_jump_pcLoad_payload = (_zz_155_[0] ? CsrPlugin_jumpInterface_payload : BranchPlugin_jumpInterface_payload);
  always @ (*) begin
    IBusSimplePlugin_fetchPc_correction = 1'b0;
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_correction = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_corrected = (IBusSimplePlugin_fetchPc_correction || IBusSimplePlugin_fetchPc_correctionReg);
  always @ (*) begin
    IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b0;
    if(IBusSimplePlugin_iBusRsp_stages_1_input_ready)begin
      IBusSimplePlugin_fetchPc_pcRegPropagate = 1'b1;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_pc = (IBusSimplePlugin_fetchPc_pcReg + _zz_158_);
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_pc = IBusSimplePlugin_jump_pcLoad_payload;
    end
    IBusSimplePlugin_fetchPc_pc[0] = 1'b0;
    IBusSimplePlugin_fetchPc_pc[1] = 1'b0;
  end

  always @ (*) begin
    IBusSimplePlugin_fetchPc_flushed = 1'b0;
    if(IBusSimplePlugin_jump_pcLoad_valid)begin
      IBusSimplePlugin_fetchPc_flushed = 1'b1;
    end
  end

  assign IBusSimplePlugin_fetchPc_output_valid = ((! IBusSimplePlugin_fetcherHalt) && IBusSimplePlugin_fetchPc_booted);
  assign IBusSimplePlugin_fetchPc_output_payload = IBusSimplePlugin_fetchPc_pc;
  assign IBusSimplePlugin_iBusRsp_redoFetch = 1'b0;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_valid = IBusSimplePlugin_fetchPc_output_valid;
  assign IBusSimplePlugin_fetchPc_output_ready = IBusSimplePlugin_iBusRsp_stages_0_input_ready;
  assign IBusSimplePlugin_iBusRsp_stages_0_input_payload = IBusSimplePlugin_fetchPc_output_payload;
  assign IBusSimplePlugin_iBusRsp_stages_0_halt = 1'b0;
  assign _zz_46_ = (! IBusSimplePlugin_iBusRsp_stages_0_halt);
  assign IBusSimplePlugin_iBusRsp_stages_0_input_ready = (IBusSimplePlugin_iBusRsp_stages_0_output_ready && _zz_46_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_valid = (IBusSimplePlugin_iBusRsp_stages_0_input_valid && _zz_46_);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_payload = IBusSimplePlugin_iBusRsp_stages_0_input_payload;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b0;
    if((IBusSimplePlugin_iBusRsp_stages_1_input_valid && ((! IBusSimplePlugin_cmdFork_canEmit) || (! IBusSimplePlugin_cmd_ready))))begin
      IBusSimplePlugin_iBusRsp_stages_1_halt = 1'b1;
    end
  end

  assign _zz_47_ = (! IBusSimplePlugin_iBusRsp_stages_1_halt);
  assign IBusSimplePlugin_iBusRsp_stages_1_input_ready = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && _zz_47_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && _zz_47_);
  assign IBusSimplePlugin_iBusRsp_stages_1_output_payload = IBusSimplePlugin_iBusRsp_stages_1_input_payload;
  assign IBusSimplePlugin_iBusRsp_stages_2_halt = 1'b0;
  assign _zz_48_ = (! IBusSimplePlugin_iBusRsp_stages_2_halt);
  assign IBusSimplePlugin_iBusRsp_stages_2_input_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_ready && _zz_48_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_valid = (IBusSimplePlugin_iBusRsp_stages_2_input_valid && _zz_48_);
  assign IBusSimplePlugin_iBusRsp_stages_2_output_payload = IBusSimplePlugin_iBusRsp_stages_2_input_payload;
  assign IBusSimplePlugin_iBusRsp_flush = (IBusSimplePlugin_externalFlush || IBusSimplePlugin_iBusRsp_redoFetch);
  assign IBusSimplePlugin_iBusRsp_stages_0_output_ready = _zz_49_;
  assign _zz_49_ = ((1'b0 && (! _zz_50_)) || IBusSimplePlugin_iBusRsp_stages_1_input_ready);
  assign _zz_50_ = _zz_51_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_valid = _zz_50_;
  assign IBusSimplePlugin_iBusRsp_stages_1_input_payload = IBusSimplePlugin_fetchPc_pcReg;
  assign IBusSimplePlugin_iBusRsp_stages_1_output_ready = ((1'b0 && (! _zz_52_)) || IBusSimplePlugin_iBusRsp_stages_2_input_ready);
  assign _zz_52_ = _zz_53_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_valid = _zz_52_;
  assign IBusSimplePlugin_iBusRsp_stages_2_input_payload = _zz_54_;
  always @ (*) begin
    IBusSimplePlugin_iBusRsp_readyForError = 1'b1;
    if(IBusSimplePlugin_injector_decodeInput_valid)begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
    if((! IBusSimplePlugin_pcValids_0))begin
      IBusSimplePlugin_iBusRsp_readyForError = 1'b0;
    end
  end

  assign IBusSimplePlugin_iBusRsp_output_ready = ((1'b0 && (! IBusSimplePlugin_injector_decodeInput_valid)) || IBusSimplePlugin_injector_decodeInput_ready);
  assign IBusSimplePlugin_injector_decodeInput_valid = _zz_55_;
  assign IBusSimplePlugin_injector_decodeInput_payload_pc = _zz_56_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_error = _zz_57_;
  assign IBusSimplePlugin_injector_decodeInput_payload_rsp_inst = _zz_58_;
  assign IBusSimplePlugin_injector_decodeInput_payload_isRvc = _zz_59_;
  assign IBusSimplePlugin_pcValids_0 = IBusSimplePlugin_injector_nextPcCalc_valids_2;
  assign IBusSimplePlugin_pcValids_1 = IBusSimplePlugin_injector_nextPcCalc_valids_3;
  assign IBusSimplePlugin_pcValids_2 = IBusSimplePlugin_injector_nextPcCalc_valids_4;
  assign IBusSimplePlugin_pcValids_3 = IBusSimplePlugin_injector_nextPcCalc_valids_5;
  assign IBusSimplePlugin_injector_decodeInput_ready = (! decode_arbitration_isStuck);
  always @ (*) begin
    decode_arbitration_isValid = IBusSimplePlugin_injector_decodeInput_valid;
    case(_zz_110_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b011 : begin
        decode_arbitration_isValid = 1'b1;
      end
      3'b100 : begin
      end
      default : begin
      end
    endcase
  end

  assign iBus_cmd_valid = IBusSimplePlugin_cmd_valid;
  assign IBusSimplePlugin_cmd_ready = iBus_cmd_ready;
  assign iBus_cmd_payload_pc = IBusSimplePlugin_cmd_payload_pc;
  assign IBusSimplePlugin_pending_next = (_zz_159_ - _zz_163_);
  assign IBusSimplePlugin_cmdFork_canEmit = (IBusSimplePlugin_iBusRsp_stages_1_output_ready && (IBusSimplePlugin_pending_value != (3'b111)));
  assign IBusSimplePlugin_cmd_valid = (IBusSimplePlugin_iBusRsp_stages_1_input_valid && IBusSimplePlugin_cmdFork_canEmit);
  assign IBusSimplePlugin_pending_inc = (IBusSimplePlugin_cmd_valid && IBusSimplePlugin_cmd_ready);
  assign IBusSimplePlugin_cmd_payload_pc = {IBusSimplePlugin_iBusRsp_stages_1_input_payload[31 : 2],(2'b00)};
  assign IBusSimplePlugin_rspJoin_rspBuffer_flush = ((IBusSimplePlugin_rspJoin_rspBuffer_discardCounter != (3'b000)) || IBusSimplePlugin_iBusRsp_flush);
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_valid = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter == (3'b000)));
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_error;
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst = IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_payload_inst;
  assign _zz_115_ = (IBusSimplePlugin_rspJoin_rspBuffer_output_ready || IBusSimplePlugin_rspJoin_rspBuffer_flush);
  assign IBusSimplePlugin_pending_dec = (IBusSimplePlugin_rspJoin_rspBuffer_c_io_pop_valid && _zz_115_);
  assign IBusSimplePlugin_rspJoin_fetchRsp_pc = IBusSimplePlugin_iBusRsp_stages_2_output_payload;
  always @ (*) begin
    IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = IBusSimplePlugin_rspJoin_rspBuffer_output_payload_error;
    if((! IBusSimplePlugin_rspJoin_rspBuffer_output_valid))begin
      IBusSimplePlugin_rspJoin_fetchRsp_rsp_error = 1'b0;
    end
  end

  assign IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst = IBusSimplePlugin_rspJoin_rspBuffer_output_payload_inst;
  assign IBusSimplePlugin_rspJoin_exceptionDetected = 1'b0;
  assign IBusSimplePlugin_rspJoin_join_valid = (IBusSimplePlugin_iBusRsp_stages_2_output_valid && IBusSimplePlugin_rspJoin_rspBuffer_output_valid);
  assign IBusSimplePlugin_rspJoin_join_payload_pc = IBusSimplePlugin_rspJoin_fetchRsp_pc;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_error = IBusSimplePlugin_rspJoin_fetchRsp_rsp_error;
  assign IBusSimplePlugin_rspJoin_join_payload_rsp_inst = IBusSimplePlugin_rspJoin_fetchRsp_rsp_inst;
  assign IBusSimplePlugin_rspJoin_join_payload_isRvc = IBusSimplePlugin_rspJoin_fetchRsp_isRvc;
  assign IBusSimplePlugin_iBusRsp_stages_2_output_ready = (IBusSimplePlugin_iBusRsp_stages_2_output_valid ? (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready) : IBusSimplePlugin_rspJoin_join_ready);
  assign IBusSimplePlugin_rspJoin_rspBuffer_output_ready = (IBusSimplePlugin_rspJoin_join_valid && IBusSimplePlugin_rspJoin_join_ready);
  assign _zz_60_ = (! IBusSimplePlugin_rspJoin_exceptionDetected);
  assign IBusSimplePlugin_rspJoin_join_ready = (IBusSimplePlugin_iBusRsp_output_ready && _zz_60_);
  assign IBusSimplePlugin_iBusRsp_output_valid = (IBusSimplePlugin_rspJoin_join_valid && _zz_60_);
  assign IBusSimplePlugin_iBusRsp_output_payload_pc = IBusSimplePlugin_rspJoin_join_payload_pc;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_error = IBusSimplePlugin_rspJoin_join_payload_rsp_error;
  assign IBusSimplePlugin_iBusRsp_output_payload_rsp_inst = IBusSimplePlugin_rspJoin_join_payload_rsp_inst;
  assign IBusSimplePlugin_iBusRsp_output_payload_isRvc = IBusSimplePlugin_rspJoin_join_payload_isRvc;
  assign _zz_61_ = 1'b0;
  always @ (*) begin
    execute_DBusSimplePlugin_skipCmd = 1'b0;
    if(execute_ALIGNEMENT_FAULT)begin
      execute_DBusSimplePlugin_skipCmd = 1'b1;
    end
  end

  assign dBus_cmd_valid = (((((execute_arbitration_isValid && execute_MEMORY_ENABLE) && (! execute_arbitration_isStuckByOthers)) && (! execute_arbitration_isFlushed)) && (! execute_DBusSimplePlugin_skipCmd)) && (! _zz_61_));
  assign dBus_cmd_payload_wr = execute_MEMORY_STORE;
  assign dBus_cmd_payload_size = execute_INSTRUCTION[13 : 12];
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_62_ = {{{execute_RS2[7 : 0],execute_RS2[7 : 0]},execute_RS2[7 : 0]},execute_RS2[7 : 0]};
      end
      2'b01 : begin
        _zz_62_ = {execute_RS2[15 : 0],execute_RS2[15 : 0]};
      end
      default : begin
        _zz_62_ = execute_RS2[31 : 0];
      end
    endcase
  end

  assign dBus_cmd_payload_data = _zz_62_;
  always @ (*) begin
    case(dBus_cmd_payload_size)
      2'b00 : begin
        _zz_63_ = (4'b0001);
      end
      2'b01 : begin
        _zz_63_ = (4'b0011);
      end
      default : begin
        _zz_63_ = (4'b1111);
      end
    endcase
  end

  assign execute_DBusSimplePlugin_formalMask = (_zz_63_ <<< dBus_cmd_payload_address[1 : 0]);
  assign dBus_cmd_payload_address = execute_SRC_ADD;
  always @ (*) begin
    writeBack_DBusSimplePlugin_rspShifted = writeBack_MEMORY_READ_DATA;
    case(writeBack_MEMORY_ADDRESS_LOW)
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[15 : 8];
      end
      2'b10 : begin
        writeBack_DBusSimplePlugin_rspShifted[15 : 0] = writeBack_MEMORY_READ_DATA[31 : 16];
      end
      2'b11 : begin
        writeBack_DBusSimplePlugin_rspShifted[7 : 0] = writeBack_MEMORY_READ_DATA[31 : 24];
      end
      default : begin
      end
    endcase
  end

  assign _zz_64_ = (writeBack_DBusSimplePlugin_rspShifted[7] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_65_[31] = _zz_64_;
    _zz_65_[30] = _zz_64_;
    _zz_65_[29] = _zz_64_;
    _zz_65_[28] = _zz_64_;
    _zz_65_[27] = _zz_64_;
    _zz_65_[26] = _zz_64_;
    _zz_65_[25] = _zz_64_;
    _zz_65_[24] = _zz_64_;
    _zz_65_[23] = _zz_64_;
    _zz_65_[22] = _zz_64_;
    _zz_65_[21] = _zz_64_;
    _zz_65_[20] = _zz_64_;
    _zz_65_[19] = _zz_64_;
    _zz_65_[18] = _zz_64_;
    _zz_65_[17] = _zz_64_;
    _zz_65_[16] = _zz_64_;
    _zz_65_[15] = _zz_64_;
    _zz_65_[14] = _zz_64_;
    _zz_65_[13] = _zz_64_;
    _zz_65_[12] = _zz_64_;
    _zz_65_[11] = _zz_64_;
    _zz_65_[10] = _zz_64_;
    _zz_65_[9] = _zz_64_;
    _zz_65_[8] = _zz_64_;
    _zz_65_[7 : 0] = writeBack_DBusSimplePlugin_rspShifted[7 : 0];
  end

  assign _zz_66_ = (writeBack_DBusSimplePlugin_rspShifted[15] && (! writeBack_INSTRUCTION[14]));
  always @ (*) begin
    _zz_67_[31] = _zz_66_;
    _zz_67_[30] = _zz_66_;
    _zz_67_[29] = _zz_66_;
    _zz_67_[28] = _zz_66_;
    _zz_67_[27] = _zz_66_;
    _zz_67_[26] = _zz_66_;
    _zz_67_[25] = _zz_66_;
    _zz_67_[24] = _zz_66_;
    _zz_67_[23] = _zz_66_;
    _zz_67_[22] = _zz_66_;
    _zz_67_[21] = _zz_66_;
    _zz_67_[20] = _zz_66_;
    _zz_67_[19] = _zz_66_;
    _zz_67_[18] = _zz_66_;
    _zz_67_[17] = _zz_66_;
    _zz_67_[16] = _zz_66_;
    _zz_67_[15 : 0] = writeBack_DBusSimplePlugin_rspShifted[15 : 0];
  end

  always @ (*) begin
    case(_zz_139_)
      2'b00 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_65_;
      end
      2'b01 : begin
        writeBack_DBusSimplePlugin_rspFormated = _zz_67_;
      end
      default : begin
        writeBack_DBusSimplePlugin_rspFormated = writeBack_DBusSimplePlugin_rspShifted;
      end
    endcase
  end

  always @ (*) begin
    CsrPlugin_privilege = (2'b11);
    if(CsrPlugin_forceMachineWire)begin
      CsrPlugin_privilege = (2'b11);
    end
  end

  assign CsrPlugin_misa_base = (2'b01);
  assign CsrPlugin_misa_extensions = 26'h0000042;
  assign CsrPlugin_mtvec_mode = (2'b00);
  assign CsrPlugin_mtvec_base = 30'h20000008;
  assign _zz_68_ = (CsrPlugin_mip_MTIP && CsrPlugin_mie_MTIE);
  assign _zz_69_ = (CsrPlugin_mip_MSIP && CsrPlugin_mie_MSIE);
  assign _zz_70_ = (CsrPlugin_mip_MEIP && CsrPlugin_mie_MEIE);
  assign CsrPlugin_exception = 1'b0;
  assign CsrPlugin_lastStageWasWfi = 1'b0;
  assign CsrPlugin_pipelineLiberator_active = ((CsrPlugin_interrupt_valid && CsrPlugin_allowInterrupts) && decode_arbitration_isValid);
  always @ (*) begin
    CsrPlugin_pipelineLiberator_done = CsrPlugin_pipelineLiberator_pcValids_2;
    if(CsrPlugin_hadException)begin
      CsrPlugin_pipelineLiberator_done = 1'b0;
    end
  end

  assign CsrPlugin_interruptJump = ((CsrPlugin_interrupt_valid && CsrPlugin_pipelineLiberator_done) && CsrPlugin_allowInterrupts);
  assign CsrPlugin_targetPrivilege = CsrPlugin_interrupt_targetPrivilege;
  assign CsrPlugin_trapCause = CsrPlugin_interrupt_code;
  always @ (*) begin
    CsrPlugin_xtvec_mode = (2'bxx);
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_mode = CsrPlugin_mtvec_mode;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    CsrPlugin_xtvec_base = 30'h0;
    case(CsrPlugin_targetPrivilege)
      2'b11 : begin
        CsrPlugin_xtvec_base = CsrPlugin_mtvec_base;
      end
      default : begin
      end
    endcase
  end

  assign contextSwitching = CsrPlugin_jumpInterface_valid;
  assign execute_CsrPlugin_blockedBySideEffects = ({writeBack_arbitration_isValid,memory_arbitration_isValid} != (2'b00));
  always @ (*) begin
    execute_CsrPlugin_illegalAccess = 1'b1;
    if(execute_CsrPlugin_csr_768)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_836)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_772)begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
    if(execute_CsrPlugin_csr_834)begin
      if(execute_CSR_READ_OPCODE)begin
        execute_CsrPlugin_illegalAccess = 1'b0;
      end
    end
    if((CsrPlugin_privilege < execute_CsrPlugin_csrAddress[9 : 8]))begin
      execute_CsrPlugin_illegalAccess = 1'b1;
    end
    if(((! execute_arbitration_isValid) || (! execute_IS_CSR)))begin
      execute_CsrPlugin_illegalAccess = 1'b0;
    end
  end

  always @ (*) begin
    execute_CsrPlugin_illegalInstruction = 1'b0;
    if((execute_arbitration_isValid && (execute_ENV_CTRL == `EnvCtrlEnum_defaultEncoding_XRET)))begin
      if((CsrPlugin_privilege < execute_INSTRUCTION[29 : 28]))begin
        execute_CsrPlugin_illegalInstruction = 1'b1;
      end
    end
  end

  assign execute_CsrPlugin_writeInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_WRITE_OPCODE);
  assign execute_CsrPlugin_readInstruction = ((execute_arbitration_isValid && execute_IS_CSR) && execute_CSR_READ_OPCODE);
  assign execute_CsrPlugin_writeEnable = ((execute_CsrPlugin_writeInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readEnable = ((execute_CsrPlugin_readInstruction && (! execute_CsrPlugin_blockedBySideEffects)) && (! execute_arbitration_isStuckByOthers));
  assign execute_CsrPlugin_readToWriteData = execute_CsrPlugin_readData;
  always @ (*) begin
    case(_zz_140_)
      1'b0 : begin
        execute_CsrPlugin_writeData = execute_SRC1;
      end
      default : begin
        execute_CsrPlugin_writeData = (execute_INSTRUCTION[12] ? (execute_CsrPlugin_readToWriteData & (~ execute_SRC1)) : (execute_CsrPlugin_readToWriteData | execute_SRC1));
      end
    endcase
  end

  assign execute_CsrPlugin_csrAddress = execute_INSTRUCTION[31 : 20];
  assign _zz_72_ = ((decode_INSTRUCTION & 32'h0000000c) == 32'h00000008);
  assign _zz_73_ = ((decode_INSTRUCTION & 32'h00004050) == 32'h00004050);
  assign _zz_74_ = ((decode_INSTRUCTION & 32'h00006004) == 32'h00002000);
  assign _zz_75_ = ((decode_INSTRUCTION & 32'h00000050) == 32'h00000010);
  assign _zz_76_ = ((decode_INSTRUCTION & 32'h00000008) == 32'h00000008);
  assign _zz_77_ = ((decode_INSTRUCTION & 32'h00000048) == 32'h00000048);
  assign _zz_78_ = ((decode_INSTRUCTION & 32'h00000004) == 32'h00000004);
  assign _zz_71_ = {(((decode_INSTRUCTION & _zz_191_) == 32'h00001000) != (1'b0)),{((_zz_192_ == _zz_193_) != (1'b0)),{({_zz_194_,_zz_195_} != (2'b00)),{(_zz_196_ != _zz_197_),{_zz_198_,{_zz_199_,_zz_200_}}}}}};
  assign _zz_79_ = _zz_71_[3 : 2];
  assign _zz_38_ = _zz_79_;
  assign _zz_80_ = _zz_71_[7 : 6];
  assign _zz_37_ = _zz_80_;
  assign _zz_81_ = _zz_71_[13 : 12];
  assign _zz_36_ = _zz_81_;
  assign _zz_82_ = _zz_71_[15 : 14];
  assign _zz_35_ = _zz_82_;
  assign _zz_83_ = _zz_71_[16 : 16];
  assign _zz_34_ = _zz_83_;
  assign _zz_84_ = _zz_71_[24 : 23];
  assign _zz_33_ = _zz_84_;
  assign _zz_85_ = _zz_71_[26 : 25];
  assign _zz_32_ = _zz_85_;
  assign decode_RegFilePlugin_regFileReadAddress1 = decode_INSTRUCTION_ANTICIPATED[19 : 15];
  assign decode_RegFilePlugin_regFileReadAddress2 = decode_INSTRUCTION_ANTICIPATED[24 : 20];
  assign decode_RegFilePlugin_rs1Data = _zz_117_;
  assign decode_RegFilePlugin_rs2Data = _zz_118_;
  always @ (*) begin
    lastStageRegFileWrite_valid = (_zz_30_ && writeBack_arbitration_isFiring);
    if(_zz_86_)begin
      lastStageRegFileWrite_valid = 1'b1;
    end
  end

  assign lastStageRegFileWrite_payload_address = _zz_29_[11 : 7];
  assign lastStageRegFileWrite_payload_data = _zz_42_;
  always @ (*) begin
    case(execute_ALU_BITWISE_CTRL)
      `AluBitwiseCtrlEnum_defaultEncoding_AND_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 & execute_SRC2);
      end
      `AluBitwiseCtrlEnum_defaultEncoding_OR_1 : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 | execute_SRC2);
      end
      default : begin
        execute_IntAluPlugin_bitwise = (execute_SRC1 ^ execute_SRC2);
      end
    endcase
  end

  always @ (*) begin
    case(execute_ALU_CTRL)
      `AluCtrlEnum_defaultEncoding_BITWISE : begin
        _zz_87_ = execute_IntAluPlugin_bitwise;
      end
      `AluCtrlEnum_defaultEncoding_SLT_SLTU : begin
        _zz_87_ = {31'd0, _zz_166_};
      end
      default : begin
        _zz_87_ = execute_SRC_ADD_SUB;
      end
    endcase
  end

  always @ (*) begin
    case(decode_SRC1_CTRL)
      `Src1CtrlEnum_defaultEncoding_RS : begin
        _zz_88_ = _zz_25_;
      end
      `Src1CtrlEnum_defaultEncoding_PC_INCREMENT : begin
        _zz_88_ = {29'd0, _zz_167_};
      end
      `Src1CtrlEnum_defaultEncoding_IMU : begin
        _zz_88_ = {decode_INSTRUCTION[31 : 12],12'h0};
      end
      default : begin
        _zz_88_ = {27'd0, _zz_168_};
      end
    endcase
  end

  assign _zz_89_ = _zz_169_[11];
  always @ (*) begin
    _zz_90_[19] = _zz_89_;
    _zz_90_[18] = _zz_89_;
    _zz_90_[17] = _zz_89_;
    _zz_90_[16] = _zz_89_;
    _zz_90_[15] = _zz_89_;
    _zz_90_[14] = _zz_89_;
    _zz_90_[13] = _zz_89_;
    _zz_90_[12] = _zz_89_;
    _zz_90_[11] = _zz_89_;
    _zz_90_[10] = _zz_89_;
    _zz_90_[9] = _zz_89_;
    _zz_90_[8] = _zz_89_;
    _zz_90_[7] = _zz_89_;
    _zz_90_[6] = _zz_89_;
    _zz_90_[5] = _zz_89_;
    _zz_90_[4] = _zz_89_;
    _zz_90_[3] = _zz_89_;
    _zz_90_[2] = _zz_89_;
    _zz_90_[1] = _zz_89_;
    _zz_90_[0] = _zz_89_;
  end

  assign _zz_91_ = _zz_170_[11];
  always @ (*) begin
    _zz_92_[19] = _zz_91_;
    _zz_92_[18] = _zz_91_;
    _zz_92_[17] = _zz_91_;
    _zz_92_[16] = _zz_91_;
    _zz_92_[15] = _zz_91_;
    _zz_92_[14] = _zz_91_;
    _zz_92_[13] = _zz_91_;
    _zz_92_[12] = _zz_91_;
    _zz_92_[11] = _zz_91_;
    _zz_92_[10] = _zz_91_;
    _zz_92_[9] = _zz_91_;
    _zz_92_[8] = _zz_91_;
    _zz_92_[7] = _zz_91_;
    _zz_92_[6] = _zz_91_;
    _zz_92_[5] = _zz_91_;
    _zz_92_[4] = _zz_91_;
    _zz_92_[3] = _zz_91_;
    _zz_92_[2] = _zz_91_;
    _zz_92_[1] = _zz_91_;
    _zz_92_[0] = _zz_91_;
  end

  always @ (*) begin
    case(decode_SRC2_CTRL)
      `Src2CtrlEnum_defaultEncoding_RS : begin
        _zz_93_ = _zz_23_;
      end
      `Src2CtrlEnum_defaultEncoding_IMI : begin
        _zz_93_ = {_zz_90_,decode_INSTRUCTION[31 : 20]};
      end
      `Src2CtrlEnum_defaultEncoding_IMS : begin
        _zz_93_ = {_zz_92_,{decode_INSTRUCTION[31 : 25],decode_INSTRUCTION[11 : 7]}};
      end
      default : begin
        _zz_93_ = _zz_22_;
      end
    endcase
  end

  always @ (*) begin
    execute_SrcPlugin_addSub = _zz_171_;
    if(execute_SRC2_FORCE_ZERO)begin
      execute_SrcPlugin_addSub = execute_SRC1;
    end
  end

  assign execute_SrcPlugin_less = ((execute_SRC1[31] == execute_SRC2[31]) ? execute_SrcPlugin_addSub[31] : (execute_SRC_LESS_UNSIGNED ? execute_SRC2[31] : execute_SRC1[31]));
  assign execute_LightShifterPlugin_isShift = (execute_SHIFT_CTRL != `ShiftCtrlEnum_defaultEncoding_DISABLE_1);
  assign execute_LightShifterPlugin_amplitude = (execute_LightShifterPlugin_isActive ? execute_LightShifterPlugin_amplitudeReg : execute_SRC2[4 : 0]);
  assign execute_LightShifterPlugin_shiftInput = (execute_LightShifterPlugin_isActive ? memory_REGFILE_WRITE_DATA : execute_SRC1);
  assign execute_LightShifterPlugin_done = (execute_LightShifterPlugin_amplitude[4 : 1] == (4'b0000));
  always @ (*) begin
    case(execute_SHIFT_CTRL)
      `ShiftCtrlEnum_defaultEncoding_SLL_1 : begin
        _zz_94_ = (execute_LightShifterPlugin_shiftInput <<< 1);
      end
      default : begin
        _zz_94_ = _zz_178_;
      end
    endcase
  end

  always @ (*) begin
    _zz_95_ = 1'b0;
    if(_zz_97_)begin
      if((_zz_98_ == decode_INSTRUCTION[19 : 15]))begin
        _zz_95_ = 1'b1;
      end
    end
    if(_zz_128_)begin
      if(_zz_129_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_95_ = 1'b1;
        end
      end
    end
    if(_zz_130_)begin
      if(_zz_131_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_95_ = 1'b1;
        end
      end
    end
    if(_zz_132_)begin
      if(_zz_133_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[19 : 15]))begin
          _zz_95_ = 1'b1;
        end
      end
    end
    if((! decode_RS1_USE))begin
      _zz_95_ = 1'b0;
    end
  end

  always @ (*) begin
    _zz_96_ = 1'b0;
    if(_zz_97_)begin
      if((_zz_98_ == decode_INSTRUCTION[24 : 20]))begin
        _zz_96_ = 1'b1;
      end
    end
    if(_zz_128_)begin
      if(_zz_129_)begin
        if((writeBack_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_96_ = 1'b1;
        end
      end
    end
    if(_zz_130_)begin
      if(_zz_131_)begin
        if((memory_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_96_ = 1'b1;
        end
      end
    end
    if(_zz_132_)begin
      if(_zz_133_)begin
        if((execute_INSTRUCTION[11 : 7] == decode_INSTRUCTION[24 : 20]))begin
          _zz_96_ = 1'b1;
        end
      end
    end
    if((! decode_RS2_USE))begin
      _zz_96_ = 1'b0;
    end
  end

  assign execute_BranchPlugin_eq = (execute_SRC1 == execute_SRC2);
  assign _zz_99_ = execute_INSTRUCTION[14 : 12];
  always @ (*) begin
    if((_zz_99_ == (3'b000))) begin
        _zz_100_ = execute_BranchPlugin_eq;
    end else if((_zz_99_ == (3'b001))) begin
        _zz_100_ = (! execute_BranchPlugin_eq);
    end else if((((_zz_99_ & (3'b101)) == (3'b101)))) begin
        _zz_100_ = (! execute_SRC_LESS);
    end else begin
        _zz_100_ = execute_SRC_LESS;
    end
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_INC : begin
        _zz_101_ = 1'b0;
      end
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_101_ = 1'b1;
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_101_ = 1'b1;
      end
      default : begin
        _zz_101_ = _zz_100_;
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src1 = ((execute_BRANCH_CTRL == `BranchCtrlEnum_defaultEncoding_JALR) ? execute_RS1 : execute_PC);
  assign _zz_102_ = _zz_180_[19];
  always @ (*) begin
    _zz_103_[10] = _zz_102_;
    _zz_103_[9] = _zz_102_;
    _zz_103_[8] = _zz_102_;
    _zz_103_[7] = _zz_102_;
    _zz_103_[6] = _zz_102_;
    _zz_103_[5] = _zz_102_;
    _zz_103_[4] = _zz_102_;
    _zz_103_[3] = _zz_102_;
    _zz_103_[2] = _zz_102_;
    _zz_103_[1] = _zz_102_;
    _zz_103_[0] = _zz_102_;
  end

  assign _zz_104_ = _zz_181_[11];
  always @ (*) begin
    _zz_105_[19] = _zz_104_;
    _zz_105_[18] = _zz_104_;
    _zz_105_[17] = _zz_104_;
    _zz_105_[16] = _zz_104_;
    _zz_105_[15] = _zz_104_;
    _zz_105_[14] = _zz_104_;
    _zz_105_[13] = _zz_104_;
    _zz_105_[12] = _zz_104_;
    _zz_105_[11] = _zz_104_;
    _zz_105_[10] = _zz_104_;
    _zz_105_[9] = _zz_104_;
    _zz_105_[8] = _zz_104_;
    _zz_105_[7] = _zz_104_;
    _zz_105_[6] = _zz_104_;
    _zz_105_[5] = _zz_104_;
    _zz_105_[4] = _zz_104_;
    _zz_105_[3] = _zz_104_;
    _zz_105_[2] = _zz_104_;
    _zz_105_[1] = _zz_104_;
    _zz_105_[0] = _zz_104_;
  end

  assign _zz_106_ = _zz_182_[11];
  always @ (*) begin
    _zz_107_[18] = _zz_106_;
    _zz_107_[17] = _zz_106_;
    _zz_107_[16] = _zz_106_;
    _zz_107_[15] = _zz_106_;
    _zz_107_[14] = _zz_106_;
    _zz_107_[13] = _zz_106_;
    _zz_107_[12] = _zz_106_;
    _zz_107_[11] = _zz_106_;
    _zz_107_[10] = _zz_106_;
    _zz_107_[9] = _zz_106_;
    _zz_107_[8] = _zz_106_;
    _zz_107_[7] = _zz_106_;
    _zz_107_[6] = _zz_106_;
    _zz_107_[5] = _zz_106_;
    _zz_107_[4] = _zz_106_;
    _zz_107_[3] = _zz_106_;
    _zz_107_[2] = _zz_106_;
    _zz_107_[1] = _zz_106_;
    _zz_107_[0] = _zz_106_;
  end

  always @ (*) begin
    case(execute_BRANCH_CTRL)
      `BranchCtrlEnum_defaultEncoding_JAL : begin
        _zz_108_ = {{_zz_103_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[19 : 12]},execute_INSTRUCTION[20]},execute_INSTRUCTION[30 : 21]}},1'b0};
      end
      `BranchCtrlEnum_defaultEncoding_JALR : begin
        _zz_108_ = {_zz_105_,execute_INSTRUCTION[31 : 20]};
      end
      default : begin
        _zz_108_ = {{_zz_107_,{{{execute_INSTRUCTION[31],execute_INSTRUCTION[7]},execute_INSTRUCTION[30 : 25]},execute_INSTRUCTION[11 : 8]}},1'b0};
      end
    endcase
  end

  assign execute_BranchPlugin_branch_src2 = _zz_108_;
  assign execute_BranchPlugin_branchAdder = (execute_BranchPlugin_branch_src1 + execute_BranchPlugin_branch_src2);
  assign BranchPlugin_jumpInterface_valid = ((memory_arbitration_isValid && memory_BRANCH_DO) && (! 1'b0));
  assign BranchPlugin_jumpInterface_payload = memory_BRANCH_CALC;
  always @ (*) begin
    debug_bus_cmd_ready = 1'b1;
    if(debug_bus_cmd_valid)begin
      case(_zz_134_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            debug_bus_cmd_ready = IBusSimplePlugin_injectionPort_ready;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (*) begin
    debug_bus_rsp_data = DebugPlugin_busReadDataReg;
    if((! _zz_109_))begin
      debug_bus_rsp_data[0] = DebugPlugin_resetIt;
      debug_bus_rsp_data[1] = DebugPlugin_haltIt;
      debug_bus_rsp_data[2] = DebugPlugin_isPipBusy;
      debug_bus_rsp_data[3] = DebugPlugin_haltedByBreak;
      debug_bus_rsp_data[4] = DebugPlugin_stepIt;
    end
  end

  always @ (*) begin
    IBusSimplePlugin_injectionPort_valid = 1'b0;
    if(debug_bus_cmd_valid)begin
      case(_zz_134_)
        6'b000000 : begin
        end
        6'b000001 : begin
          if(debug_bus_cmd_payload_wr)begin
            IBusSimplePlugin_injectionPort_valid = 1'b1;
          end
        end
        default : begin
        end
      endcase
    end
  end

  assign IBusSimplePlugin_injectionPort_payload = debug_bus_cmd_payload_data;
  assign debug_resetOut = DebugPlugin_resetIt_regNext;
  assign _zz_19_ = decode_ENV_CTRL;
  assign _zz_16_ = execute_ENV_CTRL;
  assign _zz_14_ = memory_ENV_CTRL;
  assign _zz_17_ = _zz_34_;
  assign _zz_40_ = decode_to_execute_ENV_CTRL;
  assign _zz_39_ = execute_to_memory_ENV_CTRL;
  assign _zz_41_ = memory_to_writeBack_ENV_CTRL;
  assign _zz_26_ = _zz_38_;
  assign _zz_24_ = _zz_33_;
  assign _zz_12_ = decode_ALU_BITWISE_CTRL;
  assign _zz_10_ = _zz_32_;
  assign _zz_28_ = decode_to_execute_ALU_BITWISE_CTRL;
  assign _zz_9_ = decode_BRANCH_CTRL;
  assign _zz_7_ = _zz_35_;
  assign _zz_20_ = decode_to_execute_BRANCH_CTRL;
  assign _zz_6_ = decode_ALU_CTRL;
  assign _zz_4_ = _zz_36_;
  assign _zz_27_ = decode_to_execute_ALU_CTRL;
  assign _zz_3_ = decode_SHIFT_CTRL;
  assign _zz_1_ = _zz_37_;
  assign _zz_21_ = decode_to_execute_SHIFT_CTRL;
  assign decode_arbitration_isFlushed = (({writeBack_arbitration_flushNext,{memory_arbitration_flushNext,execute_arbitration_flushNext}} != (3'b000)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,{execute_arbitration_flushIt,decode_arbitration_flushIt}}} != (4'b0000)));
  assign execute_arbitration_isFlushed = (({writeBack_arbitration_flushNext,memory_arbitration_flushNext} != (2'b00)) || ({writeBack_arbitration_flushIt,{memory_arbitration_flushIt,execute_arbitration_flushIt}} != (3'b000)));
  assign memory_arbitration_isFlushed = ((writeBack_arbitration_flushNext != (1'b0)) || ({writeBack_arbitration_flushIt,memory_arbitration_flushIt} != (2'b00)));
  assign writeBack_arbitration_isFlushed = (1'b0 || (writeBack_arbitration_flushIt != (1'b0)));
  assign decode_arbitration_isStuckByOthers = (decode_arbitration_haltByOther || (((1'b0 || execute_arbitration_isStuck) || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign decode_arbitration_isStuck = (decode_arbitration_haltItself || decode_arbitration_isStuckByOthers);
  assign decode_arbitration_isMoving = ((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt));
  assign decode_arbitration_isFiring = ((decode_arbitration_isValid && (! decode_arbitration_isStuck)) && (! decode_arbitration_removeIt));
  assign execute_arbitration_isStuckByOthers = (execute_arbitration_haltByOther || ((1'b0 || memory_arbitration_isStuck) || writeBack_arbitration_isStuck));
  assign execute_arbitration_isStuck = (execute_arbitration_haltItself || execute_arbitration_isStuckByOthers);
  assign execute_arbitration_isMoving = ((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt));
  assign execute_arbitration_isFiring = ((execute_arbitration_isValid && (! execute_arbitration_isStuck)) && (! execute_arbitration_removeIt));
  assign memory_arbitration_isStuckByOthers = (memory_arbitration_haltByOther || (1'b0 || writeBack_arbitration_isStuck));
  assign memory_arbitration_isStuck = (memory_arbitration_haltItself || memory_arbitration_isStuckByOthers);
  assign memory_arbitration_isMoving = ((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt));
  assign memory_arbitration_isFiring = ((memory_arbitration_isValid && (! memory_arbitration_isStuck)) && (! memory_arbitration_removeIt));
  assign writeBack_arbitration_isStuckByOthers = (writeBack_arbitration_haltByOther || 1'b0);
  assign writeBack_arbitration_isStuck = (writeBack_arbitration_haltItself || writeBack_arbitration_isStuckByOthers);
  assign writeBack_arbitration_isMoving = ((! writeBack_arbitration_isStuck) && (! writeBack_arbitration_removeIt));
  assign writeBack_arbitration_isFiring = ((writeBack_arbitration_isValid && (! writeBack_arbitration_isStuck)) && (! writeBack_arbitration_removeIt));
  always @ (*) begin
    IBusSimplePlugin_injectionPort_ready = 1'b0;
    case(_zz_110_)
      3'b000 : begin
      end
      3'b001 : begin
      end
      3'b010 : begin
      end
      3'b011 : begin
      end
      3'b100 : begin
        IBusSimplePlugin_injectionPort_ready = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_111_ = 32'h0;
    if(execute_CsrPlugin_csr_768)begin
      _zz_111_[12 : 11] = CsrPlugin_mstatus_MPP;
      _zz_111_[7 : 7] = CsrPlugin_mstatus_MPIE;
      _zz_111_[3 : 3] = CsrPlugin_mstatus_MIE;
    end
  end

  always @ (*) begin
    _zz_112_ = 32'h0;
    if(execute_CsrPlugin_csr_836)begin
      _zz_112_[11 : 11] = CsrPlugin_mip_MEIP;
      _zz_112_[7 : 7] = CsrPlugin_mip_MTIP;
      _zz_112_[3 : 3] = CsrPlugin_mip_MSIP;
    end
  end

  always @ (*) begin
    _zz_113_ = 32'h0;
    if(execute_CsrPlugin_csr_772)begin
      _zz_113_[11 : 11] = CsrPlugin_mie_MEIE;
      _zz_113_[7 : 7] = CsrPlugin_mie_MTIE;
      _zz_113_[3 : 3] = CsrPlugin_mie_MSIE;
    end
  end

  always @ (*) begin
    _zz_114_ = 32'h0;
    if(execute_CsrPlugin_csr_834)begin
      _zz_114_[31 : 31] = CsrPlugin_mcause_interrupt;
      _zz_114_[3 : 0] = CsrPlugin_mcause_exceptionCode;
    end
  end

  assign execute_CsrPlugin_readData = ((_zz_111_ | _zz_112_) | (_zz_113_ | _zz_114_));
  assign _zz_116_ = 1'b0;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      IBusSimplePlugin_fetchPc_pcReg <= 32'h80000000;
      IBusSimplePlugin_fetchPc_correctionReg <= 1'b0;
      IBusSimplePlugin_fetchPc_booted <= 1'b0;
      IBusSimplePlugin_fetchPc_inc <= 1'b0;
      _zz_51_ <= 1'b0;
      _zz_53_ <= 1'b0;
      _zz_55_ <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      IBusSimplePlugin_pending_value <= (3'b000);
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (3'b000);
      CsrPlugin_mstatus_MIE <= 1'b0;
      CsrPlugin_mstatus_MPIE <= 1'b0;
      CsrPlugin_mstatus_MPP <= (2'b11);
      CsrPlugin_mie_MEIE <= 1'b0;
      CsrPlugin_mie_MTIE <= 1'b0;
      CsrPlugin_mie_MSIE <= 1'b0;
      CsrPlugin_interrupt_valid <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_1 <= 1'b0;
      CsrPlugin_pipelineLiberator_pcValids_2 <= 1'b0;
      CsrPlugin_hadException <= 1'b0;
      execute_CsrPlugin_wfiWake <= 1'b0;
      _zz_86_ <= 1'b1;
      execute_LightShifterPlugin_isActive <= 1'b0;
      _zz_97_ <= 1'b0;
      execute_arbitration_isValid <= 1'b0;
      memory_arbitration_isValid <= 1'b0;
      writeBack_arbitration_isValid <= 1'b0;
      _zz_110_ <= (3'b000);
      memory_to_writeBack_REGFILE_WRITE_DATA <= 32'h0;
      memory_to_writeBack_INSTRUCTION <= 32'h0;
    end else begin
      if(IBusSimplePlugin_fetchPc_correction)begin
        IBusSimplePlugin_fetchPc_correctionReg <= 1'b1;
      end
      if((IBusSimplePlugin_fetchPc_output_valid && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_correctionReg <= 1'b0;
      end
      IBusSimplePlugin_fetchPc_booted <= 1'b1;
      if((IBusSimplePlugin_fetchPc_correction || IBusSimplePlugin_fetchPc_pcRegPropagate))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_output_valid && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b1;
      end
      if(((! IBusSimplePlugin_fetchPc_output_valid) && IBusSimplePlugin_fetchPc_output_ready))begin
        IBusSimplePlugin_fetchPc_inc <= 1'b0;
      end
      if((IBusSimplePlugin_fetchPc_booted && ((IBusSimplePlugin_fetchPc_output_ready || IBusSimplePlugin_fetchPc_correction) || IBusSimplePlugin_fetchPc_pcRegPropagate)))begin
        IBusSimplePlugin_fetchPc_pcReg <= IBusSimplePlugin_fetchPc_pc;
      end
      if(IBusSimplePlugin_iBusRsp_flush)begin
        _zz_51_ <= 1'b0;
      end
      if(_zz_49_)begin
        _zz_51_ <= (IBusSimplePlugin_iBusRsp_stages_0_output_valid && (! 1'b0));
      end
      if(IBusSimplePlugin_iBusRsp_flush)begin
        _zz_53_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
        _zz_53_ <= (IBusSimplePlugin_iBusRsp_stages_1_output_valid && (! IBusSimplePlugin_iBusRsp_flush));
      end
      if(decode_arbitration_removeIt)begin
        _zz_55_ <= 1'b0;
      end
      if(IBusSimplePlugin_iBusRsp_output_ready)begin
        _zz_55_ <= (IBusSimplePlugin_iBusRsp_output_valid && (! IBusSimplePlugin_externalFlush));
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_1_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_0 <= 1'b1;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_iBusRsp_stages_2_input_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= IBusSimplePlugin_injector_nextPcCalc_valids_0;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_1 <= 1'b0;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if((! (! IBusSimplePlugin_injector_decodeInput_ready)))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= IBusSimplePlugin_injector_nextPcCalc_valids_1;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_2 <= 1'b0;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if((! execute_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= IBusSimplePlugin_injector_nextPcCalc_valids_2;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_3 <= 1'b0;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if((! memory_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= IBusSimplePlugin_injector_nextPcCalc_valids_3;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_4 <= 1'b0;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      end
      if((! writeBack_arbitration_isStuck))begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= IBusSimplePlugin_injector_nextPcCalc_valids_4;
      end
      if(IBusSimplePlugin_fetchPc_flushed)begin
        IBusSimplePlugin_injector_nextPcCalc_valids_5 <= 1'b0;
      end
      IBusSimplePlugin_pending_value <= IBusSimplePlugin_pending_next;
      IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= (IBusSimplePlugin_rspJoin_rspBuffer_discardCounter - _zz_165_);
      if(IBusSimplePlugin_iBusRsp_flush)begin
        IBusSimplePlugin_rspJoin_rspBuffer_discardCounter <= IBusSimplePlugin_pending_next;
      end
      CsrPlugin_interrupt_valid <= 1'b0;
      if(_zz_135_)begin
        if(_zz_136_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_137_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
        if(_zz_138_)begin
          CsrPlugin_interrupt_valid <= 1'b1;
        end
      end
      if(CsrPlugin_pipelineLiberator_active)begin
        if((! execute_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b1;
        end
        if((! memory_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_1 <= CsrPlugin_pipelineLiberator_pcValids_0;
        end
        if((! writeBack_arbitration_isStuck))begin
          CsrPlugin_pipelineLiberator_pcValids_2 <= CsrPlugin_pipelineLiberator_pcValids_1;
        end
      end
      if(((! CsrPlugin_pipelineLiberator_active) || decode_arbitration_removeIt))begin
        CsrPlugin_pipelineLiberator_pcValids_0 <= 1'b0;
        CsrPlugin_pipelineLiberator_pcValids_1 <= 1'b0;
        CsrPlugin_pipelineLiberator_pcValids_2 <= 1'b0;
      end
      if(CsrPlugin_interruptJump)begin
        CsrPlugin_interrupt_valid <= 1'b0;
      end
      CsrPlugin_hadException <= CsrPlugin_exception;
      if(_zz_124_)begin
        case(CsrPlugin_targetPrivilege)
          2'b11 : begin
            CsrPlugin_mstatus_MIE <= 1'b0;
            CsrPlugin_mstatus_MPIE <= CsrPlugin_mstatus_MIE;
            CsrPlugin_mstatus_MPP <= CsrPlugin_privilege;
          end
          default : begin
          end
        endcase
      end
      if(_zz_125_)begin
        case(_zz_127_)
          2'b11 : begin
            CsrPlugin_mstatus_MPP <= (2'b00);
            CsrPlugin_mstatus_MIE <= CsrPlugin_mstatus_MPIE;
            CsrPlugin_mstatus_MPIE <= 1'b1;
          end
          default : begin
          end
        endcase
      end
      execute_CsrPlugin_wfiWake <= (({_zz_70_,{_zz_69_,_zz_68_}} != (3'b000)) || CsrPlugin_thirdPartyWake);
      _zz_86_ <= 1'b0;
      if(_zz_120_)begin
        if(_zz_121_)begin
          execute_LightShifterPlugin_isActive <= 1'b1;
          if(execute_LightShifterPlugin_done)begin
            execute_LightShifterPlugin_isActive <= 1'b0;
          end
        end
      end
      if(execute_arbitration_removeIt)begin
        execute_LightShifterPlugin_isActive <= 1'b0;
      end
      _zz_97_ <= (_zz_30_ && writeBack_arbitration_isFiring);
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_REGFILE_WRITE_DATA <= memory_REGFILE_WRITE_DATA;
      end
      if((! writeBack_arbitration_isStuck))begin
        memory_to_writeBack_INSTRUCTION <= memory_INSTRUCTION;
      end
      if(((! execute_arbitration_isStuck) || execute_arbitration_removeIt))begin
        execute_arbitration_isValid <= 1'b0;
      end
      if(((! decode_arbitration_isStuck) && (! decode_arbitration_removeIt)))begin
        execute_arbitration_isValid <= decode_arbitration_isValid;
      end
      if(((! memory_arbitration_isStuck) || memory_arbitration_removeIt))begin
        memory_arbitration_isValid <= 1'b0;
      end
      if(((! execute_arbitration_isStuck) && (! execute_arbitration_removeIt)))begin
        memory_arbitration_isValid <= execute_arbitration_isValid;
      end
      if(((! writeBack_arbitration_isStuck) || writeBack_arbitration_removeIt))begin
        writeBack_arbitration_isValid <= 1'b0;
      end
      if(((! memory_arbitration_isStuck) && (! memory_arbitration_removeIt)))begin
        writeBack_arbitration_isValid <= memory_arbitration_isValid;
      end
      case(_zz_110_)
        3'b000 : begin
          if(IBusSimplePlugin_injectionPort_valid)begin
            _zz_110_ <= (3'b001);
          end
        end
        3'b001 : begin
          _zz_110_ <= (3'b010);
        end
        3'b010 : begin
          _zz_110_ <= (3'b011);
        end
        3'b011 : begin
          if((! decode_arbitration_isStuck))begin
            _zz_110_ <= (3'b100);
          end
        end
        3'b100 : begin
          _zz_110_ <= (3'b000);
        end
        default : begin
        end
      endcase
      if(execute_CsrPlugin_csr_768)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mstatus_MPP <= execute_CsrPlugin_writeData[12 : 11];
          CsrPlugin_mstatus_MPIE <= _zz_183_[0];
          CsrPlugin_mstatus_MIE <= _zz_184_[0];
        end
      end
      if(execute_CsrPlugin_csr_772)begin
        if(execute_CsrPlugin_writeEnable)begin
          CsrPlugin_mie_MEIE <= _zz_186_[0];
          CsrPlugin_mie_MTIE <= _zz_187_[0];
          CsrPlugin_mie_MSIE <= _zz_188_[0];
        end
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(IBusSimplePlugin_iBusRsp_stages_1_output_ready)begin
      _zz_54_ <= IBusSimplePlugin_iBusRsp_stages_1_output_payload;
    end
    if(IBusSimplePlugin_iBusRsp_output_ready)begin
      _zz_56_ <= IBusSimplePlugin_iBusRsp_output_payload_pc;
      _zz_57_ <= IBusSimplePlugin_iBusRsp_output_payload_rsp_error;
      _zz_58_ <= IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
      _zz_59_ <= IBusSimplePlugin_iBusRsp_output_payload_isRvc;
    end
    if(IBusSimplePlugin_injector_decodeInput_ready)begin
      IBusSimplePlugin_injector_formal_rawInDecode <= IBusSimplePlugin_iBusRsp_output_payload_rsp_inst;
    end
    `ifndef SYNTHESIS
      `ifdef FORMAL
        assert((! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck)))
      `else
        if(!(! (((dBus_rsp_ready && memory_MEMORY_ENABLE) && memory_arbitration_isValid) && memory_arbitration_isStuck))) begin
          $display("FAILURE DBusSimplePlugin doesn't allow memory stage stall when read happend");
          $finish;
        end
      `endif
    `endif
    `ifndef SYNTHESIS
      `ifdef FORMAL
        assert((! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck)))
      `else
        if(!(! (((writeBack_arbitration_isValid && writeBack_MEMORY_ENABLE) && (! writeBack_MEMORY_STORE)) && writeBack_arbitration_isStuck))) begin
          $display("FAILURE DBusSimplePlugin doesn't allow writeback stage stall when read happend");
          $finish;
        end
      `endif
    `endif
    CsrPlugin_mip_MEIP <= externalInterrupt;
    CsrPlugin_mip_MTIP <= timerInterrupt;
    CsrPlugin_mip_MSIP <= softwareInterrupt;
    CsrPlugin_mcycle <= (CsrPlugin_mcycle + 64'h0000000000000001);
    if(writeBack_arbitration_isFiring)begin
      CsrPlugin_minstret <= (CsrPlugin_minstret + 64'h0000000000000001);
    end
    if(_zz_135_)begin
      if(_zz_136_)begin
        CsrPlugin_interrupt_code <= (4'b0111);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_137_)begin
        CsrPlugin_interrupt_code <= (4'b0011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
      if(_zz_138_)begin
        CsrPlugin_interrupt_code <= (4'b1011);
        CsrPlugin_interrupt_targetPrivilege <= (2'b11);
      end
    end
    if(_zz_124_)begin
      case(CsrPlugin_targetPrivilege)
        2'b11 : begin
          CsrPlugin_mcause_interrupt <= (! CsrPlugin_hadException);
          CsrPlugin_mcause_exceptionCode <= CsrPlugin_trapCause;
          CsrPlugin_mepc <= decode_PC;
        end
        default : begin
        end
      endcase
    end
    if(_zz_120_)begin
      if(_zz_121_)begin
        execute_LightShifterPlugin_amplitudeReg <= (execute_LightShifterPlugin_amplitude - 5'h01);
      end
    end
    _zz_98_ <= _zz_29_[11 : 7];
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS2 <= _zz_23_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_DO_EBREAK <= decode_DO_EBREAK;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC1 <= decode_SRC1;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_CALC <= execute_BRANCH_CALC;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ENV_CTRL <= _zz_18_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_ENV_CTRL <= _zz_15_;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_ENV_CTRL <= _zz_13_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_CSR <= decode_IS_CSR;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_RS1 <= _zz_25_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_PC <= _zz_22_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_PC <= execute_PC;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_PC <= memory_PC;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BRANCH_DO <= execute_BRANCH_DO;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2_FORCE_ZERO <= decode_SRC2_FORCE_ZERO;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_READ_DATA <= memory_MEMORY_READ_DATA;
    end
    if(((! memory_arbitration_isStuck) && (! execute_arbitration_isStuckByOthers)))begin
      execute_to_memory_REGFILE_WRITE_DATA <= _zz_44_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_WRITE_OPCODE <= decode_CSR_WRITE_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_USE_SUB_LESS <= decode_SRC_USE_SUB_LESS;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_BITWISE_CTRL <= _zz_11_;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ADDRESS_LOW <= execute_MEMORY_ADDRESS_LOW;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ADDRESS_LOW <= memory_MEMORY_ADDRESS_LOW;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_MEMORY_STAGE <= decode_BYPASSABLE_MEMORY_STAGE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_BYPASSABLE_MEMORY_STAGE <= execute_BYPASSABLE_MEMORY_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_REGFILE_WRITE_VALID <= decode_REGFILE_WRITE_VALID;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_REGFILE_WRITE_VALID <= execute_REGFILE_WRITE_VALID;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_REGFILE_WRITE_VALID <= memory_REGFILE_WRITE_VALID;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_FORMAL_PC_NEXT <= decode_FORMAL_PC_NEXT;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_FORMAL_PC_NEXT <= execute_FORMAL_PC_NEXT;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_FORMAL_PC_NEXT <= _zz_43_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_ENABLE <= decode_MEMORY_ENABLE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_ENABLE <= execute_MEMORY_ENABLE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_ENABLE <= memory_MEMORY_ENABLE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC2 <= decode_SRC2;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_MEMORY_STORE <= decode_MEMORY_STORE;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_MEMORY_STORE <= execute_MEMORY_STORE;
    end
    if((! writeBack_arbitration_isStuck))begin
      memory_to_writeBack_MEMORY_STORE <= memory_MEMORY_STORE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BRANCH_CTRL <= _zz_8_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_CSR_READ_OPCODE <= decode_CSR_READ_OPCODE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_ALU_CTRL <= _zz_5_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_BYPASSABLE_EXECUTE_STAGE <= decode_BYPASSABLE_EXECUTE_STAGE;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SHIFT_CTRL <= _zz_2_;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_SRC_LESS_UNSIGNED <= decode_SRC_LESS_UNSIGNED;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_IS_SIMD_ADD <= decode_IS_SIMD_ADD;
    end
    if((! execute_arbitration_isStuck))begin
      decode_to_execute_INSTRUCTION <= decode_INSTRUCTION;
    end
    if((! memory_arbitration_isStuck))begin
      execute_to_memory_INSTRUCTION <= execute_INSTRUCTION;
    end
    if((_zz_110_ != (3'b000)))begin
      _zz_58_ <= IBusSimplePlugin_injectionPort_payload;
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_768 <= (decode_INSTRUCTION[31 : 20] == 12'h300);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_836 <= (decode_INSTRUCTION[31 : 20] == 12'h344);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_772 <= (decode_INSTRUCTION[31 : 20] == 12'h304);
    end
    if((! execute_arbitration_isStuck))begin
      execute_CsrPlugin_csr_834 <= (decode_INSTRUCTION[31 : 20] == 12'h342);
    end
    if(execute_CsrPlugin_csr_836)begin
      if(execute_CsrPlugin_writeEnable)begin
        CsrPlugin_mip_MSIP <= _zz_185_[0];
      end
    end
  end

  always @ (posedge io_mainClk) begin
    DebugPlugin_firstCycle <= 1'b0;
    if(debug_bus_cmd_ready)begin
      DebugPlugin_firstCycle <= 1'b1;
    end
    DebugPlugin_secondCycle <= DebugPlugin_firstCycle;
    DebugPlugin_isPipBusy <= (({writeBack_arbitration_isValid,{memory_arbitration_isValid,{execute_arbitration_isValid,decode_arbitration_isValid}}} != (4'b0000)) || IBusSimplePlugin_incomingInstruction);
    if(writeBack_arbitration_isValid)begin
      DebugPlugin_busReadDataReg <= _zz_42_;
    end
    _zz_109_ <= debug_bus_cmd_payload_address[2];
    if(_zz_122_)begin
      DebugPlugin_busReadDataReg <= execute_PC;
    end
    DebugPlugin_resetIt_regNext <= DebugPlugin_resetIt;
  end

  always @ (posedge io_mainClk or posedge resetCtrl_mainClkReset) begin
    if (resetCtrl_mainClkReset) begin
      DebugPlugin_resetIt <= 1'b0;
      DebugPlugin_haltIt <= 1'b0;
      DebugPlugin_stepIt <= 1'b0;
      DebugPlugin_godmode <= 1'b0;
      DebugPlugin_haltedByBreak <= 1'b0;
    end else begin
      if((DebugPlugin_haltIt && (! DebugPlugin_isPipBusy)))begin
        DebugPlugin_godmode <= 1'b1;
      end
      if(debug_bus_cmd_valid)begin
        case(_zz_134_)
          6'b000000 : begin
            if(debug_bus_cmd_payload_wr)begin
              DebugPlugin_stepIt <= debug_bus_cmd_payload_data[4];
              if(debug_bus_cmd_payload_data[16])begin
                DebugPlugin_resetIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[24])begin
                DebugPlugin_resetIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[17])begin
                DebugPlugin_haltIt <= 1'b1;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltIt <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_haltedByBreak <= 1'b0;
              end
              if(debug_bus_cmd_payload_data[25])begin
                DebugPlugin_godmode <= 1'b0;
              end
            end
          end
          6'b000001 : begin
          end
          default : begin
          end
        endcase
      end
      if(_zz_122_)begin
        if(_zz_123_)begin
          DebugPlugin_haltIt <= 1'b1;
          DebugPlugin_haltedByBreak <= 1'b1;
        end
      end
      if(_zz_126_)begin
        if(decode_arbitration_isValid)begin
          DebugPlugin_haltIt <= 1'b1;
        end
      end
    end
  end


endmodule

module JtagBridge (
  input               io_jtag_tms,
  input               io_jtag_tdi,
  output              io_jtag_tdo,
  input               io_jtag_tck,
  output              io_remote_cmd_valid,
  input               io_remote_cmd_ready,
  output              io_remote_cmd_payload_last,
  output     [0:0]    io_remote_cmd_payload_fragment,
  input               io_remote_rsp_valid,
  output              io_remote_rsp_ready,
  input               io_remote_rsp_payload_error,
  input      [31:0]   io_remote_rsp_payload_data,
  input               io_mainClk,
  input               resetCtrl_mainClkReset 
);
  wire                flowCCByToggle_1__io_output_valid;
  wire                flowCCByToggle_1__io_output_payload_last;
  wire       [0:0]    flowCCByToggle_1__io_output_payload_fragment;
  wire                _zz_2_;
  wire                _zz_3_;
  wire       [0:0]    _zz_4_;
  wire       [3:0]    _zz_5_;
  wire       [1:0]    _zz_6_;
  wire       [3:0]    _zz_7_;
  wire       [1:0]    _zz_8_;
  wire       [3:0]    _zz_9_;
  wire       [0:0]    _zz_10_;
  wire                system_cmd_valid;
  wire                system_cmd_payload_last;
  wire       [0:0]    system_cmd_payload_fragment;
  reg                 system_rsp_valid;
  reg                 system_rsp_payload_error;
  reg        [31:0]   system_rsp_payload_data;
  wire       `JtagState_defaultEncoding_type jtag_tap_fsm_stateNext;
  reg        `JtagState_defaultEncoding_type jtag_tap_fsm_state = `JtagState_defaultEncoding_RESET;
  reg        `JtagState_defaultEncoding_type _zz_1_;
  reg        [3:0]    jtag_tap_instruction;
  reg        [3:0]    jtag_tap_instructionShift;
  reg                 jtag_tap_bypass;
  reg                 jtag_tap_tdoUnbufferd;
  reg                 jtag_tap_tdoUnbufferd_regNext;
  wire                jtag_idcodeArea_instructionHit;
  reg        [31:0]   jtag_idcodeArea_shifter;
  wire                jtag_writeArea_instructionHit;
  reg                 jtag_writeArea_source_valid;
  wire                jtag_writeArea_source_payload_last;
  wire       [0:0]    jtag_writeArea_source_payload_fragment;
  wire                jtag_readArea_instructionHit;
  reg        [33:0]   jtag_readArea_shifter;
  `ifndef SYNTHESIS
  reg [79:0] jtag_tap_fsm_stateNext_string;
  reg [79:0] jtag_tap_fsm_state_string;
  reg [79:0] _zz_1__string;
  `endif


  assign _zz_2_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_3_ = (jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT);
  assign _zz_4_ = (1'b1);
  assign _zz_5_ = {3'd0, _zz_4_};
  assign _zz_6_ = (2'b10);
  assign _zz_7_ = {2'd0, _zz_6_};
  assign _zz_8_ = (2'b11);
  assign _zz_9_ = {2'd0, _zz_8_};
  assign _zz_10_ = (1'b1);
  FlowCCByToggle flowCCByToggle_1_ ( 
    .io_input_valid                (jtag_writeArea_source_valid                   ), //i
    .io_input_payload_last         (jtag_writeArea_source_payload_last            ), //i
    .io_input_payload_fragment     (jtag_writeArea_source_payload_fragment        ), //i
    .io_output_valid               (flowCCByToggle_1__io_output_valid             ), //o
    .io_output_payload_last        (flowCCByToggle_1__io_output_payload_last      ), //o
    .io_output_payload_fragment    (flowCCByToggle_1__io_output_payload_fragment  ), //o
    .io_jtag_tck                   (io_jtag_tck                                   ), //i
    .io_mainClk                    (io_mainClk                                    ), //i
    .resetCtrl_mainClkReset        (resetCtrl_mainClkReset                        )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtag_tap_fsm_stateNext)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_stateNext_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_stateNext_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_stateNext_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_stateNext_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_stateNext_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_stateNext_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_stateNext_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_stateNext_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_stateNext_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_stateNext_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_stateNext_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_stateNext_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_stateNext_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_stateNext_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_stateNext_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_stateNext_string = "DR_UPDATE ";
      default : jtag_tap_fsm_stateNext_string = "??????????";
    endcase
  end
  always @(*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_RESET : jtag_tap_fsm_state_string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : jtag_tap_fsm_state_string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : jtag_tap_fsm_state_string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : jtag_tap_fsm_state_string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : jtag_tap_fsm_state_string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : jtag_tap_fsm_state_string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : jtag_tap_fsm_state_string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : jtag_tap_fsm_state_string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : jtag_tap_fsm_state_string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : jtag_tap_fsm_state_string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : jtag_tap_fsm_state_string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : jtag_tap_fsm_state_string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : jtag_tap_fsm_state_string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : jtag_tap_fsm_state_string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : jtag_tap_fsm_state_string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : jtag_tap_fsm_state_string = "DR_UPDATE ";
      default : jtag_tap_fsm_state_string = "??????????";
    endcase
  end
  always @(*) begin
    case(_zz_1_)
      `JtagState_defaultEncoding_RESET : _zz_1__string = "RESET     ";
      `JtagState_defaultEncoding_IDLE : _zz_1__string = "IDLE      ";
      `JtagState_defaultEncoding_IR_SELECT : _zz_1__string = "IR_SELECT ";
      `JtagState_defaultEncoding_IR_CAPTURE : _zz_1__string = "IR_CAPTURE";
      `JtagState_defaultEncoding_IR_SHIFT : _zz_1__string = "IR_SHIFT  ";
      `JtagState_defaultEncoding_IR_EXIT1 : _zz_1__string = "IR_EXIT1  ";
      `JtagState_defaultEncoding_IR_PAUSE : _zz_1__string = "IR_PAUSE  ";
      `JtagState_defaultEncoding_IR_EXIT2 : _zz_1__string = "IR_EXIT2  ";
      `JtagState_defaultEncoding_IR_UPDATE : _zz_1__string = "IR_UPDATE ";
      `JtagState_defaultEncoding_DR_SELECT : _zz_1__string = "DR_SELECT ";
      `JtagState_defaultEncoding_DR_CAPTURE : _zz_1__string = "DR_CAPTURE";
      `JtagState_defaultEncoding_DR_SHIFT : _zz_1__string = "DR_SHIFT  ";
      `JtagState_defaultEncoding_DR_EXIT1 : _zz_1__string = "DR_EXIT1  ";
      `JtagState_defaultEncoding_DR_PAUSE : _zz_1__string = "DR_PAUSE  ";
      `JtagState_defaultEncoding_DR_EXIT2 : _zz_1__string = "DR_EXIT2  ";
      `JtagState_defaultEncoding_DR_UPDATE : _zz_1__string = "DR_UPDATE ";
      default : _zz_1__string = "??????????";
    endcase
  end
  `endif

  assign io_remote_cmd_valid = system_cmd_valid;
  assign io_remote_cmd_payload_last = system_cmd_payload_last;
  assign io_remote_cmd_payload_fragment = system_cmd_payload_fragment;
  assign io_remote_rsp_ready = 1'b1;
  always @ (*) begin
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IDLE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_IR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IR_CAPTURE);
      end
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT1 : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_EXIT2 : `JtagState_defaultEncoding_IR_PAUSE);
      end
      `JtagState_defaultEncoding_IR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_UPDATE : `JtagState_defaultEncoding_IR_SHIFT);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      `JtagState_defaultEncoding_DR_SELECT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_IR_SELECT : `JtagState_defaultEncoding_DR_CAPTURE);
      end
      `JtagState_defaultEncoding_DR_CAPTURE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_SHIFT : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT1 : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_EXIT1 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_PAUSE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_EXIT2 : `JtagState_defaultEncoding_DR_PAUSE);
      end
      `JtagState_defaultEncoding_DR_EXIT2 : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_UPDATE : `JtagState_defaultEncoding_DR_SHIFT);
      end
      `JtagState_defaultEncoding_DR_UPDATE : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_DR_SELECT : `JtagState_defaultEncoding_IDLE);
      end
      default : begin
        _zz_1_ = (io_jtag_tms ? `JtagState_defaultEncoding_RESET : `JtagState_defaultEncoding_IDLE);
      end
    endcase
  end

  assign jtag_tap_fsm_stateNext = _zz_1_;
  always @ (*) begin
    jtag_tap_tdoUnbufferd = jtag_tap_bypass;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_tdoUnbufferd = jtag_tap_instructionShift[0];
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_tap_tdoUnbufferd = jtag_idcodeArea_shifter[0];
      end
    end
    if(jtag_readArea_instructionHit)begin
      if(_zz_3_)begin
        jtag_tap_tdoUnbufferd = jtag_readArea_shifter[0];
      end
    end
  end

  assign io_jtag_tdo = jtag_tap_tdoUnbufferd_regNext;
  assign jtag_idcodeArea_instructionHit = (jtag_tap_instruction == _zz_5_);
  assign jtag_writeArea_instructionHit = (jtag_tap_instruction == _zz_7_);
  always @ (*) begin
    jtag_writeArea_source_valid = 1'b0;
    if(jtag_writeArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_SHIFT))begin
        jtag_writeArea_source_valid = 1'b1;
      end
    end
  end

  assign jtag_writeArea_source_payload_last = io_jtag_tms;
  assign jtag_writeArea_source_payload_fragment[0] = io_jtag_tdi;
  assign system_cmd_valid = flowCCByToggle_1__io_output_valid;
  assign system_cmd_payload_last = flowCCByToggle_1__io_output_payload_last;
  assign system_cmd_payload_fragment = flowCCByToggle_1__io_output_payload_fragment;
  assign jtag_readArea_instructionHit = (jtag_tap_instruction == _zz_9_);
  always @ (posedge io_mainClk) begin
    if(io_remote_cmd_valid)begin
      system_rsp_valid <= 1'b0;
    end
    if((io_remote_rsp_valid && io_remote_rsp_ready))begin
      system_rsp_valid <= 1'b1;
      system_rsp_payload_error <= io_remote_rsp_payload_error;
      system_rsp_payload_data <= io_remote_rsp_payload_data;
    end
  end

  always @ (posedge io_jtag_tck) begin
    jtag_tap_fsm_state <= jtag_tap_fsm_stateNext;
    jtag_tap_bypass <= io_jtag_tdi;
    case(jtag_tap_fsm_state)
      `JtagState_defaultEncoding_IR_CAPTURE : begin
        jtag_tap_instructionShift <= jtag_tap_instruction;
      end
      `JtagState_defaultEncoding_IR_SHIFT : begin
        jtag_tap_instructionShift <= ({io_jtag_tdi,jtag_tap_instructionShift} >>> 1);
      end
      `JtagState_defaultEncoding_IR_UPDATE : begin
        jtag_tap_instruction <= jtag_tap_instructionShift;
      end
      default : begin
      end
    endcase
    if(jtag_idcodeArea_instructionHit)begin
      if(_zz_2_)begin
        jtag_idcodeArea_shifter <= ({io_jtag_tdi,jtag_idcodeArea_shifter} >>> 1);
      end
    end
    if((jtag_tap_fsm_state == `JtagState_defaultEncoding_RESET))begin
      jtag_idcodeArea_shifter <= 32'h10001fff;
      jtag_tap_instruction <= {3'd0, _zz_10_};
    end
    if(jtag_readArea_instructionHit)begin
      if((jtag_tap_fsm_state == `JtagState_defaultEncoding_DR_CAPTURE))begin
        jtag_readArea_shifter <= {{system_rsp_payload_data,system_rsp_payload_error},system_rsp_valid};
      end
      if(_zz_3_)begin
        jtag_readArea_shifter <= ({io_jtag_tdi,jtag_readArea_shifter} >>> 1);
      end
    end
  end

  always @ (negedge io_jtag_tck) begin
    jtag_tap_tdoUnbufferd_regNext <= jtag_tap_tdoUnbufferd;
  end


endmodule

module SystemDebugger (
  input               io_remote_cmd_valid,
  output              io_remote_cmd_ready,
  input               io_remote_cmd_payload_last,
  input      [0:0]    io_remote_cmd_payload_fragment,
  output              io_remote_rsp_valid,
  input               io_remote_rsp_ready,
  output              io_remote_rsp_payload_error,
  output     [31:0]   io_remote_rsp_payload_data,
  output              io_mem_cmd_valid,
  input               io_mem_cmd_ready,
  output     [31:0]   io_mem_cmd_payload_address,
  output     [31:0]   io_mem_cmd_payload_data,
  output              io_mem_cmd_payload_wr,
  output     [1:0]    io_mem_cmd_payload_size,
  input               io_mem_rsp_valid,
  input      [31:0]   io_mem_rsp_payload,
  input               io_mainClk,
  input               resetCtrl_mainClkReset 
);
  wire                _zz_2_;
  wire       [0:0]    _zz_3_;
  reg        [66:0]   dispatcher_dataShifter;
  reg                 dispatcher_dataLoaded;
  reg        [7:0]    dispatcher_headerShifter;
  wire       [7:0]    dispatcher_header;
  reg                 dispatcher_headerLoaded;
  reg        [2:0]    dispatcher_counter;
  wire       [66:0]   _zz_1_;

  assign _zz_2_ = (dispatcher_headerLoaded == 1'b0);
  assign _zz_3_ = _zz_1_[64 : 64];
  assign dispatcher_header = dispatcher_headerShifter[7 : 0];
  assign io_remote_cmd_ready = (! dispatcher_dataLoaded);
  assign _zz_1_ = dispatcher_dataShifter[66 : 0];
  assign io_mem_cmd_payload_address = _zz_1_[31 : 0];
  assign io_mem_cmd_payload_data = _zz_1_[63 : 32];
  assign io_mem_cmd_payload_wr = _zz_3_[0];
  assign io_mem_cmd_payload_size = _zz_1_[66 : 65];
  assign io_mem_cmd_valid = (dispatcher_dataLoaded && (dispatcher_header == 8'h0));
  assign io_remote_rsp_valid = io_mem_rsp_valid;
  assign io_remote_rsp_payload_error = 1'b0;
  assign io_remote_rsp_payload_data = io_mem_rsp_payload;
  always @ (posedge io_mainClk or posedge resetCtrl_mainClkReset) begin
    if (resetCtrl_mainClkReset) begin
      dispatcher_dataLoaded <= 1'b0;
      dispatcher_headerLoaded <= 1'b0;
      dispatcher_counter <= (3'b000);
    end else begin
      if(io_remote_cmd_valid)begin
        if(_zz_2_)begin
          dispatcher_counter <= (dispatcher_counter + (3'b001));
          if((dispatcher_counter == (3'b111)))begin
            dispatcher_headerLoaded <= 1'b1;
          end
        end
        if(io_remote_cmd_payload_last)begin
          dispatcher_headerLoaded <= 1'b1;
          dispatcher_dataLoaded <= 1'b1;
          dispatcher_counter <= (3'b000);
        end
      end
      if((io_mem_cmd_valid && io_mem_cmd_ready))begin
        dispatcher_headerLoaded <= 1'b0;
        dispatcher_dataLoaded <= 1'b0;
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(io_remote_cmd_valid)begin
      if(_zz_2_)begin
        dispatcher_headerShifter <= ({io_remote_cmd_payload_fragment,dispatcher_headerShifter} >>> 1);
      end else begin
        dispatcher_dataShifter <= ({io_remote_cmd_payload_fragment,dispatcher_dataShifter} >>> 1);
      end
    end
  end


endmodule

module MuraxPipelinedMemoryBusRam (
  input               io_bus_cmd_valid,
  output              io_bus_cmd_ready,
  input               io_bus_cmd_payload_write,
  input      [31:0]   io_bus_cmd_payload_address,
  input      [31:0]   io_bus_cmd_payload_data,
  input      [3:0]    io_bus_cmd_payload_mask,
  output              io_bus_rsp_valid,
  output     [31:0]   io_bus_rsp_payload_data,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg        [31:0]   _zz_4_;
  wire       [10:0]   _zz_5_;
  reg                 _zz_1_;
  wire       [29:0]   _zz_2_;
  wire       [31:0]   _zz_3_;
  reg [7:0] ram_symbol0 [0:2047];
  reg [7:0] ram_symbol1 [0:2047];
  reg [7:0] ram_symbol2 [0:2047];
  reg [7:0] ram_symbol3 [0:2047];
  reg [7:0] _zz_6_;
  reg [7:0] _zz_7_;
  reg [7:0] _zz_8_;
  reg [7:0] _zz_9_;

  assign _zz_5_ = _zz_2_[10:0];
  always @ (*) begin
    _zz_4_ = {_zz_9_, _zz_8_, _zz_7_, _zz_6_};
  end
  always @ (posedge io_mainClk) begin
    if(io_bus_cmd_valid) begin
      _zz_6_ <= ram_symbol0[_zz_5_];
      _zz_7_ <= ram_symbol1[_zz_5_];
      _zz_8_ <= ram_symbol2[_zz_5_];
      _zz_9_ <= ram_symbol3[_zz_5_];
    end
  end

  always @ (posedge io_mainClk) begin
    if(io_bus_cmd_payload_mask[0] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol0[_zz_5_] <= _zz_3_[7 : 0];
    end
    if(io_bus_cmd_payload_mask[1] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol1[_zz_5_] <= _zz_3_[15 : 8];
    end
    if(io_bus_cmd_payload_mask[2] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol2[_zz_5_] <= _zz_3_[23 : 16];
    end
    if(io_bus_cmd_payload_mask[3] && io_bus_cmd_valid && io_bus_cmd_payload_write ) begin
      ram_symbol3[_zz_5_] <= _zz_3_[31 : 24];
    end
  end

  assign io_bus_rsp_valid = _zz_1_;
  assign _zz_2_ = (io_bus_cmd_payload_address >>> 2);
  assign _zz_3_ = io_bus_cmd_payload_data;
  assign io_bus_rsp_payload_data = _zz_4_;
  assign io_bus_cmd_ready = 1'b1;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      _zz_1_ <= 1'b0;
    end else begin
      _zz_1_ <= ((io_bus_cmd_valid && io_bus_cmd_ready) && (! io_bus_cmd_payload_write));
    end
  end


endmodule

module PipelinedMemoryBusToApbBridge (
  input               io_pipelinedMemoryBus_cmd_valid,
  output              io_pipelinedMemoryBus_cmd_ready,
  input               io_pipelinedMemoryBus_cmd_payload_write,
  input      [31:0]   io_pipelinedMemoryBus_cmd_payload_address,
  input      [31:0]   io_pipelinedMemoryBus_cmd_payload_data,
  input      [3:0]    io_pipelinedMemoryBus_cmd_payload_mask,
  output              io_pipelinedMemoryBus_rsp_valid,
  output     [31:0]   io_pipelinedMemoryBus_rsp_payload_data,
  output     [19:0]   io_apb_PADDR,
  output     [0:0]    io_apb_PSEL,
  output              io_apb_PENABLE,
  input               io_apb_PREADY,
  output              io_apb_PWRITE,
  output     [31:0]   io_apb_PWDATA,
  input      [31:0]   io_apb_PRDATA,
  input               io_apb_PSLVERROR,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_1_;
  wire                _zz_2_;
  wire                pipelinedMemoryBusStage_cmd_valid;
  reg                 pipelinedMemoryBusStage_cmd_ready;
  wire                pipelinedMemoryBusStage_cmd_payload_write;
  wire       [31:0]   pipelinedMemoryBusStage_cmd_payload_address;
  wire       [31:0]   pipelinedMemoryBusStage_cmd_payload_data;
  wire       [3:0]    pipelinedMemoryBusStage_cmd_payload_mask;
  reg                 pipelinedMemoryBusStage_rsp_valid;
  wire       [31:0]   pipelinedMemoryBusStage_rsp_payload_data;
  wire                io_pipelinedMemoryBus_cmd_halfPipe_valid;
  wire                io_pipelinedMemoryBus_cmd_halfPipe_ready;
  wire                io_pipelinedMemoryBus_cmd_halfPipe_payload_write;
  wire       [31:0]   io_pipelinedMemoryBus_cmd_halfPipe_payload_address;
  wire       [31:0]   io_pipelinedMemoryBus_cmd_halfPipe_payload_data;
  wire       [3:0]    io_pipelinedMemoryBus_cmd_halfPipe_payload_mask;
  reg                 io_pipelinedMemoryBus_cmd_halfPipe_regs_valid;
  reg                 io_pipelinedMemoryBus_cmd_halfPipe_regs_ready;
  reg                 io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write;
  reg        [31:0]   io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address;
  reg        [31:0]   io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data;
  reg        [3:0]    io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask;
  reg                 pipelinedMemoryBusStage_rsp_regNext_valid;
  reg        [31:0]   pipelinedMemoryBusStage_rsp_regNext_payload_data;
  reg                 state;

  assign _zz_1_ = (! state);
  assign _zz_2_ = (! io_pipelinedMemoryBus_cmd_halfPipe_regs_valid);
  assign io_pipelinedMemoryBus_cmd_halfPipe_valid = io_pipelinedMemoryBus_cmd_halfPipe_regs_valid;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_write = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_address = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_data = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data;
  assign io_pipelinedMemoryBus_cmd_halfPipe_payload_mask = io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask;
  assign io_pipelinedMemoryBus_cmd_ready = io_pipelinedMemoryBus_cmd_halfPipe_regs_ready;
  assign pipelinedMemoryBusStage_cmd_valid = io_pipelinedMemoryBus_cmd_halfPipe_valid;
  assign io_pipelinedMemoryBus_cmd_halfPipe_ready = pipelinedMemoryBusStage_cmd_ready;
  assign pipelinedMemoryBusStage_cmd_payload_write = io_pipelinedMemoryBus_cmd_halfPipe_payload_write;
  assign pipelinedMemoryBusStage_cmd_payload_address = io_pipelinedMemoryBus_cmd_halfPipe_payload_address;
  assign pipelinedMemoryBusStage_cmd_payload_data = io_pipelinedMemoryBus_cmd_halfPipe_payload_data;
  assign pipelinedMemoryBusStage_cmd_payload_mask = io_pipelinedMemoryBus_cmd_halfPipe_payload_mask;
  assign io_pipelinedMemoryBus_rsp_valid = pipelinedMemoryBusStage_rsp_regNext_valid;
  assign io_pipelinedMemoryBus_rsp_payload_data = pipelinedMemoryBusStage_rsp_regNext_payload_data;
  always @ (*) begin
    pipelinedMemoryBusStage_cmd_ready = 1'b0;
    if(! _zz_1_) begin
      if(io_apb_PREADY)begin
        pipelinedMemoryBusStage_cmd_ready = 1'b1;
      end
    end
  end

  assign io_apb_PSEL[0] = pipelinedMemoryBusStage_cmd_valid;
  assign io_apb_PENABLE = state;
  assign io_apb_PWRITE = pipelinedMemoryBusStage_cmd_payload_write;
  assign io_apb_PADDR = pipelinedMemoryBusStage_cmd_payload_address[19:0];
  assign io_apb_PWDATA = pipelinedMemoryBusStage_cmd_payload_data;
  always @ (*) begin
    pipelinedMemoryBusStage_rsp_valid = 1'b0;
    if(! _zz_1_) begin
      if(io_apb_PREADY)begin
        pipelinedMemoryBusStage_rsp_valid = (! pipelinedMemoryBusStage_cmd_payload_write);
      end
    end
  end

  assign pipelinedMemoryBusStage_rsp_payload_data = io_apb_PRDATA;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= 1'b0;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= 1'b1;
      pipelinedMemoryBusStage_rsp_regNext_valid <= 1'b0;
      state <= 1'b0;
    end else begin
      if(_zz_2_)begin
        io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= io_pipelinedMemoryBus_cmd_valid;
        io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= (! io_pipelinedMemoryBus_cmd_valid);
      end else begin
        io_pipelinedMemoryBus_cmd_halfPipe_regs_valid <= (! io_pipelinedMemoryBus_cmd_halfPipe_ready);
        io_pipelinedMemoryBus_cmd_halfPipe_regs_ready <= io_pipelinedMemoryBus_cmd_halfPipe_ready;
      end
      pipelinedMemoryBusStage_rsp_regNext_valid <= pipelinedMemoryBusStage_rsp_valid;
      if(_zz_1_)begin
        state <= pipelinedMemoryBusStage_cmd_valid;
      end else begin
        if(io_apb_PREADY)begin
          state <= 1'b0;
        end
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_2_)begin
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_write <= io_pipelinedMemoryBus_cmd_payload_write;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_address <= io_pipelinedMemoryBus_cmd_payload_address;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_data <= io_pipelinedMemoryBus_cmd_payload_data;
      io_pipelinedMemoryBus_cmd_halfPipe_regs_payload_mask <= io_pipelinedMemoryBus_cmd_payload_mask;
    end
    pipelinedMemoryBusStage_rsp_regNext_payload_data <= pipelinedMemoryBusStage_rsp_payload_data;
  end


endmodule

module Apb3Gpio (
  input      [3:0]    io_apb_PADDR,
  input      [0:0]    io_apb_PSEL,
  input               io_apb_PENABLE,
  output              io_apb_PREADY,
  input               io_apb_PWRITE,
  input      [31:0]   io_apb_PWDATA,
  output reg [31:0]   io_apb_PRDATA,
  output              io_apb_PSLVERROR,
  input      [31:0]   io_gpio_read,
  output     [31:0]   io_gpio_write,
  output     [31:0]   io_gpio_writeEnable,
  output     [31:0]   io_value,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire       [31:0]   io_gpio_read_buffercc_io_dataOut;
  wire                ctrl_askWrite;
  wire                ctrl_askRead;
  wire                ctrl_doWrite;
  wire                ctrl_doRead;
  reg        [31:0]   io_gpio_write_driver;
  reg        [31:0]   io_gpio_writeEnable_driver;

  BufferCC_2_ io_gpio_read_buffercc ( 
    .io_dataIn                (io_gpio_read[31:0]                      ), //i
    .io_dataOut               (io_gpio_read_buffercc_io_dataOut[31:0]  ), //o
    .io_mainClk               (io_mainClk                              ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                   )  //i
  );
  assign io_value = io_gpio_read_buffercc_io_dataOut;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = 32'h0;
    case(io_apb_PADDR)
      4'b0000 : begin
        io_apb_PRDATA[31 : 0] = io_value;
      end
      4'b0100 : begin
        io_apb_PRDATA[31 : 0] = io_gpio_write_driver;
      end
      4'b1000 : begin
        io_apb_PRDATA[31 : 0] = io_gpio_writeEnable_driver;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign ctrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign ctrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign ctrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign ctrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign io_gpio_write = io_gpio_write_driver;
  assign io_gpio_writeEnable = io_gpio_writeEnable_driver;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      io_gpio_writeEnable_driver <= 32'h0;
    end else begin
      case(io_apb_PADDR)
        4'b0000 : begin
        end
        4'b0100 : begin
        end
        4'b1000 : begin
          if(ctrl_doWrite)begin
            io_gpio_writeEnable_driver <= io_apb_PWDATA[31 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_mainClk) begin
    case(io_apb_PADDR)
      4'b0000 : begin
      end
      4'b0100 : begin
        if(ctrl_doWrite)begin
          io_gpio_write_driver <= io_apb_PWDATA[31 : 0];
        end
      end
      4'b1000 : begin
      end
      default : begin
      end
    endcase
  end


endmodule

module Apb3UartCtrl (
  input      [4:0]    io_apb_PADDR,
  input      [0:0]    io_apb_PSEL,
  input               io_apb_PENABLE,
  output              io_apb_PREADY,
  input               io_apb_PWRITE,
  input      [31:0]   io_apb_PWDATA,
  output reg [31:0]   io_apb_PRDATA,
  output              io_uart_txd,
  input               io_uart_rxd,
  output              io_interrupt,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_7_;
  reg                 _zz_8_;
  wire                _zz_9_;
  wire                uartCtrl_1__io_write_ready;
  wire                uartCtrl_1__io_read_valid;
  wire       [7:0]    uartCtrl_1__io_read_payload;
  wire                uartCtrl_1__io_uart_txd;
  wire                uartCtrl_1__io_readError;
  wire                uartCtrl_1__io_readBreak;
  wire                bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  wire                bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload;
  wire       [4:0]    bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy;
  wire       [4:0]    bridge_write_streamUnbuffered_queueWithOccupancy_io_availability;
  wire                uartCtrl_1__io_read_queueWithOccupancy_io_push_ready;
  wire                uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
  wire       [4:0]    uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
  wire       [4:0]    uartCtrl_1__io_read_queueWithOccupancy_io_availability;
  wire       [0:0]    _zz_10_;
  wire       [0:0]    _zz_11_;
  wire       [0:0]    _zz_12_;
  wire       [0:0]    _zz_13_;
  wire       [0:0]    _zz_14_;
  wire       [0:0]    _zz_15_;
  wire       [0:0]    _zz_16_;
  wire       [0:0]    _zz_17_;
  wire       [0:0]    _zz_18_;
  wire       [0:0]    _zz_19_;
  wire       [0:0]    _zz_20_;
  wire       [0:0]    _zz_21_;
  wire       [4:0]    _zz_22_;
  wire                busCtrl_askWrite;
  wire                busCtrl_askRead;
  wire                busCtrl_doWrite;
  wire                busCtrl_doRead;
  wire       [2:0]    bridge_uartConfigReg_frame_dataLength;
  wire       `UartStopType_defaultEncoding_type bridge_uartConfigReg_frame_stop;
  wire       `UartParityType_defaultEncoding_type bridge_uartConfigReg_frame_parity;
  reg        [19:0]   bridge_uartConfigReg_clockDivider;
  reg                 _zz_1_;
  wire                bridge_write_streamUnbuffered_valid;
  wire                bridge_write_streamUnbuffered_ready;
  wire       [7:0]    bridge_write_streamUnbuffered_payload;
  reg                 bridge_read_streamBreaked_valid;
  reg                 bridge_read_streamBreaked_ready;
  wire       [7:0]    bridge_read_streamBreaked_payload;
  reg                 bridge_interruptCtrl_writeIntEnable;
  reg                 bridge_interruptCtrl_readIntEnable;
  wire                bridge_interruptCtrl_readInt;
  wire                bridge_interruptCtrl_writeInt;
  wire                bridge_interruptCtrl_interrupt;
  reg                 bridge_misc_readError;
  reg                 _zz_2_;
  reg                 bridge_misc_readOverflowError;
  reg                 _zz_3_;
  reg                 bridge_misc_breakDetected;
  reg                 uartCtrl_1__io_readBreak_regNext;
  reg                 _zz_4_;
  reg                 bridge_misc_doBreak;
  reg                 _zz_5_;
  reg                 _zz_6_;
  `ifndef SYNTHESIS
  reg [23:0] bridge_uartConfigReg_frame_stop_string;
  reg [31:0] bridge_uartConfigReg_frame_parity_string;
  `endif

  function [19:0] zz_bridge_uartConfigReg_clockDivider(input dummy);
    begin
      zz_bridge_uartConfigReg_clockDivider = 20'h0;
      zz_bridge_uartConfigReg_clockDivider = 20'h0002a;
    end
  endfunction
  wire [19:0] _zz_23_;

  assign _zz_10_ = io_apb_PWDATA[0 : 0];
  assign _zz_11_ = (1'b0);
  assign _zz_12_ = io_apb_PWDATA[1 : 1];
  assign _zz_13_ = (1'b0);
  assign _zz_14_ = io_apb_PWDATA[9 : 9];
  assign _zz_15_ = (1'b0);
  assign _zz_16_ = io_apb_PWDATA[10 : 10];
  assign _zz_17_ = (1'b1);
  assign _zz_18_ = io_apb_PWDATA[11 : 11];
  assign _zz_19_ = (1'b0);
  assign _zz_20_ = io_apb_PWDATA[0 : 0];
  assign _zz_21_ = io_apb_PWDATA[1 : 1];
  assign _zz_22_ = (5'h10 - bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy);
  UartCtrl uartCtrl_1_ ( 
    .io_config_frame_dataLength    (bridge_uartConfigReg_frame_dataLength[2:0]                            ), //i
    .io_config_frame_stop          (bridge_uartConfigReg_frame_stop                                       ), //i
    .io_config_frame_parity        (bridge_uartConfigReg_frame_parity[1:0]                                ), //i
    .io_config_clockDivider        (bridge_uartConfigReg_clockDivider[19:0]                               ), //i
    .io_write_valid                (bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid         ), //i
    .io_write_ready                (uartCtrl_1__io_write_ready                                            ), //o
    .io_write_payload              (bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0]  ), //i
    .io_read_valid                 (uartCtrl_1__io_read_valid                                             ), //o
    .io_read_ready                 (uartCtrl_1__io_read_queueWithOccupancy_io_push_ready                  ), //i
    .io_read_payload               (uartCtrl_1__io_read_payload[7:0]                                      ), //o
    .io_uart_txd                   (uartCtrl_1__io_uart_txd                                               ), //o
    .io_uart_rxd                   (io_uart_rxd                                                           ), //i
    .io_readError                  (uartCtrl_1__io_readError                                              ), //o
    .io_writeBreak                 (bridge_misc_doBreak                                                   ), //i
    .io_readBreak                  (uartCtrl_1__io_readBreak                                              ), //o
    .io_mainClk                    (io_mainClk                                                            ), //i
    .resetCtrl_systemReset         (resetCtrl_systemReset                                                 )  //i
  );
  StreamFifo bridge_write_streamUnbuffered_queueWithOccupancy ( 
    .io_push_valid            (bridge_write_streamUnbuffered_valid                                    ), //i
    .io_push_ready            (bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready         ), //o
    .io_push_payload          (bridge_write_streamUnbuffered_payload[7:0]                             ), //i
    .io_pop_valid             (bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid          ), //o
    .io_pop_ready             (uartCtrl_1__io_write_ready                                             ), //i
    .io_pop_payload           (bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0]   ), //o
    .io_flush                 (_zz_7_                                                                 ), //i
    .io_occupancy             (bridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy[4:0]     ), //o
    .io_availability          (bridge_write_streamUnbuffered_queueWithOccupancy_io_availability[4:0]  ), //o
    .io_mainClk               (io_mainClk                                                             ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                                                  )  //i
  );
  StreamFifo uartCtrl_1__io_read_queueWithOccupancy ( 
    .io_push_valid            (uartCtrl_1__io_read_valid                                    ), //i
    .io_push_ready            (uartCtrl_1__io_read_queueWithOccupancy_io_push_ready         ), //o
    .io_push_payload          (uartCtrl_1__io_read_payload[7:0]                             ), //i
    .io_pop_valid             (uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid          ), //o
    .io_pop_ready             (_zz_8_                                                       ), //i
    .io_pop_payload           (uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload[7:0]   ), //o
    .io_flush                 (_zz_9_                                                       ), //i
    .io_occupancy             (uartCtrl_1__io_read_queueWithOccupancy_io_occupancy[4:0]     ), //o
    .io_availability          (uartCtrl_1__io_read_queueWithOccupancy_io_availability[4:0]  ), //o
    .io_mainClk               (io_mainClk                                                   ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                                        )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(bridge_uartConfigReg_frame_stop)
      `UartStopType_defaultEncoding_ONE : bridge_uartConfigReg_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : bridge_uartConfigReg_frame_stop_string = "TWO";
      default : bridge_uartConfigReg_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(bridge_uartConfigReg_frame_parity)
      `UartParityType_defaultEncoding_NONE : bridge_uartConfigReg_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : bridge_uartConfigReg_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : bridge_uartConfigReg_frame_parity_string = "ODD ";
      default : bridge_uartConfigReg_frame_parity_string = "????";
    endcase
  end
  `endif

  assign io_uart_txd = uartCtrl_1__io_uart_txd;
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = 32'h0;
    case(io_apb_PADDR)
      5'b00000 : begin
        io_apb_PRDATA[16 : 16] = (bridge_read_streamBreaked_valid ^ 1'b0);
        io_apb_PRDATA[7 : 0] = bridge_read_streamBreaked_payload;
      end
      5'b00100 : begin
        io_apb_PRDATA[20 : 16] = _zz_22_;
        io_apb_PRDATA[15 : 15] = bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
        io_apb_PRDATA[28 : 24] = uartCtrl_1__io_read_queueWithOccupancy_io_occupancy;
        io_apb_PRDATA[0 : 0] = bridge_interruptCtrl_writeIntEnable;
        io_apb_PRDATA[1 : 1] = bridge_interruptCtrl_readIntEnable;
        io_apb_PRDATA[8 : 8] = bridge_interruptCtrl_writeInt;
        io_apb_PRDATA[9 : 9] = bridge_interruptCtrl_readInt;
      end
      5'b10000 : begin
        io_apb_PRDATA[0 : 0] = bridge_misc_readError;
        io_apb_PRDATA[1 : 1] = bridge_misc_readOverflowError;
        io_apb_PRDATA[8 : 8] = uartCtrl_1__io_readBreak;
        io_apb_PRDATA[9 : 9] = bridge_misc_breakDetected;
      end
      default : begin
      end
    endcase
  end

  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  assign _zz_23_ = zz_bridge_uartConfigReg_clockDivider(1'b0);
  always @ (*) bridge_uartConfigReg_clockDivider = _zz_23_;
  assign bridge_uartConfigReg_frame_dataLength = (3'b111);
  assign bridge_uartConfigReg_frame_parity = `UartParityType_defaultEncoding_NONE;
  assign bridge_uartConfigReg_frame_stop = `UartStopType_defaultEncoding_ONE;
  always @ (*) begin
    _zz_1_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ = 1'b1;
        end
      end
      5'b00100 : begin
      end
      5'b10000 : begin
      end
      default : begin
      end
    endcase
  end

  assign bridge_write_streamUnbuffered_valid = _zz_1_;
  assign bridge_write_streamUnbuffered_payload = io_apb_PWDATA[7 : 0];
  assign bridge_write_streamUnbuffered_ready = bridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  always @ (*) begin
    bridge_read_streamBreaked_valid = uartCtrl_1__io_read_queueWithOccupancy_io_pop_valid;
    if(uartCtrl_1__io_readBreak)begin
      bridge_read_streamBreaked_valid = 1'b0;
    end
  end

  always @ (*) begin
    _zz_8_ = bridge_read_streamBreaked_ready;
    if(uartCtrl_1__io_readBreak)begin
      _zz_8_ = 1'b1;
    end
  end

  assign bridge_read_streamBreaked_payload = uartCtrl_1__io_read_queueWithOccupancy_io_pop_payload;
  always @ (*) begin
    bridge_read_streamBreaked_ready = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
        if(busCtrl_doRead)begin
          bridge_read_streamBreaked_ready = 1'b1;
        end
      end
      5'b00100 : begin
      end
      5'b10000 : begin
      end
      default : begin
      end
    endcase
  end

  assign bridge_interruptCtrl_readInt = (bridge_interruptCtrl_readIntEnable && bridge_read_streamBreaked_valid);
  assign bridge_interruptCtrl_writeInt = (bridge_interruptCtrl_writeIntEnable && (! bridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid));
  assign bridge_interruptCtrl_interrupt = (bridge_interruptCtrl_readInt || bridge_interruptCtrl_writeInt);
  always @ (*) begin
    _zz_2_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
      end
      5'b10000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_3_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
      end
      5'b10000 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
      end
      5'b10000 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_5_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
      end
      5'b10000 : begin
        if(busCtrl_doWrite)begin
          _zz_5_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_6_ = 1'b0;
    case(io_apb_PADDR)
      5'b00000 : begin
      end
      5'b00100 : begin
      end
      5'b10000 : begin
        if(busCtrl_doWrite)begin
          _zz_6_ = 1'b1;
        end
      end
      default : begin
      end
    endcase
  end

  assign io_interrupt = bridge_interruptCtrl_interrupt;
  assign _zz_7_ = 1'b0;
  assign _zz_9_ = 1'b0;
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      bridge_interruptCtrl_writeIntEnable <= 1'b0;
      bridge_interruptCtrl_readIntEnable <= 1'b0;
      bridge_misc_readError <= 1'b0;
      bridge_misc_readOverflowError <= 1'b0;
      bridge_misc_breakDetected <= 1'b0;
      bridge_misc_doBreak <= 1'b0;
    end else begin
      if(_zz_2_)begin
        if(_zz_10_[0])begin
          bridge_misc_readError <= _zz_11_[0];
        end
      end
      if(uartCtrl_1__io_readError)begin
        bridge_misc_readError <= 1'b1;
      end
      if(_zz_3_)begin
        if(_zz_12_[0])begin
          bridge_misc_readOverflowError <= _zz_13_[0];
        end
      end
      if((uartCtrl_1__io_read_valid && (! uartCtrl_1__io_read_queueWithOccupancy_io_push_ready)))begin
        bridge_misc_readOverflowError <= 1'b1;
      end
      if((uartCtrl_1__io_readBreak && (! uartCtrl_1__io_readBreak_regNext)))begin
        bridge_misc_breakDetected <= 1'b1;
      end
      if(_zz_4_)begin
        if(_zz_14_[0])begin
          bridge_misc_breakDetected <= _zz_15_[0];
        end
      end
      if(_zz_5_)begin
        if(_zz_16_[0])begin
          bridge_misc_doBreak <= _zz_17_[0];
        end
      end
      if(_zz_6_)begin
        if(_zz_18_[0])begin
          bridge_misc_doBreak <= _zz_19_[0];
        end
      end
      case(io_apb_PADDR)
        5'b00000 : begin
        end
        5'b00100 : begin
          if(busCtrl_doWrite)begin
            bridge_interruptCtrl_writeIntEnable <= _zz_20_[0];
            bridge_interruptCtrl_readIntEnable <= _zz_21_[0];
          end
        end
        5'b10000 : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_mainClk) begin
    uartCtrl_1__io_readBreak_regNext <= uartCtrl_1__io_readBreak;
  end


endmodule

module MuraxApb3Timer (
  input      [7:0]    io_apb_PADDR,
  input      [0:0]    io_apb_PSEL,
  input               io_apb_PENABLE,
  output              io_apb_PREADY,
  input               io_apb_PWRITE,
  input      [31:0]   io_apb_PWDATA,
  output reg [31:0]   io_apb_PRDATA,
  output              io_apb_PSLVERROR,
  output              io_interrupt,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  wire                _zz_7_;
  wire                _zz_8_;
  wire                _zz_9_;
  wire                _zz_10_;
  reg        [1:0]    _zz_11_;
  reg        [1:0]    _zz_12_;
  wire                prescaler_1__io_overflow;
  wire                timerA_io_full;
  wire       [15:0]   timerA_io_value;
  wire                timerB_io_full;
  wire       [15:0]   timerB_io_value;
  wire       [1:0]    interruptCtrl_1__io_pendings;
  wire                busCtrl_askWrite;
  wire                busCtrl_askRead;
  wire                busCtrl_doWrite;
  wire                busCtrl_doRead;
  reg        [15:0]   _zz_1_;
  reg                 _zz_2_;
  reg        [1:0]    timerABridge_ticksEnable;
  reg        [0:0]    timerABridge_clearsEnable;
  reg                 timerABridge_busClearing;
  reg        [15:0]   timerA_io_limit_driver;
  reg                 _zz_3_;
  reg                 _zz_4_;
  reg        [1:0]    timerBBridge_ticksEnable;
  reg        [0:0]    timerBBridge_clearsEnable;
  reg                 timerBBridge_busClearing;
  reg        [15:0]   timerB_io_limit_driver;
  reg                 _zz_5_;
  reg                 _zz_6_;
  reg        [1:0]    interruptCtrl_1__io_masks_driver;

  Prescaler prescaler_1_ ( 
    .io_clear                 (_zz_2_                    ), //i
    .io_limit                 (_zz_1_[15:0]              ), //i
    .io_overflow              (prescaler_1__io_overflow  ), //o
    .io_mainClk               (io_mainClk                ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset     )  //i
  );
  Timer timerA ( 
    .io_tick                  (_zz_7_                        ), //i
    .io_clear                 (_zz_8_                        ), //i
    .io_limit                 (timerA_io_limit_driver[15:0]  ), //i
    .io_full                  (timerA_io_full                ), //o
    .io_value                 (timerA_io_value[15:0]         ), //o
    .io_mainClk               (io_mainClk                    ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset         )  //i
  );
  Timer timerB ( 
    .io_tick                  (_zz_9_                        ), //i
    .io_clear                 (_zz_10_                       ), //i
    .io_limit                 (timerB_io_limit_driver[15:0]  ), //i
    .io_full                  (timerB_io_full                ), //o
    .io_value                 (timerB_io_value[15:0]         ), //o
    .io_mainClk               (io_mainClk                    ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset         )  //i
  );
  InterruptCtrl interruptCtrl_1_ ( 
    .io_inputs                (_zz_11_[1:0]                           ), //i
    .io_clears                (_zz_12_[1:0]                           ), //i
    .io_masks                 (interruptCtrl_1__io_masks_driver[1:0]  ), //i
    .io_pendings              (interruptCtrl_1__io_pendings[1:0]      ), //o
    .io_mainClk               (io_mainClk                             ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                  )  //i
  );
  assign io_apb_PREADY = 1'b1;
  always @ (*) begin
    io_apb_PRDATA = 32'h0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        io_apb_PRDATA[15 : 0] = _zz_1_;
      end
      8'b01000000 : begin
        io_apb_PRDATA[1 : 0] = timerABridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerABridge_clearsEnable;
      end
      8'b01000100 : begin
        io_apb_PRDATA[15 : 0] = timerA_io_limit_driver;
      end
      8'b01001000 : begin
        io_apb_PRDATA[15 : 0] = timerA_io_value;
      end
      8'b01010000 : begin
        io_apb_PRDATA[1 : 0] = timerBBridge_ticksEnable;
        io_apb_PRDATA[16 : 16] = timerBBridge_clearsEnable;
      end
      8'b01010100 : begin
        io_apb_PRDATA[15 : 0] = timerB_io_limit_driver;
      end
      8'b01011000 : begin
        io_apb_PRDATA[15 : 0] = timerB_io_value;
      end
      8'b00010000 : begin
        io_apb_PRDATA[1 : 0] = interruptCtrl_1__io_pendings;
      end
      8'b00010100 : begin
        io_apb_PRDATA[1 : 0] = interruptCtrl_1__io_masks_driver;
      end
      default : begin
      end
    endcase
  end

  assign io_apb_PSLVERROR = 1'b0;
  assign busCtrl_askWrite = ((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PWRITE);
  assign busCtrl_askRead = ((io_apb_PSEL[0] && io_apb_PENABLE) && (! io_apb_PWRITE));
  assign busCtrl_doWrite = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && io_apb_PWRITE);
  assign busCtrl_doRead = (((io_apb_PSEL[0] && io_apb_PENABLE) && io_apb_PREADY) && (! io_apb_PWRITE));
  always @ (*) begin
    _zz_2_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_2_ = 1'b1;
        end
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    timerABridge_busClearing = 1'b0;
    if(_zz_3_)begin
      timerABridge_busClearing = 1'b1;
    end
    if(_zz_4_)begin
      timerABridge_busClearing = 1'b1;
    end
  end

  always @ (*) begin
    _zz_3_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          _zz_3_ = 1'b1;
        end
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_4_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
      end
      8'b01001000 : begin
        if(busCtrl_doWrite)begin
          _zz_4_ = 1'b1;
        end
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_8_ = (((timerABridge_clearsEnable & timerA_io_full) != (1'b0)) || timerABridge_busClearing);
  assign _zz_7_ = ((timerABridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    timerBBridge_busClearing = 1'b0;
    if(_zz_5_)begin
      timerBBridge_busClearing = 1'b1;
    end
    if(_zz_6_)begin
      timerBBridge_busClearing = 1'b1;
    end
  end

  always @ (*) begin
    _zz_5_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          _zz_5_ = 1'b1;
        end
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_6_ = 1'b0;
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
        if(busCtrl_doWrite)begin
          _zz_6_ = 1'b1;
        end
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  assign _zz_10_ = (((timerBBridge_clearsEnable & timerB_io_full) != (1'b0)) || timerBBridge_busClearing);
  assign _zz_9_ = ((timerBBridge_ticksEnable & {prescaler_1__io_overflow,1'b1}) != (2'b00));
  always @ (*) begin
    _zz_12_ = (2'b00);
    case(io_apb_PADDR)
      8'b00000000 : begin
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
        if(busCtrl_doWrite)begin
          _zz_12_ = io_apb_PWDATA[1 : 0];
        end
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_11_[0] = timerA_io_full;
    _zz_11_[1] = timerB_io_full;
  end

  assign io_interrupt = (interruptCtrl_1__io_pendings != (2'b00));
  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      timerABridge_ticksEnable <= (2'b00);
      timerABridge_clearsEnable <= (1'b0);
      timerBBridge_ticksEnable <= (2'b00);
      timerBBridge_clearsEnable <= (1'b0);
      interruptCtrl_1__io_masks_driver <= (2'b00);
    end else begin
      case(io_apb_PADDR)
        8'b00000000 : begin
        end
        8'b01000000 : begin
          if(busCtrl_doWrite)begin
            timerABridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerABridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01000100 : begin
        end
        8'b01001000 : begin
        end
        8'b01010000 : begin
          if(busCtrl_doWrite)begin
            timerBBridge_ticksEnable <= io_apb_PWDATA[1 : 0];
            timerBBridge_clearsEnable <= io_apb_PWDATA[16 : 16];
          end
        end
        8'b01010100 : begin
        end
        8'b01011000 : begin
        end
        8'b00010000 : begin
        end
        8'b00010100 : begin
          if(busCtrl_doWrite)begin
            interruptCtrl_1__io_masks_driver <= io_apb_PWDATA[1 : 0];
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge io_mainClk) begin
    case(io_apb_PADDR)
      8'b00000000 : begin
        if(busCtrl_doWrite)begin
          _zz_1_ <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01000000 : begin
      end
      8'b01000100 : begin
        if(busCtrl_doWrite)begin
          timerA_io_limit_driver <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01001000 : begin
      end
      8'b01010000 : begin
      end
      8'b01010100 : begin
        if(busCtrl_doWrite)begin
          timerB_io_limit_driver <= io_apb_PWDATA[15 : 0];
        end
      end
      8'b01011000 : begin
      end
      8'b00010000 : begin
      end
      8'b00010100 : begin
      end
      default : begin
      end
    endcase
  end


endmodule

module Apb3Decoder (
  input      [19:0]   io_input_PADDR,
  input      [0:0]    io_input_PSEL,
  input               io_input_PENABLE,
  output reg          io_input_PREADY,
  input               io_input_PWRITE,
  input      [31:0]   io_input_PWDATA,
  output     [31:0]   io_input_PRDATA,
  output reg          io_input_PSLVERROR,
  output     [19:0]   io_output_PADDR,
  output reg [2:0]    io_output_PSEL,
  output              io_output_PENABLE,
  input               io_output_PREADY,
  output              io_output_PWRITE,
  output     [31:0]   io_output_PWDATA,
  input      [31:0]   io_output_PRDATA,
  input               io_output_PSLVERROR 
);
  wire                _zz_1_;

  assign _zz_1_ = (io_input_PSEL[0] && (io_output_PSEL == (3'b000)));
  assign io_output_PADDR = io_input_PADDR;
  assign io_output_PENABLE = io_input_PENABLE;
  assign io_output_PWRITE = io_input_PWRITE;
  assign io_output_PWDATA = io_input_PWDATA;
  always @ (*) begin
    io_output_PSEL[0] = (((io_input_PADDR & (~ 20'h00fff)) == 20'h0) && io_input_PSEL[0]);
    io_output_PSEL[1] = (((io_input_PADDR & (~ 20'h00fff)) == 20'h10000) && io_input_PSEL[0]);
    io_output_PSEL[2] = (((io_input_PADDR & (~ 20'h00fff)) == 20'h20000) && io_input_PSEL[0]);
  end

  always @ (*) begin
    io_input_PREADY = io_output_PREADY;
    if(_zz_1_)begin
      io_input_PREADY = 1'b1;
    end
  end

  assign io_input_PRDATA = io_output_PRDATA;
  always @ (*) begin
    io_input_PSLVERROR = io_output_PSLVERROR;
    if(_zz_1_)begin
      io_input_PSLVERROR = 1'b1;
    end
  end


endmodule

module Apb3Router (
  input      [19:0]   io_input_PADDR,
  input      [2:0]    io_input_PSEL,
  input               io_input_PENABLE,
  output              io_input_PREADY,
  input               io_input_PWRITE,
  input      [31:0]   io_input_PWDATA,
  output     [31:0]   io_input_PRDATA,
  output              io_input_PSLVERROR,
  output     [19:0]   io_outputs_0_PADDR,
  output     [0:0]    io_outputs_0_PSEL,
  output              io_outputs_0_PENABLE,
  input               io_outputs_0_PREADY,
  output              io_outputs_0_PWRITE,
  output     [31:0]   io_outputs_0_PWDATA,
  input      [31:0]   io_outputs_0_PRDATA,
  input               io_outputs_0_PSLVERROR,
  output     [19:0]   io_outputs_1_PADDR,
  output     [0:0]    io_outputs_1_PSEL,
  output              io_outputs_1_PENABLE,
  input               io_outputs_1_PREADY,
  output              io_outputs_1_PWRITE,
  output     [31:0]   io_outputs_1_PWDATA,
  input      [31:0]   io_outputs_1_PRDATA,
  input               io_outputs_1_PSLVERROR,
  output     [19:0]   io_outputs_2_PADDR,
  output     [0:0]    io_outputs_2_PSEL,
  output              io_outputs_2_PENABLE,
  input               io_outputs_2_PREADY,
  output              io_outputs_2_PWRITE,
  output     [31:0]   io_outputs_2_PWDATA,
  input      [31:0]   io_outputs_2_PRDATA,
  input               io_outputs_2_PSLVERROR,
  input               io_mainClk,
  input               resetCtrl_systemReset 
);
  reg                 _zz_3_;
  reg        [31:0]   _zz_4_;
  reg                 _zz_5_;
  wire                _zz_1_;
  wire                _zz_2_;
  reg        [1:0]    selIndex;

  always @(*) begin
    case(selIndex)
      2'b00 : begin
        _zz_3_ = io_outputs_0_PREADY;
        _zz_4_ = io_outputs_0_PRDATA;
        _zz_5_ = io_outputs_0_PSLVERROR;
      end
      2'b01 : begin
        _zz_3_ = io_outputs_1_PREADY;
        _zz_4_ = io_outputs_1_PRDATA;
        _zz_5_ = io_outputs_1_PSLVERROR;
      end
      default : begin
        _zz_3_ = io_outputs_2_PREADY;
        _zz_4_ = io_outputs_2_PRDATA;
        _zz_5_ = io_outputs_2_PSLVERROR;
      end
    endcase
  end

  assign io_outputs_0_PADDR = io_input_PADDR;
  assign io_outputs_0_PENABLE = io_input_PENABLE;
  assign io_outputs_0_PSEL[0] = io_input_PSEL[0];
  assign io_outputs_0_PWRITE = io_input_PWRITE;
  assign io_outputs_0_PWDATA = io_input_PWDATA;
  assign io_outputs_1_PADDR = io_input_PADDR;
  assign io_outputs_1_PENABLE = io_input_PENABLE;
  assign io_outputs_1_PSEL[0] = io_input_PSEL[1];
  assign io_outputs_1_PWRITE = io_input_PWRITE;
  assign io_outputs_1_PWDATA = io_input_PWDATA;
  assign io_outputs_2_PADDR = io_input_PADDR;
  assign io_outputs_2_PENABLE = io_input_PENABLE;
  assign io_outputs_2_PSEL[0] = io_input_PSEL[2];
  assign io_outputs_2_PWRITE = io_input_PWRITE;
  assign io_outputs_2_PWDATA = io_input_PWDATA;
  assign _zz_1_ = io_input_PSEL[1];
  assign _zz_2_ = io_input_PSEL[2];
  assign io_input_PREADY = _zz_3_;
  assign io_input_PRDATA = _zz_4_;
  assign io_input_PSLVERROR = _zz_5_;
  always @ (posedge io_mainClk) begin
    selIndex <= {_zz_2_,_zz_1_};
  end


endmodule

module Murax (
  input               io_asyncReset,
  input               io_mainClk,
  input               io_jtag_tms,
  input               io_jtag_tdi,
  output              io_jtag_tdo,
  input               io_jtag_tck,
  input      [31:0]   io_gpioA_read,
  output     [31:0]   io_gpioA_write,
  output     [31:0]   io_gpioA_writeEnable,
  output              io_uart_txd,
  input               io_uart_rxd 
);
  wire                _zz_5_;
  wire       [7:0]    _zz_6_;
  reg                 _zz_7_;
  reg                 _zz_8_;
  wire       [3:0]    _zz_9_;
  wire       [4:0]    _zz_10_;
  wire       [7:0]    _zz_11_;
  wire                _zz_12_;
  reg        [31:0]   _zz_13_;
  wire                io_asyncReset_buffercc_io_dataOut;
  wire                system_mainBusArbiter_io_iBus_cmd_ready;
  wire                system_mainBusArbiter_io_iBus_rsp_valid;
  wire                system_mainBusArbiter_io_iBus_rsp_payload_error;
  wire       [31:0]   system_mainBusArbiter_io_iBus_rsp_payload_inst;
  wire                system_mainBusArbiter_io_dBus_cmd_ready;
  wire                system_mainBusArbiter_io_dBus_rsp_ready;
  wire                system_mainBusArbiter_io_dBus_rsp_error;
  wire       [31:0]   system_mainBusArbiter_io_dBus_rsp_data;
  wire                system_mainBusArbiter_io_masterBus_cmd_valid;
  wire                system_mainBusArbiter_io_masterBus_cmd_payload_write;
  wire       [31:0]   system_mainBusArbiter_io_masterBus_cmd_payload_address;
  wire       [31:0]   system_mainBusArbiter_io_masterBus_cmd_payload_data;
  wire       [3:0]    system_mainBusArbiter_io_masterBus_cmd_payload_mask;
  wire                system_cpu_iBus_cmd_valid;
  wire       [31:0]   system_cpu_iBus_cmd_payload_pc;
  wire                system_cpu_debug_bus_cmd_ready;
  wire       [31:0]   system_cpu_debug_bus_rsp_data;
  wire                system_cpu_debug_resetOut;
  wire                system_cpu_dBus_cmd_valid;
  wire                system_cpu_dBus_cmd_payload_wr;
  wire       [31:0]   system_cpu_dBus_cmd_payload_address;
  wire       [31:0]   system_cpu_dBus_cmd_payload_data;
  wire       [1:0]    system_cpu_dBus_cmd_payload_size;
  wire                jtagBridge_1__io_jtag_tdo;
  wire                jtagBridge_1__io_remote_cmd_valid;
  wire                jtagBridge_1__io_remote_cmd_payload_last;
  wire       [0:0]    jtagBridge_1__io_remote_cmd_payload_fragment;
  wire                jtagBridge_1__io_remote_rsp_ready;
  wire                systemDebugger_1__io_remote_cmd_ready;
  wire                systemDebugger_1__io_remote_rsp_valid;
  wire                systemDebugger_1__io_remote_rsp_payload_error;
  wire       [31:0]   systemDebugger_1__io_remote_rsp_payload_data;
  wire                systemDebugger_1__io_mem_cmd_valid;
  wire       [31:0]   systemDebugger_1__io_mem_cmd_payload_address;
  wire       [31:0]   systemDebugger_1__io_mem_cmd_payload_data;
  wire                systemDebugger_1__io_mem_cmd_payload_wr;
  wire       [1:0]    systemDebugger_1__io_mem_cmd_payload_size;
  wire                system_ram_io_bus_cmd_ready;
  wire                system_ram_io_bus_rsp_valid;
  wire       [31:0]   system_ram_io_bus_rsp_payload_data;
  wire                system_apbBridge_io_pipelinedMemoryBus_cmd_ready;
  wire                system_apbBridge_io_pipelinedMemoryBus_rsp_valid;
  wire       [31:0]   system_apbBridge_io_pipelinedMemoryBus_rsp_payload_data;
  wire       [19:0]   system_apbBridge_io_apb_PADDR;
  wire       [0:0]    system_apbBridge_io_apb_PSEL;
  wire                system_apbBridge_io_apb_PENABLE;
  wire                system_apbBridge_io_apb_PWRITE;
  wire       [31:0]   system_apbBridge_io_apb_PWDATA;
  wire                system_gpioACtrl_io_apb_PREADY;
  wire       [31:0]   system_gpioACtrl_io_apb_PRDATA;
  wire                system_gpioACtrl_io_apb_PSLVERROR;
  wire       [31:0]   system_gpioACtrl_io_gpio_write;
  wire       [31:0]   system_gpioACtrl_io_gpio_writeEnable;
  wire       [31:0]   system_gpioACtrl_io_value;
  wire                system_uartCtrl_io_apb_PREADY;
  wire       [31:0]   system_uartCtrl_io_apb_PRDATA;
  wire                system_uartCtrl_io_uart_txd;
  wire                system_uartCtrl_io_interrupt;
  wire                system_timer_io_apb_PREADY;
  wire       [31:0]   system_timer_io_apb_PRDATA;
  wire                system_timer_io_apb_PSLVERROR;
  wire                system_timer_io_interrupt;
  wire                io_apb_decoder_io_input_PREADY;
  wire       [31:0]   io_apb_decoder_io_input_PRDATA;
  wire                io_apb_decoder_io_input_PSLVERROR;
  wire       [19:0]   io_apb_decoder_io_output_PADDR;
  wire       [2:0]    io_apb_decoder_io_output_PSEL;
  wire                io_apb_decoder_io_output_PENABLE;
  wire                io_apb_decoder_io_output_PWRITE;
  wire       [31:0]   io_apb_decoder_io_output_PWDATA;
  wire                apb3Router_1__io_input_PREADY;
  wire       [31:0]   apb3Router_1__io_input_PRDATA;
  wire                apb3Router_1__io_input_PSLVERROR;
  wire       [19:0]   apb3Router_1__io_outputs_0_PADDR;
  wire       [0:0]    apb3Router_1__io_outputs_0_PSEL;
  wire                apb3Router_1__io_outputs_0_PENABLE;
  wire                apb3Router_1__io_outputs_0_PWRITE;
  wire       [31:0]   apb3Router_1__io_outputs_0_PWDATA;
  wire       [19:0]   apb3Router_1__io_outputs_1_PADDR;
  wire       [0:0]    apb3Router_1__io_outputs_1_PSEL;
  wire                apb3Router_1__io_outputs_1_PENABLE;
  wire                apb3Router_1__io_outputs_1_PWRITE;
  wire       [31:0]   apb3Router_1__io_outputs_1_PWDATA;
  wire       [19:0]   apb3Router_1__io_outputs_2_PADDR;
  wire       [0:0]    apb3Router_1__io_outputs_2_PSEL;
  wire                apb3Router_1__io_outputs_2_PENABLE;
  wire                apb3Router_1__io_outputs_2_PWRITE;
  wire       [31:0]   apb3Router_1__io_outputs_2_PWDATA;
  wire                _zz_14_;
  wire                _zz_15_;
  wire                _zz_16_;
  reg                 resetCtrl_mainClkResetUnbuffered;
  reg        [5:0]    resetCtrl_systemClkResetCounter = 6'h0;
  wire       [5:0]    _zz_1_;
  reg                 resetCtrl_mainClkReset;
  reg                 resetCtrl_systemReset;
  reg                 system_timerInterrupt;
  reg                 system_externalInterrupt;
  wire                system_cpu_dBus_cmd_halfPipe_valid;
  wire                system_cpu_dBus_cmd_halfPipe_ready;
  wire                system_cpu_dBus_cmd_halfPipe_payload_wr;
  wire       [31:0]   system_cpu_dBus_cmd_halfPipe_payload_address;
  wire       [31:0]   system_cpu_dBus_cmd_halfPipe_payload_data;
  wire       [1:0]    system_cpu_dBus_cmd_halfPipe_payload_size;
  reg                 system_cpu_dBus_cmd_halfPipe_regs_valid;
  reg                 system_cpu_dBus_cmd_halfPipe_regs_ready;
  reg                 system_cpu_dBus_cmd_halfPipe_regs_payload_wr;
  reg        [31:0]   system_cpu_dBus_cmd_halfPipe_regs_payload_address;
  reg        [31:0]   system_cpu_dBus_cmd_halfPipe_regs_payload_data;
  reg        [1:0]    system_cpu_dBus_cmd_halfPipe_regs_payload_size;
  reg                 system_cpu_debug_resetOut_regNext;
  reg                 _zz_2_;
  wire                system_mainBusDecoder_logic_masterPipelined_cmd_valid;
  reg                 system_mainBusDecoder_logic_masterPipelined_cmd_ready;
  wire                system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  wire       [31:0]   system_mainBusDecoder_logic_masterPipelined_cmd_payload_address;
  wire       [31:0]   system_mainBusDecoder_logic_masterPipelined_cmd_payload_data;
  wire       [3:0]    system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask;
  wire                system_mainBusDecoder_logic_masterPipelined_rsp_valid;
  wire       [31:0]   system_mainBusDecoder_logic_masterPipelined_rsp_payload_data;
  wire                system_mainBusDecoder_logic_hits_0;
  wire                _zz_3_;
  wire                system_mainBusDecoder_logic_hits_1;
  wire                _zz_4_;
  wire                system_mainBusDecoder_logic_noHit;
  reg                 system_mainBusDecoder_logic_rspPending;
  reg                 system_mainBusDecoder_logic_rspNoHit;
  reg        [0:0]    system_mainBusDecoder_logic_rspSourceId;

  assign _zz_14_ = (resetCtrl_systemClkResetCounter != _zz_1_);
  assign _zz_15_ = (system_mainBusDecoder_logic_rspPending && (! system_mainBusDecoder_logic_masterPipelined_rsp_valid));
  assign _zz_16_ = (! system_cpu_dBus_cmd_halfPipe_regs_valid);
  BufferCC_3_ io_asyncReset_buffercc ( 
    .io_dataIn     (io_asyncReset                      ), //i
    .io_dataOut    (io_asyncReset_buffercc_io_dataOut  ), //o
    .io_mainClk    (io_mainClk                         )  //i
  );
  MuraxMasterArbiter system_mainBusArbiter ( 
    .io_iBus_cmd_valid                   (system_cpu_iBus_cmd_valid                                           ), //i
    .io_iBus_cmd_ready                   (system_mainBusArbiter_io_iBus_cmd_ready                             ), //o
    .io_iBus_cmd_payload_pc              (system_cpu_iBus_cmd_payload_pc[31:0]                                ), //i
    .io_iBus_rsp_valid                   (system_mainBusArbiter_io_iBus_rsp_valid                             ), //o
    .io_iBus_rsp_payload_error           (system_mainBusArbiter_io_iBus_rsp_payload_error                     ), //o
    .io_iBus_rsp_payload_inst            (system_mainBusArbiter_io_iBus_rsp_payload_inst[31:0]                ), //o
    .io_dBus_cmd_valid                   (system_cpu_dBus_cmd_halfPipe_valid                                  ), //i
    .io_dBus_cmd_ready                   (system_mainBusArbiter_io_dBus_cmd_ready                             ), //o
    .io_dBus_cmd_payload_wr              (system_cpu_dBus_cmd_halfPipe_payload_wr                             ), //i
    .io_dBus_cmd_payload_address         (system_cpu_dBus_cmd_halfPipe_payload_address[31:0]                  ), //i
    .io_dBus_cmd_payload_data            (system_cpu_dBus_cmd_halfPipe_payload_data[31:0]                     ), //i
    .io_dBus_cmd_payload_size            (system_cpu_dBus_cmd_halfPipe_payload_size[1:0]                      ), //i
    .io_dBus_rsp_ready                   (system_mainBusArbiter_io_dBus_rsp_ready                             ), //o
    .io_dBus_rsp_error                   (system_mainBusArbiter_io_dBus_rsp_error                             ), //o
    .io_dBus_rsp_data                    (system_mainBusArbiter_io_dBus_rsp_data[31:0]                        ), //o
    .io_masterBus_cmd_valid              (system_mainBusArbiter_io_masterBus_cmd_valid                        ), //o
    .io_masterBus_cmd_ready              (system_mainBusDecoder_logic_masterPipelined_cmd_ready               ), //i
    .io_masterBus_cmd_payload_write      (system_mainBusArbiter_io_masterBus_cmd_payload_write                ), //o
    .io_masterBus_cmd_payload_address    (system_mainBusArbiter_io_masterBus_cmd_payload_address[31:0]        ), //o
    .io_masterBus_cmd_payload_data       (system_mainBusArbiter_io_masterBus_cmd_payload_data[31:0]           ), //o
    .io_masterBus_cmd_payload_mask       (system_mainBusArbiter_io_masterBus_cmd_payload_mask[3:0]            ), //o
    .io_masterBus_rsp_valid              (system_mainBusDecoder_logic_masterPipelined_rsp_valid               ), //i
    .io_masterBus_rsp_payload_data       (system_mainBusDecoder_logic_masterPipelined_rsp_payload_data[31:0]  ), //i
    .io_mainClk                          (io_mainClk                                                          ), //i
    .resetCtrl_systemReset               (resetCtrl_systemReset                                               )  //i
  );
  VexRiscv system_cpu ( 
    .iBus_cmd_valid                   (system_cpu_iBus_cmd_valid                             ), //o
    .iBus_cmd_ready                   (system_mainBusArbiter_io_iBus_cmd_ready               ), //i
    .iBus_cmd_payload_pc              (system_cpu_iBus_cmd_payload_pc[31:0]                  ), //o
    .iBus_rsp_valid                   (system_mainBusArbiter_io_iBus_rsp_valid               ), //i
    .iBus_rsp_payload_error           (system_mainBusArbiter_io_iBus_rsp_payload_error       ), //i
    .iBus_rsp_payload_inst            (system_mainBusArbiter_io_iBus_rsp_payload_inst[31:0]  ), //i
    .timerInterrupt                   (system_timerInterrupt                                 ), //i
    .externalInterrupt                (system_externalInterrupt                              ), //i
    .softwareInterrupt                (_zz_5_                                                ), //i
    .debug_bus_cmd_valid              (systemDebugger_1__io_mem_cmd_valid                    ), //i
    .debug_bus_cmd_ready              (system_cpu_debug_bus_cmd_ready                        ), //o
    .debug_bus_cmd_payload_wr         (systemDebugger_1__io_mem_cmd_payload_wr               ), //i
    .debug_bus_cmd_payload_address    (_zz_6_[7:0]                                           ), //i
    .debug_bus_cmd_payload_data       (systemDebugger_1__io_mem_cmd_payload_data[31:0]       ), //i
    .debug_bus_rsp_data               (system_cpu_debug_bus_rsp_data[31:0]                   ), //o
    .debug_resetOut                   (system_cpu_debug_resetOut                             ), //o
    .dBus_cmd_valid                   (system_cpu_dBus_cmd_valid                             ), //o
    .dBus_cmd_ready                   (system_cpu_dBus_cmd_halfPipe_regs_ready               ), //i
    .dBus_cmd_payload_wr              (system_cpu_dBus_cmd_payload_wr                        ), //o
    .dBus_cmd_payload_address         (system_cpu_dBus_cmd_payload_address[31:0]             ), //o
    .dBus_cmd_payload_data            (system_cpu_dBus_cmd_payload_data[31:0]                ), //o
    .dBus_cmd_payload_size            (system_cpu_dBus_cmd_payload_size[1:0]                 ), //o
    .dBus_rsp_ready                   (system_mainBusArbiter_io_dBus_rsp_ready               ), //i
    .dBus_rsp_error                   (system_mainBusArbiter_io_dBus_rsp_error               ), //i
    .dBus_rsp_data                    (system_mainBusArbiter_io_dBus_rsp_data[31:0]          ), //i
    .io_mainClk                       (io_mainClk                                            ), //i
    .resetCtrl_systemReset            (resetCtrl_systemReset                                 ), //i
    .resetCtrl_mainClkReset           (resetCtrl_mainClkReset                                )  //i
  );
  JtagBridge jtagBridge_1_ ( 
    .io_jtag_tms                       (io_jtag_tms                                         ), //i
    .io_jtag_tdi                       (io_jtag_tdi                                         ), //i
    .io_jtag_tdo                       (jtagBridge_1__io_jtag_tdo                           ), //o
    .io_jtag_tck                       (io_jtag_tck                                         ), //i
    .io_remote_cmd_valid               (jtagBridge_1__io_remote_cmd_valid                   ), //o
    .io_remote_cmd_ready               (systemDebugger_1__io_remote_cmd_ready               ), //i
    .io_remote_cmd_payload_last        (jtagBridge_1__io_remote_cmd_payload_last            ), //o
    .io_remote_cmd_payload_fragment    (jtagBridge_1__io_remote_cmd_payload_fragment        ), //o
    .io_remote_rsp_valid               (systemDebugger_1__io_remote_rsp_valid               ), //i
    .io_remote_rsp_ready               (jtagBridge_1__io_remote_rsp_ready                   ), //o
    .io_remote_rsp_payload_error       (systemDebugger_1__io_remote_rsp_payload_error       ), //i
    .io_remote_rsp_payload_data        (systemDebugger_1__io_remote_rsp_payload_data[31:0]  ), //i
    .io_mainClk                        (io_mainClk                                          ), //i
    .resetCtrl_mainClkReset            (resetCtrl_mainClkReset                              )  //i
  );
  SystemDebugger systemDebugger_1_ ( 
    .io_remote_cmd_valid               (jtagBridge_1__io_remote_cmd_valid                   ), //i
    .io_remote_cmd_ready               (systemDebugger_1__io_remote_cmd_ready               ), //o
    .io_remote_cmd_payload_last        (jtagBridge_1__io_remote_cmd_payload_last            ), //i
    .io_remote_cmd_payload_fragment    (jtagBridge_1__io_remote_cmd_payload_fragment        ), //i
    .io_remote_rsp_valid               (systemDebugger_1__io_remote_rsp_valid               ), //o
    .io_remote_rsp_ready               (jtagBridge_1__io_remote_rsp_ready                   ), //i
    .io_remote_rsp_payload_error       (systemDebugger_1__io_remote_rsp_payload_error       ), //o
    .io_remote_rsp_payload_data        (systemDebugger_1__io_remote_rsp_payload_data[31:0]  ), //o
    .io_mem_cmd_valid                  (systemDebugger_1__io_mem_cmd_valid                  ), //o
    .io_mem_cmd_ready                  (system_cpu_debug_bus_cmd_ready                      ), //i
    .io_mem_cmd_payload_address        (systemDebugger_1__io_mem_cmd_payload_address[31:0]  ), //o
    .io_mem_cmd_payload_data           (systemDebugger_1__io_mem_cmd_payload_data[31:0]     ), //o
    .io_mem_cmd_payload_wr             (systemDebugger_1__io_mem_cmd_payload_wr             ), //o
    .io_mem_cmd_payload_size           (systemDebugger_1__io_mem_cmd_payload_size[1:0]      ), //o
    .io_mem_rsp_valid                  (_zz_2_                                              ), //i
    .io_mem_rsp_payload                (system_cpu_debug_bus_rsp_data[31:0]                 ), //i
    .io_mainClk                        (io_mainClk                                          ), //i
    .resetCtrl_mainClkReset            (resetCtrl_mainClkReset                              )  //i
  );
  MuraxPipelinedMemoryBusRam system_ram ( 
    .io_bus_cmd_valid              (_zz_7_                                                                 ), //i
    .io_bus_cmd_ready              (system_ram_io_bus_cmd_ready                                            ), //o
    .io_bus_cmd_payload_write      (_zz_3_                                                                 ), //i
    .io_bus_cmd_payload_address    (system_mainBusDecoder_logic_masterPipelined_cmd_payload_address[31:0]  ), //i
    .io_bus_cmd_payload_data       (system_mainBusDecoder_logic_masterPipelined_cmd_payload_data[31:0]     ), //i
    .io_bus_cmd_payload_mask       (system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask[3:0]      ), //i
    .io_bus_rsp_valid              (system_ram_io_bus_rsp_valid                                            ), //o
    .io_bus_rsp_payload_data       (system_ram_io_bus_rsp_payload_data[31:0]                               ), //o
    .io_mainClk                    (io_mainClk                                                             ), //i
    .resetCtrl_systemReset         (resetCtrl_systemReset                                                  )  //i
  );
  PipelinedMemoryBusToApbBridge system_apbBridge ( 
    .io_pipelinedMemoryBus_cmd_valid              (_zz_8_                                                                 ), //i
    .io_pipelinedMemoryBus_cmd_ready              (system_apbBridge_io_pipelinedMemoryBus_cmd_ready                       ), //o
    .io_pipelinedMemoryBus_cmd_payload_write      (_zz_4_                                                                 ), //i
    .io_pipelinedMemoryBus_cmd_payload_address    (system_mainBusDecoder_logic_masterPipelined_cmd_payload_address[31:0]  ), //i
    .io_pipelinedMemoryBus_cmd_payload_data       (system_mainBusDecoder_logic_masterPipelined_cmd_payload_data[31:0]     ), //i
    .io_pipelinedMemoryBus_cmd_payload_mask       (system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask[3:0]      ), //i
    .io_pipelinedMemoryBus_rsp_valid              (system_apbBridge_io_pipelinedMemoryBus_rsp_valid                       ), //o
    .io_pipelinedMemoryBus_rsp_payload_data       (system_apbBridge_io_pipelinedMemoryBus_rsp_payload_data[31:0]          ), //o
    .io_apb_PADDR                                 (system_apbBridge_io_apb_PADDR[19:0]                                    ), //o
    .io_apb_PSEL                                  (system_apbBridge_io_apb_PSEL                                           ), //o
    .io_apb_PENABLE                               (system_apbBridge_io_apb_PENABLE                                        ), //o
    .io_apb_PREADY                                (io_apb_decoder_io_input_PREADY                                         ), //i
    .io_apb_PWRITE                                (system_apbBridge_io_apb_PWRITE                                         ), //o
    .io_apb_PWDATA                                (system_apbBridge_io_apb_PWDATA[31:0]                                   ), //o
    .io_apb_PRDATA                                (io_apb_decoder_io_input_PRDATA[31:0]                                   ), //i
    .io_apb_PSLVERROR                             (io_apb_decoder_io_input_PSLVERROR                                      ), //i
    .io_mainClk                                   (io_mainClk                                                             ), //i
    .resetCtrl_systemReset                        (resetCtrl_systemReset                                                  )  //i
  );
  Apb3Gpio system_gpioACtrl ( 
    .io_apb_PADDR             (_zz_9_[3:0]                                 ), //i
    .io_apb_PSEL              (apb3Router_1__io_outputs_0_PSEL             ), //i
    .io_apb_PENABLE           (apb3Router_1__io_outputs_0_PENABLE          ), //i
    .io_apb_PREADY            (system_gpioACtrl_io_apb_PREADY              ), //o
    .io_apb_PWRITE            (apb3Router_1__io_outputs_0_PWRITE           ), //i
    .io_apb_PWDATA            (apb3Router_1__io_outputs_0_PWDATA[31:0]     ), //i
    .io_apb_PRDATA            (system_gpioACtrl_io_apb_PRDATA[31:0]        ), //o
    .io_apb_PSLVERROR         (system_gpioACtrl_io_apb_PSLVERROR           ), //o
    .io_gpio_read             (io_gpioA_read[31:0]                         ), //i
    .io_gpio_write            (system_gpioACtrl_io_gpio_write[31:0]        ), //o
    .io_gpio_writeEnable      (system_gpioACtrl_io_gpio_writeEnable[31:0]  ), //o
    .io_value                 (system_gpioACtrl_io_value[31:0]             ), //o
    .io_mainClk               (io_mainClk                                  ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                       )  //i
  );
  Apb3UartCtrl system_uartCtrl ( 
    .io_apb_PADDR             (_zz_10_[4:0]                             ), //i
    .io_apb_PSEL              (apb3Router_1__io_outputs_1_PSEL          ), //i
    .io_apb_PENABLE           (apb3Router_1__io_outputs_1_PENABLE       ), //i
    .io_apb_PREADY            (system_uartCtrl_io_apb_PREADY            ), //o
    .io_apb_PWRITE            (apb3Router_1__io_outputs_1_PWRITE        ), //i
    .io_apb_PWDATA            (apb3Router_1__io_outputs_1_PWDATA[31:0]  ), //i
    .io_apb_PRDATA            (system_uartCtrl_io_apb_PRDATA[31:0]      ), //o
    .io_uart_txd              (system_uartCtrl_io_uart_txd              ), //o
    .io_uart_rxd              (io_uart_rxd                              ), //i
    .io_interrupt             (system_uartCtrl_io_interrupt             ), //o
    .io_mainClk               (io_mainClk                               ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                    )  //i
  );
  MuraxApb3Timer system_timer ( 
    .io_apb_PADDR             (_zz_11_[7:0]                             ), //i
    .io_apb_PSEL              (apb3Router_1__io_outputs_2_PSEL          ), //i
    .io_apb_PENABLE           (apb3Router_1__io_outputs_2_PENABLE       ), //i
    .io_apb_PREADY            (system_timer_io_apb_PREADY               ), //o
    .io_apb_PWRITE            (apb3Router_1__io_outputs_2_PWRITE        ), //i
    .io_apb_PWDATA            (apb3Router_1__io_outputs_2_PWDATA[31:0]  ), //i
    .io_apb_PRDATA            (system_timer_io_apb_PRDATA[31:0]         ), //o
    .io_apb_PSLVERROR         (system_timer_io_apb_PSLVERROR            ), //o
    .io_interrupt             (system_timer_io_interrupt                ), //o
    .io_mainClk               (io_mainClk                               ), //i
    .resetCtrl_systemReset    (resetCtrl_systemReset                    )  //i
  );
  Apb3Decoder io_apb_decoder ( 
    .io_input_PADDR         (system_apbBridge_io_apb_PADDR[19:0]    ), //i
    .io_input_PSEL          (system_apbBridge_io_apb_PSEL           ), //i
    .io_input_PENABLE       (system_apbBridge_io_apb_PENABLE        ), //i
    .io_input_PREADY        (io_apb_decoder_io_input_PREADY         ), //o
    .io_input_PWRITE        (system_apbBridge_io_apb_PWRITE         ), //i
    .io_input_PWDATA        (system_apbBridge_io_apb_PWDATA[31:0]   ), //i
    .io_input_PRDATA        (io_apb_decoder_io_input_PRDATA[31:0]   ), //o
    .io_input_PSLVERROR     (io_apb_decoder_io_input_PSLVERROR      ), //o
    .io_output_PADDR        (io_apb_decoder_io_output_PADDR[19:0]   ), //o
    .io_output_PSEL         (io_apb_decoder_io_output_PSEL[2:0]     ), //o
    .io_output_PENABLE      (io_apb_decoder_io_output_PENABLE       ), //o
    .io_output_PREADY       (apb3Router_1__io_input_PREADY          ), //i
    .io_output_PWRITE       (io_apb_decoder_io_output_PWRITE        ), //o
    .io_output_PWDATA       (io_apb_decoder_io_output_PWDATA[31:0]  ), //o
    .io_output_PRDATA       (apb3Router_1__io_input_PRDATA[31:0]    ), //i
    .io_output_PSLVERROR    (apb3Router_1__io_input_PSLVERROR       )  //i
  );
  Apb3Router apb3Router_1_ ( 
    .io_input_PADDR            (io_apb_decoder_io_output_PADDR[19:0]     ), //i
    .io_input_PSEL             (io_apb_decoder_io_output_PSEL[2:0]       ), //i
    .io_input_PENABLE          (io_apb_decoder_io_output_PENABLE         ), //i
    .io_input_PREADY           (apb3Router_1__io_input_PREADY            ), //o
    .io_input_PWRITE           (io_apb_decoder_io_output_PWRITE          ), //i
    .io_input_PWDATA           (io_apb_decoder_io_output_PWDATA[31:0]    ), //i
    .io_input_PRDATA           (apb3Router_1__io_input_PRDATA[31:0]      ), //o
    .io_input_PSLVERROR        (apb3Router_1__io_input_PSLVERROR         ), //o
    .io_outputs_0_PADDR        (apb3Router_1__io_outputs_0_PADDR[19:0]   ), //o
    .io_outputs_0_PSEL         (apb3Router_1__io_outputs_0_PSEL          ), //o
    .io_outputs_0_PENABLE      (apb3Router_1__io_outputs_0_PENABLE       ), //o
    .io_outputs_0_PREADY       (system_gpioACtrl_io_apb_PREADY           ), //i
    .io_outputs_0_PWRITE       (apb3Router_1__io_outputs_0_PWRITE        ), //o
    .io_outputs_0_PWDATA       (apb3Router_1__io_outputs_0_PWDATA[31:0]  ), //o
    .io_outputs_0_PRDATA       (system_gpioACtrl_io_apb_PRDATA[31:0]     ), //i
    .io_outputs_0_PSLVERROR    (system_gpioACtrl_io_apb_PSLVERROR        ), //i
    .io_outputs_1_PADDR        (apb3Router_1__io_outputs_1_PADDR[19:0]   ), //o
    .io_outputs_1_PSEL         (apb3Router_1__io_outputs_1_PSEL          ), //o
    .io_outputs_1_PENABLE      (apb3Router_1__io_outputs_1_PENABLE       ), //o
    .io_outputs_1_PREADY       (system_uartCtrl_io_apb_PREADY            ), //i
    .io_outputs_1_PWRITE       (apb3Router_1__io_outputs_1_PWRITE        ), //o
    .io_outputs_1_PWDATA       (apb3Router_1__io_outputs_1_PWDATA[31:0]  ), //o
    .io_outputs_1_PRDATA       (system_uartCtrl_io_apb_PRDATA[31:0]      ), //i
    .io_outputs_1_PSLVERROR    (_zz_12_                                  ), //i
    .io_outputs_2_PADDR        (apb3Router_1__io_outputs_2_PADDR[19:0]   ), //o
    .io_outputs_2_PSEL         (apb3Router_1__io_outputs_2_PSEL          ), //o
    .io_outputs_2_PENABLE      (apb3Router_1__io_outputs_2_PENABLE       ), //o
    .io_outputs_2_PREADY       (system_timer_io_apb_PREADY               ), //i
    .io_outputs_2_PWRITE       (apb3Router_1__io_outputs_2_PWRITE        ), //o
    .io_outputs_2_PWDATA       (apb3Router_1__io_outputs_2_PWDATA[31:0]  ), //o
    .io_outputs_2_PRDATA       (system_timer_io_apb_PRDATA[31:0]         ), //i
    .io_outputs_2_PSLVERROR    (system_timer_io_apb_PSLVERROR            ), //i
    .io_mainClk                (io_mainClk                               ), //i
    .resetCtrl_systemReset     (resetCtrl_systemReset                    )  //i
  );
  always @(*) begin
    case(system_mainBusDecoder_logic_rspSourceId)
      1'b0 : begin
        _zz_13_ = system_ram_io_bus_rsp_payload_data;
      end
      default : begin
        _zz_13_ = system_apbBridge_io_pipelinedMemoryBus_rsp_payload_data;
      end
    endcase
  end

  always @ (*) begin
    resetCtrl_mainClkResetUnbuffered = 1'b0;
    if(_zz_14_)begin
      resetCtrl_mainClkResetUnbuffered = 1'b1;
    end
  end

  assign _zz_1_[5 : 0] = 6'h3f;
  always @ (*) begin
    system_timerInterrupt = 1'b0;
    if(system_timer_io_interrupt)begin
      system_timerInterrupt = 1'b1;
    end
  end

  always @ (*) begin
    system_externalInterrupt = 1'b0;
    if(system_uartCtrl_io_interrupt)begin
      system_externalInterrupt = 1'b1;
    end
  end

  assign system_cpu_dBus_cmd_halfPipe_valid = system_cpu_dBus_cmd_halfPipe_regs_valid;
  assign system_cpu_dBus_cmd_halfPipe_payload_wr = system_cpu_dBus_cmd_halfPipe_regs_payload_wr;
  assign system_cpu_dBus_cmd_halfPipe_payload_address = system_cpu_dBus_cmd_halfPipe_regs_payload_address;
  assign system_cpu_dBus_cmd_halfPipe_payload_data = system_cpu_dBus_cmd_halfPipe_regs_payload_data;
  assign system_cpu_dBus_cmd_halfPipe_payload_size = system_cpu_dBus_cmd_halfPipe_regs_payload_size;
  assign system_cpu_dBus_cmd_halfPipe_ready = system_mainBusArbiter_io_dBus_cmd_ready;
  assign _zz_6_ = systemDebugger_1__io_mem_cmd_payload_address[7:0];
  assign io_jtag_tdo = jtagBridge_1__io_jtag_tdo;
  assign io_gpioA_write = system_gpioACtrl_io_gpio_write;
  assign io_gpioA_writeEnable = system_gpioACtrl_io_gpio_writeEnable;
  assign io_uart_txd = system_uartCtrl_io_uart_txd;
  assign _zz_9_ = apb3Router_1__io_outputs_0_PADDR[3:0];
  assign _zz_10_ = apb3Router_1__io_outputs_1_PADDR[4:0];
  assign _zz_12_ = 1'b0;
  assign _zz_11_ = apb3Router_1__io_outputs_2_PADDR[7:0];
  assign system_mainBusDecoder_logic_masterPipelined_cmd_valid = system_mainBusArbiter_io_masterBus_cmd_valid;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_write = system_mainBusArbiter_io_masterBus_cmd_payload_write;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_address = system_mainBusArbiter_io_masterBus_cmd_payload_address;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_data = system_mainBusArbiter_io_masterBus_cmd_payload_data;
  assign system_mainBusDecoder_logic_masterPipelined_cmd_payload_mask = system_mainBusArbiter_io_masterBus_cmd_payload_mask;
  assign system_mainBusDecoder_logic_hits_0 = ((system_mainBusDecoder_logic_masterPipelined_cmd_payload_address & (~ 32'h00001fff)) == 32'h80000000);
  always @ (*) begin
    _zz_7_ = (system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_hits_0);
    if(_zz_15_)begin
      _zz_7_ = 1'b0;
    end
  end

  assign _zz_3_ = system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  assign system_mainBusDecoder_logic_hits_1 = ((system_mainBusDecoder_logic_masterPipelined_cmd_payload_address & (~ 32'h000fffff)) == 32'hf0000000);
  always @ (*) begin
    _zz_8_ = (system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_hits_1);
    if(_zz_15_)begin
      _zz_8_ = 1'b0;
    end
  end

  assign _zz_4_ = system_mainBusDecoder_logic_masterPipelined_cmd_payload_write;
  assign system_mainBusDecoder_logic_noHit = (! ({system_mainBusDecoder_logic_hits_1,system_mainBusDecoder_logic_hits_0} != (2'b00)));
  always @ (*) begin
    system_mainBusDecoder_logic_masterPipelined_cmd_ready = (({(system_mainBusDecoder_logic_hits_1 && system_apbBridge_io_pipelinedMemoryBus_cmd_ready),(system_mainBusDecoder_logic_hits_0 && system_ram_io_bus_cmd_ready)} != (2'b00)) || system_mainBusDecoder_logic_noHit);
    if(_zz_15_)begin
      system_mainBusDecoder_logic_masterPipelined_cmd_ready = 1'b0;
    end
  end

  assign system_mainBusDecoder_logic_masterPipelined_rsp_valid = (({system_apbBridge_io_pipelinedMemoryBus_rsp_valid,system_ram_io_bus_rsp_valid} != (2'b00)) || (system_mainBusDecoder_logic_rspPending && system_mainBusDecoder_logic_rspNoHit));
  assign system_mainBusDecoder_logic_masterPipelined_rsp_payload_data = _zz_13_;
  assign _zz_5_ = 1'b0;
  always @ (posedge io_mainClk) begin
    if(_zz_14_)begin
      resetCtrl_systemClkResetCounter <= (resetCtrl_systemClkResetCounter + 6'h01);
    end
    if(io_asyncReset_buffercc_io_dataOut)begin
      resetCtrl_systemClkResetCounter <= 6'h0;
    end
  end

  always @ (posedge io_mainClk) begin
    resetCtrl_mainClkReset <= resetCtrl_mainClkResetUnbuffered;
    resetCtrl_systemReset <= resetCtrl_mainClkResetUnbuffered;
    if(system_cpu_debug_resetOut_regNext)begin
      resetCtrl_systemReset <= 1'b1;
    end
  end

  always @ (posedge io_mainClk or posedge resetCtrl_systemReset) begin
    if (resetCtrl_systemReset) begin
      system_cpu_dBus_cmd_halfPipe_regs_valid <= 1'b0;
      system_cpu_dBus_cmd_halfPipe_regs_ready <= 1'b1;
      system_mainBusDecoder_logic_rspPending <= 1'b0;
      system_mainBusDecoder_logic_rspNoHit <= 1'b0;
    end else begin
      if(_zz_16_)begin
        system_cpu_dBus_cmd_halfPipe_regs_valid <= system_cpu_dBus_cmd_valid;
        system_cpu_dBus_cmd_halfPipe_regs_ready <= (! system_cpu_dBus_cmd_valid);
      end else begin
        system_cpu_dBus_cmd_halfPipe_regs_valid <= (! system_cpu_dBus_cmd_halfPipe_ready);
        system_cpu_dBus_cmd_halfPipe_regs_ready <= system_cpu_dBus_cmd_halfPipe_ready;
      end
      if(system_mainBusDecoder_logic_masterPipelined_rsp_valid)begin
        system_mainBusDecoder_logic_rspPending <= 1'b0;
      end
      if(((system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_masterPipelined_cmd_ready) && (! system_mainBusDecoder_logic_masterPipelined_cmd_payload_write)))begin
        system_mainBusDecoder_logic_rspPending <= 1'b1;
      end
      system_mainBusDecoder_logic_rspNoHit <= 1'b0;
      if(system_mainBusDecoder_logic_noHit)begin
        system_mainBusDecoder_logic_rspNoHit <= 1'b1;
      end
    end
  end

  always @ (posedge io_mainClk) begin
    if(_zz_16_)begin
      system_cpu_dBus_cmd_halfPipe_regs_payload_wr <= system_cpu_dBus_cmd_payload_wr;
      system_cpu_dBus_cmd_halfPipe_regs_payload_address <= system_cpu_dBus_cmd_payload_address;
      system_cpu_dBus_cmd_halfPipe_regs_payload_data <= system_cpu_dBus_cmd_payload_data;
      system_cpu_dBus_cmd_halfPipe_regs_payload_size <= system_cpu_dBus_cmd_payload_size;
    end
    if((system_mainBusDecoder_logic_masterPipelined_cmd_valid && system_mainBusDecoder_logic_masterPipelined_cmd_ready))begin
      system_mainBusDecoder_logic_rspSourceId <= system_mainBusDecoder_logic_hits_1;
    end
  end

  always @ (posedge io_mainClk) begin
    system_cpu_debug_resetOut_regNext <= system_cpu_debug_resetOut;
  end

  always @ (posedge io_mainClk or posedge resetCtrl_mainClkReset) begin
    if (resetCtrl_mainClkReset) begin
      _zz_2_ <= 1'b0;
    end else begin
      _zz_2_ <= (systemDebugger_1__io_mem_cmd_valid && system_cpu_debug_bus_cmd_ready);
    end
  end


endmodule
