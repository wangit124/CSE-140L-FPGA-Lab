// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
// --------------------------------------------------------------------
// Copyright (c) 2019 by UCSD CSE 140L
// --------------------------------------------------------------------
//
// Permission:
//
//   This code for use in UCSD CSE 140L.
//   It is synthesisable for Lattice iCEstick 40HX.  
//	
// Disclaimer:
//
//   This Verilog source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  
//
// -------------------------------------------------------------------- //           
//                     UCSD CSE Department
//                     9500 Gilman Dr, La Jolla, CA 92093
//                     U.S.A
//
// --------------------------------------------------------------------
// NOTE: HAS BEEN MODIFIED FOR LAB 5 EXTRA CREDIT

module Lab3_140L (
		input wire 	    rst, // reset signal (active high)
		input wire 	    clk, // global clock
		input wire oneSecStrb,  	    
		input 	    bu_rx_data_rdy, // data from the uart ready
		input [7:0] 	    bu_rx_data, // data from the uart
		output wire 	    L3_tx_data_rdy, // data to the alarm display
		output wire [7:0] L3_tx_data,     // data to the alarm display
		output [4:0] 	    L3_led,
		output wire [6:0] 	    L3_segment1, // 1's seconds
		output wire [6:0] 	    L3_segment2, // 10's seconds
		output wire [6:0] 	    L3_segment3, // 1's minutes
		output wire [6:0] 	    L3_segment4, // 10's minutes

		output [3:0] 	    di_Mtens,			// Tens minute
		output [3:0] 	    di_Mones,			// Ones minute
		output [3:0] 	    di_Stens,			// tens second
		output wire [3:0]  di_Sones,			// Ones second
		output [3:0] 	    di_AMtens,			// alarm ten minute
		output [3:0] 	    di_AMones,			// alarm one minute
		output [3:0] 	    di_AStens,			// alarm ten second
		output [3:0] 	    di_ASones			// alarm one second
		);
		
		// Define wires to enable segment displays
		wire	enableSeg1;
		wire	enableSeg2;
		wire	enableSeg3;
		wire	enableSeg4;
		 
		// Define connections between dp and ctrl
		wire ld_enable_Sones;
		wire ld_enable_Stens;
		wire ld_enable_Mones;
		wire ld_enable_Mtens;
		wire ld_enable_ASones;
		wire ld_enable_AStens;
		wire ld_enable_AMones;
		wire ld_enable_AMtens;
		wire ld_enable_dicRun;
		wire alarm_off;
		wire alarm_armed;
		wire alarm_triggered;
		wire alarm_match;
		wire n_rdy;
		
		// Decide to display either alarm or time
		wire alarm_or_time;
		wire [3:0] Sone_at;
		wire [3:0] Sten_at;
		wire [3:0] Mone_at;
		wire [3:0] Mten_at;
		
		// Initialize Counter
		wire [3:0]	Sone_init;  // initialize Sone
		wire [3:0]	Sten_init;  // initialize Sten
		wire [3:0]	Mone_init;  // initialize Mone
		wire [3:0]	Mten_init;  // initialize Mten
		
		// Create wires to toggle segment output
		wire [6:0]  tempSeg1;
		wire [6:0]	tempSeg2;
		wire [6:0]  tempSeg3;
		wire [6:0]	tempSeg4;
		
		// Temp character
		reg	[7:0]	displayAlarm;
		
		// Set status character
		always @(posedge clk) begin
			 if (alarm_off) begin
				displayAlarm <= ".";
			 end
			 else if (alarm_armed) begin
				displayAlarm <= "a";
			 end
			 else begin
				displayAlarm <= "T";
			 end
		end
		
		// Assign segments based on alarm trigger (bEEP)
		assign L3_segment1 = (alarm_triggered && ld_enable_dicRun && ~clk) ? 7'b1110011 : tempSeg1;
		assign L3_segment2 = (alarm_triggered && ld_enable_dicRun && ~clk) ? 7'b1111001 : tempSeg2;
		assign L3_segment3 = (alarm_triggered && ld_enable_dicRun && ~clk) ? 7'b1111001 : tempSeg3;
		assign L3_segment4 = (alarm_triggered && ld_enable_dicRun && ~clk) ? 7'b1111100 : tempSeg4;
		
		// Wire generating half second pulse
		wire halfPulse;
		defparam uu0.CLK_FREQ = 12000000;
		
		// Instantiate half second module
		Half_Sec_Pulse_Per_Sec_v2 uu0 (
			.i_rst (rst),       		 //reset
			.i_clk (clk),       		 //system clk 12MHz 
			.o_sec_tick (halfPulse)  //0.5sec 1 and 0.5sec 0
		);
		
		
		// Instantiate control module
		dictrl dictrl(
				.dicLdMtens(ld_enable_Mtens), 	// load the 10's minutes
				.dicLdMones(ld_enable_Mones), 	// load the 1's minutes
				.dicLdStens(ld_enable_Stens), 	// load the 10's seconds
				.dicLdSones(ld_enable_Sones), 	// load the 1's seconds
				.dicLdAMtens(ld_enable_AMtens), 	// load the alarm 10's minutes
				.dicLdAMones(ld_enable_AMones), 	// load the alarm 1's minutes
				.dicLdAStens(ld_enable_AStens), 	// load the alarm 10's seconds
				.dicLdASones(ld_enable_ASones), 	// load the alarm 1's seconds
				.dicRun(ld_enable_dicRun), 		// clock should run
				.n_rdy(n_rdy),							// User enters 'n'
				.alarm_or_time(alarm_or_time), 	// Display alarm or time
				.enable1(enableSeg1),				// enable segment 1
				.enable2(enableSeg2),				// enable segment 2
				.enable3(enableSeg3),				// enable segment 3
				.enable4(enableSeg4),				// enable segment 4
				.Sone_init(Sone_init),  			// Initialize clock
				.Sten_init(Sten_init),				// Initialize clock
				.Mone_init(Mone_init),				// Initialize clock
				.Mten_init(Mten_init),				// Initialize clock
				.dicAlarmIdle(alarm_off), 			// alarm is off
				.dicAlarmArmed(alarm_armed), 		// alarm is armed
				.dicAlarmTrig(alarm_triggered), 	// alarm is triggered
				.did_alarmMatch(alarm_match), 	// raw alarm match
				.halfPulse(halfPulse),				// Half second pulse
				.bu_rx_data_rdy(bu_rx_data_rdy), // new data from uart rdy
				.bu_rx_data(bu_rx_data), 			// new data from uart
				.rst(rst),								// Global reset
				.clk(clk)								// Global clock
		);

		// Instantiate datapath module
		didp didp(
				.di_Mtens(di_Mtens), 				// load the 10's minutes
				.di_Mones(di_Mones), 				// load the 1's minutes
				.di_Stens(di_Stens), 				// load the 10's seconds
				.di_Sones(di_Sones), 				// load the 1's seconds
				.di_AMtens(di_AMtens), 				// load the alarm 10's minutes
				.di_AMones(di_AMones), 				// load the alarm 1's minutes
				.di_AStens(di_AStens), 				// load the alarm 10's seconds
				.di_ASones(di_ASones), 				// load the alarm 1's seconds
				.did_alarmMatch(alarm_match), 	// raw alarm match
				.L3_led(L3_led), 						// LED display
				.bu_rx_data(bu_rx_data), 			// new data from uart
				.dicLdMtens(ld_enable_Mtens), 	// load the 10's minutes
				.dicLdMones(ld_enable_Mones), 	// load the 1's minutes
				.dicLdStens(ld_enable_Stens), 	// load the 10's seconds
				.dicLdSones(ld_enable_Sones), 	// load the 1's seconds
				.dicLdAMtens(ld_enable_AMtens), 	// load the alarm 10's minutes
				.dicLdAMones(ld_enable_AMones), 	// load the alarm 1's minutes
				.dicLdAStens(ld_enable_AStens), 	// load the alarm 10's seconds
				.dicLdASones(ld_enable_ASones), 	// load the alarm 1's seconds
				.dicRun(ld_enable_dicRun), 		// clock should run
				.Sone_init(Sone_init),  			// Initialize clock
				.Sten_init(Sten_init),  			// Initialize clock
				.Mone_init(Mone_init),				// Initialize clock
				.Mten_init(Mten_init),				// Initialize clock
				.n_rdy(n_rdy),							// User enters 'n'
				.oneSecStrb(oneSecStrb),			// One sec counter
				.rst(rst),								// Global reset
				.clk(clk)								// Global clock
		); 
				
		// Assign displays
		assign Sone_at = alarm_or_time ? di_ASones : di_Sones;
		assign Sten_at = alarm_or_time ? di_AStens : di_Stens;
		assign Mone_at = alarm_or_time ? di_AMones : di_Mones;
		assign Mten_at = alarm_or_time ? di_AMtens : di_Mtens;
		
		// Instantiate b2cdsegment modules
		bcd2segment bcd2segment1 (
		  .segment(tempSeg1),  					// 7 drivers for segment
		  .num(Sone_at),       						// number to convert
		  .enable(enableSeg1)  						// if 1, drive display, else blank
		);
		    
	   bcd2segment bcd2segment2 (
		  .segment(tempSeg2),  					// 7 drivers for segment
		  .num(Sten_at),      					   // number to convert
		  .enable(enableSeg2)   					// if 1, drive display, else blank
		);
		 
		bcd2segment bcd2segment3 (
		  .segment(tempSeg3), 					// 7 drivers for segment
		  .num(Mone_at),       						// number to convert
		  .enable(enableSeg3)  						// if 1, drive display, else blank
		);
		
		bcd2segment bcd2segment4 (
		  .segment(tempSeg4),  					// 7 drivers for segment
		  .num(Mten_at),       						// number to convert
		  .enable(enableSeg4)   					// if 1, drive display, else blank
		);
		
		// Instantiate displayString
		dispString display(
		 		  .rdy(L3_tx_data_rdy), // dOut  is valid
				  .dOut(L3_tx_data), // data b0 -> b7
				  .dInP(), // future expansion
				  .rdyInP(), // future expansion
				  .b0({4'b0011, di_AMtens}),
				  .b1({4'b0011, di_AMones}),
				  .b2(":"),
				  .b3({4'b0011, di_AStens}),
				  .b4({4'b0011, di_ASones}),
				  .b5(" "),
				  .b6(displayAlarm),
				  .b7(8'h0d),
				  .go(oneSecStrb), // start streaming
				  .rst(rst),
				  .clk(clk)
				);
		endmodule // Lab3_140L


		// Datapath
		module didp (
				output [3:0] di_Mtens,// current 10's minutes
				output [3:0] di_Mones, // current 1's minutes
				output [3:0] di_Stens, // current 10's second
				output [3:0] di_Sones, // current 1's second

				output [3:0] di_AMtens, // current alarm 10's minutes
				output [3:0] di_AMones, // current alarm 1's minutes
				output [3:0] di_AStens, // current alarm 10's second
				output [3:0] di_ASones, // current alarm 1's second

				output wire  did_alarmMatch, // one cydie alarm match (raw signal, unqualified)
				
				output [4:0] L3_led, 	// LED displays

				input [7:0]  bu_rx_data, // Keyboard ASCII
				
				input 	  dicLdMtens, 	// load 10's minute
				input 	  dicLdMones, 	// load 1's minute
				input 	  dicLdStens, 	// load 10's second
				input 	  dicLdSones, 	// load 1's second

				input 	  dicLdAMtens, // load alarm 10's minute
				input 	  dicLdAMones, // load alarm 1's minute
				input 	  dicLdAStens, // load alarm 10's second
				input 	  dicLdASones, // load alarm 1's second
				input 	  dicRun, 		// clock should run 	  
				
				input [3:0]	Sone_init,  // initialize Sone
				input [3:0]	Sten_init,  // initialize Sten
				input [3:0]	Mone_init,  // initialize Mone
				input [3:0]	Mten_init,  // initialize Mten
				input			n_rdy,		// User enters 'n'
				input 	  oneSecStrb, 	// one cycle strobe
				
				input 	  rst, 			// Global reset
				input 	  clk  			// Global clock
		);
			
			// Create signal wires for counter
			wire 			Sones_rst;
			wire 		   Stens_rst;
			wire 		   Mones_rst;
			wire        Mtens_rst;
			
			wire 			Sones_ce;
			wire 		   Stens_ce;
			wire 		   Mones_ce;
			wire        Mtens_ce;
			
			// Create signal wires for alarm 
			wire 			ASones_ce;
			wire 		   AStens_ce;
			wire 		   AMones_ce;
			wire        AMtens_ce;
			
			// Define four states for LEDs
			parameter	[1:0]	LED1 = 2'b00;
			parameter	[1:0]	LED2 = 2'b01;
			parameter	[1:0]	LED3 = 2'b10;
			parameter	[1:0]	LED4 = 2'b11;
			
			// Define current and next LED states
			reg	[1:0]		curr_LED;
			reg	[1:0]		next_LED;
			reg	[4:0]		L3_led_temp; // temp LED
			
			// Set signal to high once reached threshold
			assign Mtens_ce = ((di_Mones == 4'b0000) & (di_Stens == 4'b0000) & (di_Sones == 4'b0000) & oneSecStrb & dicRun) & ~did_alarmMatch;
			assign Mones_ce = ((di_Stens == 4'b0000) & (di_Sones == 4'b0000) & oneSecStrb & dicRun) & ~did_alarmMatch;
			assign Stens_ce = ((di_Sones == 4'b0000) & oneSecStrb & dicRun) & ~did_alarmMatch;
			assign Sones_ce = oneSecStrb & dicRun & ~did_alarmMatch;
			
			// If reached threshold, reset
			assign Mtens_rst = (((di_Mtens == 4'b0000) & (di_Mones == 4'b0000) & (di_Stens == 4'b0000) & (di_Sones == 4'b0000) & oneSecStrb) & dicRun);
			assign Mones_rst = (((di_Mones == 4'b0000) & (di_Stens == 4'b0000) & (di_Sones == 4'b0000) & oneSecStrb) & dicRun);
			assign Stens_rst = (((di_Stens == 4'b0000) & (di_Sones == 4'b0000) & oneSecStrb) & dicRun);
			assign Sones_rst = (((di_Sones == 4'b0000) & oneSecStrb) & dicRun);
			
			// Assign alarm reset wires 
			assign ASones_ce = dicLdASones;
			assign AStens_ce = dicLdAStens;
			assign AMones_ce = dicLdAMones;
			assign AMtens_ce = dicLdAMtens;
			
			// Check if alarm matches
			assign did_alarmMatch = ((di_Sones == 4'b0000) & (di_Stens == 4'b0000) & (di_Mones == 4'b0000) & (di_Mtens == 4'b0000));
			
			// Start in s sone state
			always @(posedge clk) begin
				if (rst) begin
					curr_LED <= LED1;
				end
				else begin
					curr_LED <= next_LED;
				end		
			end
			
			// Check for changes in n_rdy
			always @(*) begin
				case (curr_LED)
					LED1:
					begin
						if (n_rdy)
							next_LED <= LED2;
						else
							next_LED <= LED1;
					end
					LED2:
					begin
						if (n_rdy)
							next_LED <= LED3;
						else
							next_LED <= LED2;
					end
					LED3:
					begin
						if (n_rdy)
							next_LED <= LED4;
						else
							next_LED <= LED3;
					end
					LED4:
					begin
						if (n_rdy)
							next_LED <= LED1;
						else
							next_LED <= LED4;
					end
				endcase
			end
			
			// Upon state change, set output
			always @(*) begin
				case (curr_LED)
					LED1: L3_led_temp <= di_Sones;
					LED2: L3_led_temp <= di_Stens;
					LED3: L3_led_temp <= di_Mones;
					LED4: L3_led_temp <= di_Mtens;
				endcase
			end
			
			// Assign L3_LED to currLED
			assign L3_led = L3_led_temp;
			
			// Initalize counters for timer
			countrce Sones_subtractor (
				.q(di_Sones),
				.system(Mtens_rst),
				.init(4'b1001),
				.d(bu_rx_data[3:0]),
				.ld(dicLdSones),
				.ce(Sones_ce),
				.rst(rst),
				.manual(Sones_rst),
				.clk(clk)
			);
			
			countrce Stens_subtractor(
				.q(di_Stens),
				.system(Mtens_rst),
				.init(4'b0101),
				.d(bu_rx_data[3:0]),
				.ld(dicLdStens),
				.ce(Stens_ce),
				.rst(rst),
				.manual(Stens_rst),
				.clk(clk)
			);
			
			countrce Mones_subtractor (
				.q(di_Mones),
				.system(Mtens_rst),
				.init(4'b1001),
				.d(bu_rx_data[3:0]),
				.ld(dicLdMones),
				.ce(Mones_ce),
				.rst(rst),
				.manual(Mones_rst),
				.clk(clk)
			);
			
			countrce Mtens_subtractor(
				.q(di_Mtens),
				.system(Mtens_rst),
				.init(4'b0101),
				.d(bu_rx_data[3:0]),
				.ld(dicLdMtens),
				.ce(Mtens_ce),
				.rst(rst),
				.manual(Mtens_rst),
				.clk(clk)
			);
			
			// Initialize counters for alarm clock
			regrce #(4) Sones_alarm (
				.q(di_ASones),
				.d(bu_rx_data[3:0]),
				.ce(ASones_ce),
				.rst(rst),
				.clk(clk)
			);
			
			regrce #(4) Stens_alarm(
				.q(di_AStens),
				.d(bu_rx_data[3:0]),
				.ce(AStens_ce),
				.rst(rst),
				.clk(clk)
			);
			
			regrce #(4) Mones_alarm (
				.q(di_AMones),
				.d(bu_rx_data[3:0]),
				.ce(AMones_ce),
				.rst(rst),
				.clk(clk)
			);
			
			regrce #(4) Mtens_alarm(
				.q(di_AMtens),
				.d(bu_rx_data[3:0]),
				.ce(AMtens_ce),
				.rst(rst),
				.clk(clk)
			);
			
			endmodule
				
			// CTRL Module
			module dictrl(
					output 	  dicLdMtens, 	// load the 10's minutes
					output 	  dicLdMones, 	// load the 1's minutes
					output 	  dicLdStens, 	// load the 10's seconds
					output 	  dicLdSones, 	// load the 1's seconds
					output 	  dicLdAMtens, // load the alarm 10's minutes
					output 	  dicLdAMones, // load the alarm 1's minutes
					output 	  dicLdAStens, // load the alarm 10's seconds
					output 	  dicLdASones, // load the alarm 1's seconds
					output 	  dicRun, 		// clock should run
					output		n_rdy,				// Check if user enters 'n'
					output	   alarm_or_time, // Determine whether to display alarm or time
					
					output		enable1, 	// enable segment 1 display
					output		enable2, 	// enable segment 2 display
					output		enable3, 	// enable segment 3 display
					output	 	enable4, 	// enable segment 4 display
					
					output [3:0]	Sone_init,  // initialize Sone
					output [3:0]	Sten_init,  // initialize Sten
					output [3:0]	Mone_init,  // initialize Mone
					output [3:0]	Mten_init,  // initialize Mten
					
					output 		 dicAlarmIdle, 	// alarm is off
					output 		 dicAlarmArmed, // alarm is armed
					output 		 dicAlarmTrig, 	// alarm is triggered

					input       did_alarmMatch, 	// raw alarm match
					input			halfPulse,			// Parameter for display flashing
					input 	   bu_rx_data_rdy, 	// new data from uart rdy
					input [7:0] bu_rx_data, 		// new data from uart
					input 	   rst, 					// Global reset
					input 	   clk 					// Global clock
			);
			
			// Define registers for all inputs and outputs
			reg 	  r_dicLdMtens; 	// load the 10's minutes
			reg 	  r_dicLdMones; 	// load the 1's minutes
			reg 	  r_dicLdStens; 	// load the 10's seconds
			reg 	  r_dicLdSones; 	// load the 1's seconds
			reg 	  r_dicLdAMtens; 	// load the alarm 10's minutes
			reg 	  r_dicLdAMones; 	// load the alarm 1's minutes
			reg 	  r_dicLdAStens; 	// load the alarm 10's seconds
			reg 	  r_dicLdASones; 	// load the alarm 1's seconds
			reg 	  r_dicRun; 		// clock should run
			
			reg	  r_enable1; 		// enable segment 1 display
			reg	  r_enable2; 		// enable segment 2 display
			reg	  r_enable3; 		// enable segment 3 display
			reg	  r_enable4; 		// enable segment 4 display
			
			reg	[3:0] r_Sone_init; // Initialize clock
			reg	[3:0] r_Sten_init; // Initialize clock
			reg	[3:0] r_Mone_init; // Initialize clock
			reg	[3:0] r_Mten_init; // Initialize clock
			
			reg	  r_alarm_or_time; 	// Display alarm or regular time
			
			reg 	  r_dicAlarmIdle; 	// alarm is off
			reg 	  r_dicAlarmArmed; 	// alarm is armed
			reg	  r_dicAlarmTrig; 	// alarm is triggered
			
			// Check if alarm set
			wire		is_alarm_set;
			reg		alarm_set;
			
			// Wires for decoding
			wire		  de_esc;
		   wire       de_num;
		   wire       de_num0to5; 
		   wire       de_cr;
		   wire       de_atSign;
		   wire       de_littleA;
		   wire       de_littleL;
		   wire       de_littleN;
			
			// Instantiate decodeKeys
			decodeKeys decoder( 
				  .de_esc(de_esc),
				  .de_num(de_num),
				  .de_num0to5(de_num0to5), 
				  .de_cr(de_cr),
				  .de_atSign(de_atSign),
				  .de_littleA(de_littleA),
				  .de_littleL(de_littleL),
				  .de_littleN(de_littleN),
				  .charData(bu_rx_data[7:0]),
				  .charDataValid(bu_rx_data_rdy)
		   );

			// Define registers for current and next state
			reg	[3:0] 	currState;
			reg	[3:0]		nextState;
			reg	[3:0] 	currState_al;
			reg	[3:0]		nextState_al;

			// Load FSM
			parameter [3:0] 	start = 4'b0001;
			parameter [3:0] 	l_load = 4'b0010;
			parameter [3:0] 	l_Mten = 4'b0011;
			parameter [3:0] 	l_mone = 4'b0100;
			parameter [3:0] 	l_Sten = 4'b0101;
			parameter [3:0] 	l_sone = 4'b0110;
			parameter [3:0] 	a_load = 4'b0111;
			parameter [3:0] 	a_Mten = 4'b1000;
			parameter [3:0] 	a_mone = 4'b1001;
			parameter [3:0] 	a_Sten = 4'b1010;
			parameter [3:0] 	a_sone = 4'b1011;
			parameter [3:0]	count_start = 4'b1111;
			
			// Alarm FSM
			parameter [3:0]	a_off = 4'b1100;
			parameter [3:0]	a_armed = 4'b1101;
			parameter [3:0]	a_trig = 4'b1110;

			// If reset signal, switch to start state
			always @(posedge clk) begin
				if (rst) begin
					// Go to start state
					currState <= start;
					currState_al <= a_off;
				end
				// Otherwise go to next state
				else begin
					currState <= nextState;
					currState_al <= nextState_al;
				end
			end
			
			// Change inputs based on state
			always @(posedge clk) begin
				if (currState == start) begin
						// Set default values for clock
						r_Sone_init <= 4'd0;
						r_Sten_init <= 4'd0;
						r_Mone_init <= 4'd0;
						r_Mten_init <= 4'd0;
						
						// Pause clock
						r_dicRun <= 1'b0;	
					
						// Flash display
						r_enable1 <= (halfPulse | r_dicRun);
						r_enable2 <= (halfPulse | r_dicRun);
						r_enable3 <= (halfPulse | r_dicRun);
						r_enable4 <= (halfPulse | r_dicRun);
						
						// Display timer, and set all outputs to 0
						r_alarm_or_time <= 1'b0;
				end
				else if (currState == count_start) begin
						// Set default values for clock
						r_Sone_init <= 4'd0;
						r_Sten_init <= 4'd0;
						r_Mone_init <= 4'd0;
						r_Mten_init <= 4'd0;
						
						// start clock
						r_dicRun <= 1'b1;
						
						// Maintain display
						r_enable1 <= 1'b1;
						r_enable2 <= 1'b1;
						r_enable3 <= 1'b1;
						r_enable4 <= 1'b1;
						
						// Set other outputs to 0
						r_alarm_or_time <= 1'b0;
				end
				else if (currState == l_load) begin
						// When loading time, pause clock and close display
						r_dicRun <= 1'b0;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b0; // disable segment 3 display
						r_enable4 <= 1'b0; // disable segment 4 display
						
						// Display timer
						r_alarm_or_time <= 1'b0;
				end
				else if (currState == l_Mten) begin
						// Set other outputs to 0
						r_dicRun <= 1'b0;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b0; // disable segment 3 display
						r_enable4 <= 1'b1; // enable segment 4 display
						r_alarm_or_time <= 1'b0; // Display timer
				end
				else if (currState == l_mone) begin
						// Set other outputs to 0
						r_dicRun <= 1'b0;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b0; // Display timer
				end
				else if (currState == l_Sten) begin
						// Set other outputs to 0
						r_dicRun <= 1'b0;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b1; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b0; // Display timer
				end
				else if (currState == l_sone) begin
						// set other outputs to 0
						r_dicRun <= 1'b0;
						
						r_enable1 <= 1'b1; // disable segment 1 display
						r_enable2 <= 1'b1; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b0; // Display timer
				end 
				else if (currState == a_load) begin
						// When loading time, pause clock and close display
						r_dicRun <= 1'b1;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b0; // disable segment 3 display
						r_enable4 <= 1'b0; // disable segment 4 display
						
						// Display timer
						r_alarm_or_time <= 1'b1;
				end
				else if (currState == a_Mten) begin
						// Set other outputs to 0
						r_dicRun <= 1'b1;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b0; // disable segment 3 display
						r_enable4 <= 1'b1; // enable segment 4 display
						r_alarm_or_time <= 1'b1; // Display timer
				end
				else if (currState == a_mone) begin
						// Set other outputs to 0
						r_dicRun <= 1'b1;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b0; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b1; // Display timer
				end
				else if (currState == a_Sten) begin
						// Set other outputs to 0
						r_dicRun <= 1'b1;
						
						r_enable1 <= 1'b0; // disable segment 1 display
						r_enable2 <= 1'b1; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b1; // Display timer
				end
				else if (currState == a_sone) begin
						// set other outputs to 0
						r_dicRun <= 1'b1;
						
						r_enable1 <= 1'b1; // disable segment 1 display
						r_enable2 <= 1'b1; // disable segment 2 display
						r_enable3 <= 1'b1; // enable segment 3 display
						r_enable4 <= 1'b1; // enable segment 3 display
						
						r_alarm_or_time <= 1'b1; // Display timer
				end 
				else begin
				end
			end
			
			// Check for keyboard inputs
			always @(*) begin
				case (currState)
					start:
					begin
						// Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;	
						
						// If pressed 'l', load time
						if (de_littleL)
							nextState <= l_load;
						// if pressed 'a' load alarm
						else if (de_littleA)
							nextState <= a_load;
						// otherwise maintain state
						else
							nextState <= start;
					end
					l_load:
					begin
					   // Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed 0-5, load Mten, next
						if (de_num0to5) begin
							r_dicLdMtens <= 1'b1;
							nextState <= l_Mten;
						end
						// otherwise maintain state
						else begin
							nextState <= l_load;
						end
					end
					l_Mten:
					begin
						// Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed 0-9, load mone, next
						if (de_num) begin
							r_dicLdMones <= 1'b1;
							nextState <= l_mone;	
						end
						// Otherwise stay
						else begin
							nextState <= l_Mten;
						end
					end
					l_mone:
					begin
					   // Reset signals
					   r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed 0-5, load sten next
						if (de_num0to5) begin
							r_dicLdStens <= 1'b1;
							nextState <= l_Sten;	
						end
						// otherwise stay
						else begin
							nextState <= l_mone;
						end
					end
					l_Sten:
					begin
						// Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed 0-9, load sone next
						if (de_num) begin
							r_dicLdSones <= 1'b1;
							nextState <= l_sone;	
						end
						// Otherwise stay
						else begin
							nextState <= l_Sten;
						end
					end
					l_sone:
					begin
						// Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed CR, start counting, next
						if (de_cr) begin
							nextState <= count_start;
						end
						// Otherwise stay
						else begin
							nextState <= l_sone;
						end
					end
					a_load:
					begin
					   // Reset signals
						r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;	
						
						// If pressed 0-5, load Mten, next
						if (de_num0to5) begin
							r_dicLdAMtens <= 1'b1;
							nextState <= a_Mten;
						end
						// otherwise maintain state
						else begin
							nextState <= a_load;
						end
					end
					a_Mten:
					begin
						// Reset signals
						r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;
						
						// If pressed 0-9, load mone, next
						if (de_num) begin
							r_dicLdAMones <= 1'b1;
							nextState <= a_mone;	
						end
						// Otherwise stay
						else begin
							nextState <= a_Mten;
						end
					end
					a_mone:
					begin
					   // Reset signals
					   r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;
						
						// If pressed 0-5, load sten next
						if (de_num0to5) begin
							r_dicLdAStens <= 1'b1;
							nextState <= a_Sten;	
						end
						// otherwise stay
						else begin
							nextState <= a_mone;
						end
					end
					a_Sten:
					begin
						// Reset signals
						r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;
						
						// If pressed 0-9, load sone next
						if (de_num) begin
							r_dicLdASones <= 1'b1;
							nextState <= a_sone;	
						end
						// Otherwise stay
						else begin
							nextState <= a_Sten;
						end
					end
					a_sone:
					begin
						// Reset signals
						r_dicLdAMtens <= 1'b0;
						r_dicLdAMones <= 1'b0;
						r_dicLdAStens <= 1'b0;
						r_dicLdASones <= 1'b0;

						// If pressed CR, start counting, next
						if (de_cr) begin
							alarm_set <= 1'b1;
							nextState <= count_start;
						end
						// Otherwise stay
						else begin
							nextState <= a_sone;
						end
					end
					count_start:
					begin
						// Reset signals
						r_dicLdMtens <= 1'b0;
						r_dicLdMones <= 1'b0;
						r_dicLdStens <= 1'b0;
						r_dicLdSones <= 1'b0;
						
						// If pressed 'l', load time
						if (de_littleL)
							nextState <= l_load;
						// if pressed 'a' load alarm
						else if (de_littleA)
							nextState <= a_load;
						// otherwise maintain state
						else
							nextState <= count_start;
					end
				endcase
			end
			
			// Check for keyboard inputs (alarm)
			always @(*) begin
				case (currState_al)
					// If off, transition to on
					a_off:
					begin
						if (did_alarmMatch && (currState == count_start)) begin
							nextState_al <= a_trig;
						end
							else if (de_atSign && is_alarm_set) begin
								nextState_al <= a_armed;
							end
							else begin
								nextState_al <= a_off;
							end

 					end
					// If on, transistion to off
					a_armed:
					begin
						// If alarm matching, AND set, then change register
						if (did_alarmMatch) begin
							nextState_al <= a_trig;
						end
							// If '@' pressed, transistion to off
							else if (de_atSign) begin
								nextState_al <= a_off;
							end
							else begin
								nextState_al <= a_armed;
							end
						
					end
					// if triggered, transition to off
					a_trig:
					begin
						// If '@' pressed, transistion to off
						if (de_atSign) begin
							nextState_al <= a_off;
						end
						else begin
							nextState_al <= a_trig;
						end
					end
				endcase
			end
			
			// Assign outputs based on state
			always @(posedge clk) begin
				if (currState_al == a_off) begin
						r_dicAlarmIdle <= 1'b1; // alarm is off
						r_dicAlarmArmed <= 1'b0; // alarm is armed
						r_dicAlarmTrig <= 1'b0; // alarm is triggered
				end
				else if (currState_al == a_armed) begin
						r_dicAlarmIdle <= 1'b0; // alarm is off
						r_dicAlarmArmed <= 1'b1; // alarm is armed
						r_dicAlarmTrig <= 1'b0; // alarm is triggered
				end
				else if (currState_al == a_trig) begin
						r_dicAlarmIdle <= 1'b0; // alarm is off
						r_dicAlarmArmed <= 1'b0; // alarm is armed
						r_dicAlarmTrig <= 1'b1; // alarm is triggered
				end
				else begin
				end
			end
			
			// Assign all Wires to corresponding registers
			assign dicRun = r_dicRun;
			
			assign dicLdMtens = r_dicLdMtens;
			assign dicLdMones = r_dicLdMones;
			assign dicLdStens = r_dicLdStens;
			assign dicLdSones = r_dicLdSones;
			assign dicLdAMtens = r_dicLdAMtens;
			assign dicLdAMones = r_dicLdAMones;
			assign dicLdAStens = r_dicLdAStens;
			assign dicLdASones = r_dicLdASones;
			
			assign enable1 = dicAlarmTrig ? halfPulse : r_enable1;
			assign enable2 = dicAlarmTrig ? halfPulse : r_enable2;
			assign enable3 = dicAlarmTrig ? halfPulse : r_enable3; 
			assign enable4 = dicAlarmTrig ? halfPulse : r_enable4;
			
			assign Sone_init = r_Sone_init;
			assign Sten_init = r_Sten_init;
			assign Mone_init = r_Mone_init;
			assign Mten_init = r_Mten_init;
			
			assign alarm_or_time = (r_alarm_or_time | dicAlarmTrig); 
			assign dicAlarmIdle = r_dicAlarmIdle;
			assign dicAlarmArmed = r_dicAlarmArmed;
			
			// Only when counting can alarm be triggered
			assign dicAlarmTrig = r_dicAlarmTrig && (currState == count_start); 

			assign is_alarm_set = alarm_set;
			assign n_rdy = de_littleN;
			
			endmodule

