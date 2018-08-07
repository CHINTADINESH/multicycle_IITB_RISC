module multicycle(m);
	input m;
	// clk
	wire clk;
	//memory buses
	wire [15:0] memaddressbus,memdatabusout,memdatabusin;
	// mdr register
	wire [15:0] mdroutbus;
	//load multiple store multiple unit
	wire zeroflag;	//when there is no register to be loaded or stored
	wire lmsmwrite, lmselect;	//control signals for lmsmunit
	wire [2:0] regbits;
	//store multiple temporary address units
	wire [2:0] smaddressoutreg;
	//load multiple temporary address units
	wire [2:0] lmaddressoutreg;
	//instruction register
	wire [15:0] instructiondatabusout;//instruction bus
	//control unit
	wire pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol; //onebit control
	wire [1:0] MemAddress,	Regdst,	ALUSrcA,	ALUSrcB; // tow bits control 
	wire [2:0] RegDat; //three bit control 	
	//extendzeros unit
	wire [15:0] extendzerobus;
	reg [2:0] seven;
	// register file
	wire [2:0] readReg1, readReg2, writeReg;
	wire [15:0 ] readData1, readData2, writeData;
	//temporary pc register
	wire [15:0] pcdashoutbus;
	// ra multiplexer
	wire [15:0] ramuxout;
	// ra register
	wire [15:0] raoutbus;
	// rb register
	wire [15:0] rboutbus;
	// sign extend unit one
	wire [15:0] sebus1;
	// sign extend unit two
	wire [15:0] sebus2;
	// left shift multiplexer
	wire [15:0] lsmuxout;
	//left shifter output
	wire [15:0] leftshiftoutbus;
	// alu unit
	wire [15:0] i_add_term1, i_add_term2, aluoutput;
	reg [15:0] two;
	wire zb, cb, eqflag;
	// alu out register
	// zero register
	wire z;
	// carry register
	wire c;
	// aluout register
	wire [15:0] AOoutbus;
	//no action input in case of RegDat
	reg [15:0] zeros ;
	//reset 
	reg reset;
	
	initial
		begin
			zeros <=0;
		end
	initial
		begin
			seven <=7;
		end
	initial
		begin
			two <=2;
		end

	clockgenerator cg (clk);
	//memory multiplexers for address and data
	mux4_16to16 memaddressmux (memaddressbus, raoutbus, AOoutbus, readData1, aluoutput,MemAddress);
	mux2_16to16 memdatamux (memdatabusin,rboutbus, raoutbus,memdata);
	// memory bank
	memory16bitaddressable mb(memread , memwrite, clk, reset, memdatabusin, memdatabusout, memaddressbus);
	//load multiple store multiple unit
	lmsmunit lmsm1( clk,reset,regbits,zeroflag, memdatabusout[7:0],lmsmwrite,lmselect);
	//mdr register
	register_nc mdr1 (mdroutbus, memdatabusout, clk, reset);
	//  strore multiple temporory flip flops
	lmaddress_nc smadd(smaddressoutreg, regbits, clk, reset);
	//  load multiple temporory flip flops
	lmaddress_nc lmadd(lmaddressoutreg, smaddressoutreg, clk, reset);
	//instruction register
	instruction_register ir (Irwrite, instructiondatabusout, memdatabusout, clk, reset);
	//control unit
	controlunit cu(clk, instructiondatabusout[15:12], instructiondatabusout[1:0], z, c, zeroflag, eqflag, pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol,MemAddress,	Regdst,	ALUSrcA,	ALUSrcB,RegDat,lmsmwrite, lmselect, reset);
	//extend zeros unit
	extendzeros ez( extendzerobus,instructiondatabusout[8:0] );
	// multiplexers for regA, regB, regC and datain respectively
	mux2_3to3 RegAmux(readReg1, instructiondatabusout[11:9],seven, Pcread);
	mux2_3to3 RegBmux(readReg2, instructiondatabusout[8:6], smaddressoutreg, Rbadd);
	mux4_3to3 RegCmux(writeReg,seven, instructiondatabusout[5:3], instructiondatabusout[11:9], lmaddressoutreg,Regdst);
	mux8_16to16_2 writedatamux(RegDat, mdroutbus, extendzerobus, aluoutput, AOoutbus, rboutbus, zeros, zeros, zeros, writeData);
	//register file
	regfile8_16 rf(readReg1,readReg2,writeReg,readData1,readData2,writeData,Regwrite, clk, reset);
	//temporary pc register
	pcdash pcd(pcdashwrite, pcdashoutbus, readData1, clk, reset);
	// ra multiplexer
	mux2_16to16 rasrcmux(ramuxout,aluoutput,readData1, raSrc);
	// ra register
	register_nc ra (raoutbus, ramuxout, clk, reset);
	//rb register
	register_nc rb (rboutbus, readData2, clk, reset);
	// sign extend unit one
	signextend15_6 signextendunit1(sebus1,instructiondatabusout[5:0]);
	// sign extend unit two
	signextend15_9 signextendunit2(sebus2, instructiondatabusout[8:0]);
	// left shift multiplexer
	mux2_16to16 leftshiftmux1(lsmuxout,sebus1, sebus2,LsIm);
	// left shifter
	leftShift ls ( leftshiftoutbus,lsmuxout);
	// alu source A multiplexer
	mux4_16to16 alumux1(i_add_term1, pcdashoutbus, readData1, raoutbus, sebus1,ALUSrcA);
	// alu source B multiplexer
	mux4_16to16 alumux2(i_add_term2, two, leftshiftoutbus, rboutbus, sebus1,ALUSrcB);
	// alu unit
	aluiitb alu1(i_add_term1,i_add_term2,alucontrol,aluoutput,zb,cb,eqflag);
	// zero register bit
	reg1bit zeroregbit(setz, z, zb, clk, reset);
	// carry register
	reg1bit carryregbit(setc, c, cb, clk, reset);
	// aluoutput register
	register_nc instructionregister (AOoutbus, aluoutput, clk, reset);
	
	initial 
		begin
	        reset <=1;
		#40
		    reset <=0;
		end
	always@(posedge clk) 
		begin
		#40;
		end
	
	endmodule
	
module clockgenerator(clk);
	output clk;
	reg clk;
	initial
		begin
		clk = 0;
		end
	always begin
		#20 clk = !clk;
		end
	endmodule
	
module mux8to1(select,inp,oup);
	input [2:0] select;
	input [7:0] inp;
	output oup;
	wire [7:0] decoded;
	dec3to8 dec1 (.select(select), .decoded(decoded));
	and  a1 (o1, inp[0], decoded[0]);
	and  a2 (o2, inp[1], decoded[1]);
	and  a3 (o3, inp[2], decoded[2]);
	and  a4 (o4, inp[3], decoded[3]);
	and  a5 (o5, inp[4], decoded[4]);
	and  a6 (o6, inp[5], decoded[5]);
	and  a7 (o7, inp[6], decoded[6]);
	and  a8 (o8, inp[7], decoded[7]);
	or  or1 (oup1, o1, o2, o3, o4);
	or  or2 (oup2, o5, o6, o7, o8);
	or  or3 (oup, oup1, oup2);
	endmodule
module mux4to1(select,inp,oup);
	input [1:0] select;
	input [3:0] inp;
	output oup;
	wire [3:0] decoded;
	dec2to4 dec1 (.select(select), .decoded(decoded));
	and  a1 (o1, inp[0], decoded[0]);
	and  a2 (o2, inp[1], decoded[1]);
	and  a3 (o3, inp[2], decoded[2]);
	and  a4 (o4, inp[3], decoded[3]);
	or  or1 (oup, o1, o2, o3, o4);
	endmodule
module mux2to1(select,inp,oup);
	input  select;
	input [1:0] inp;
	output oup;
	wire [1:0] decoded;
	dec1to2 dec1 (.select(select), .decoded(decoded));
	and  a1 (o1, inp[0], decoded[0]);
	and  a2 (o2, inp[1], decoded[1]);
	or  or1 (oup, o1, o2 );
	endmodule
	
module mux2_16to16(oup,inp1, inp2,select);
	input [15:0] inp1;
	input[15:0] inp2;
	input select;
	output [15:0] oup;
	mux2to1 m16  (select,{inp1[15],inp2[15]},oup[15]);
	mux2to1 m15  (select,{inp1[14],inp2[14]},oup[14]);
	mux2to1 m14  (select,{inp1[13],inp2[13]},oup[13]);
	mux2to1 m13  (select,{inp1[12],inp2[12]},oup[12]);
	mux2to1 m12  (select,{inp1[11],inp2[11]},oup[11]);
	mux2to1 m11  (select,{inp1[10],inp2[10]},oup[10]);
	mux2to1 m10  (select,{inp1[9],inp2[9]},oup[9]);
	mux2to1 m9  (select,{inp1[8],inp2[8]},oup[8]);
	mux2to1 m8  (select,{inp1[7],inp2[7]},oup[7]);
	mux2to1 m7  (select,{inp1[6],inp2[6]},oup[6]);
	mux2to1 m6  (select,{inp1[5],inp2[5]},oup[5]);
	mux2to1 m5  (select,{inp1[4],inp2[4]},oup[4]);
	mux2to1 m4  (select,{inp1[3],inp2[3]},oup[3]);
	mux2to1 m3  (select,{inp1[2],inp2[2]},oup[2]);
	mux2to1 m2  (select,{inp1[1],inp2[1]},oup[1]);
	mux2to1 m1  (select,{inp1[0],inp2[0]},oup[0]);
	endmodule
	
module mux4_16to16(oup, inp1, inp2, inp3, inp4,select);
	input [15:0] inp1;
	input[15:0] inp2;
	input [15:0] inp3;
	input [15:0] inp4;
	input [1:0] select;
	output [15:0] oup;
	mux4to1 m16  (select,{inp1[15],inp2[15],inp3[15],inp4[15]},oup[15]);
	mux4to1 m15  (select,{inp1[14],inp2[14],inp3[14],inp4[14]},oup[14]);
	mux4to1 m14  (select,{inp1[13],inp2[13],inp3[13],inp4[13]},oup[13]);
	mux4to1 m13  (select,{inp1[12],inp2[12],inp3[12],inp4[12]},oup[12]);
	mux4to1 m12  (select,{inp1[11],inp2[11],inp3[11],inp4[11]},oup[11]);
	mux4to1 m11  (select,{inp1[10],inp2[10],inp3[10],inp4[10]},oup[10]);
	mux4to1 m10  (select,{inp1[9],inp2[9],inp3[9],inp4[9]},oup[9]);
	mux4to1 m9  (select,{inp1[8],inp2[8],inp3[8],inp4[8]},oup[8]);
	mux4to1 m8  (select,{inp1[7],inp2[7],inp3[7],inp4[7]},oup[7]);
	mux4to1 m7  (select,{inp1[6],inp2[6],inp3[6],inp4[6]},oup[6]);
	mux4to1 m6  (select,{inp1[5],inp2[5],inp3[5],inp4[5]},oup[5]);
	mux4to1 m5  (select,{inp1[4],inp2[4],inp3[4],inp4[4]},oup[4]);
	mux4to1 m4  (select,{inp1[3],inp2[3],inp3[3],inp4[3]},oup[3]);
	mux4to1 m3  (select,{inp1[2],inp2[2],inp3[2],inp4[2]},oup[2]);
	mux4to1 m2  (select,{inp1[1],inp2[1],inp3[1],inp4[1]},oup[1]);
	mux4to1 m1  (select,{inp1[0],inp2[0],inp3[0],inp4[0]},oup[0]);
	endmodule
	
module mux2_3to3(oup, inp1, inp2, select);
	input select;
	input [2:0] inp1;
	input [2:0] inp2;
	output [2:0] oup ;
	
	mux2to1 m3  (select,{inp1[2],inp2[2]},oup[2]);
	mux2to1 m2  (select,{inp1[1],inp2[1]},oup[1]);
	mux2to1 m1  (select,{inp1[0],inp2[0]},oup[0]);
	endmodule

module mux4_3to3(oup,inp1, inp2, inp3, inp4,select);
	input [2:0] inp1, inp2, inp3, inp4;
	output [2:0] oup;
	input [1:0] select;
	mux4to1 m3  (select,{inp1[2],inp2[2],inp3[2],inp4[2]},oup[2]);
	mux4to1 m2  (select,{inp1[1],inp2[1],inp3[1],inp4[1]},oup[1]);
	mux4to1 m1  (select,{inp1[0],inp2[0],inp3[0],inp4[0]},oup[0]);
	endmodule
	
module mux8_16to16_2 (select, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, readreg);
	input [2:0] select;
	input [15:0] reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8;
	output [15:0] readreg;
	mux8to1 m1 (select,{ reg1[0], reg2[0], reg3[0], reg4[0], reg5[0], reg6[0], reg7[0], reg8[0]},readreg[0]);
	mux8to1 m2 (select,{ reg1[1], reg2[1], reg3[1], reg4[1], reg5[1], reg6[1], reg7[1], reg8[1]},readreg[1]);
	mux8to1 m3 (select,{ reg1[2], reg2[2], reg3[2], reg4[2], reg5[2], reg6[2], reg7[2], reg8[2]},readreg[2]);
	mux8to1 m4 (select,{ reg1[3], reg2[3], reg3[3], reg4[3], reg5[3], reg6[3], reg7[3], reg8[3]},readreg[3]);
	mux8to1 m5 (select,{ reg1[4], reg2[4], reg3[4], reg4[4], reg5[4], reg6[4], reg7[4], reg8[4]},readreg[4]);
	mux8to1 m6 (select,{ reg1[5], reg2[5], reg3[5], reg4[5], reg5[5], reg6[5], reg7[5], reg8[5]},readreg[5]);
	mux8to1 m7 (select,{ reg1[6], reg2[6], reg3[6], reg4[6], reg5[6], reg6[6], reg7[6], reg8[6]},readreg[6]);
	mux8to1 m8 (select,{ reg1[7], reg2[7], reg3[7], reg4[7], reg5[7], reg6[7], reg7[7], reg8[7]},readreg[7]);
	mux8to1 m9 (select,{ reg1[8], reg2[8], reg3[8], reg4[8], reg5[8], reg6[8], reg7[8], reg8[8]},readreg[8]);
	mux8to1 m10 (select,{ reg1[9], reg2[9], reg3[9], reg4[9], reg5[9], reg6[9], reg7[9], reg8[9]},readreg[9]);
	mux8to1 m11 (select,{ reg1[10], reg2[10], reg3[10], reg4[10], reg5[10], reg6[10], reg7[10], reg8[10]},readreg[10]);
	mux8to1 m12 (select,{ reg1[11], reg2[11], reg3[11], reg4[11], reg5[11], reg6[11], reg7[11], reg8[11]},readreg[11]);
	mux8to1 m13 (select,{ reg1[12], reg2[12], reg3[12], reg4[12], reg5[12], reg6[12], reg7[12], reg8[12]},readreg[12]);
	mux8to1 m14 (select,{ reg1[13], reg2[13], reg3[13], reg4[13], reg5[13], reg6[13], reg7[13], reg8[13]},readreg[13]);
	mux8to1 m15 (select,{ reg1[14], reg2[14], reg3[14], reg4[14], reg5[14], reg6[14], reg7[14], reg8[14]},readreg[14]);
	mux8to1 m16 (select,{ reg1[15], reg2[15], reg3[15], reg4[15], reg5[15], reg6[15], reg7[15], reg8[15]},readreg[15]);	
	endmodule

module memory16bitaddressable(readEn , writeEn, clk, reset, inData, outData, address);
	input readEn , writeEn, clk, reset;
	input [15:0] inData;
	output [15:0] outData;
	input [15:0] address;
	reg [7:0] dataram [0:511];
	reg [15:0] outData;
	integer i;
	initial begin
		for (i = 0; i < 512; i = i + 1) begin
			dataram[i] <= 0;
		end
			dataram[0] <= 8'b01000000;	//lw r0 r1 32
			dataram[1] <= 8'b01100000;	
			dataram[2] <= 8'b00110100; //lhi r2 1
			dataram[3] <= 1;				
			dataram[4] <= 8'b01010000;	//sw r0 r1 34
			dataram[5] <= 8'b01100010;	
			dataram[6] <= 8'b00000000; //add r0 r0 r1
			dataram[7] <= 8'b00010000;	
			dataram[8] <= 8'b00001001;//add r4 r5 r3
			dataram[9] <= 8'b01011000;
			dataram[10] <= 8'b00001000;//adz r4 r0 r3
			dataram[11] <= 8'b00011001;
			dataram[12] <= 8'b00001000;//adz r4 r0 r5
			dataram[13] <= 8'b00101001;
			dataram[14] <= 8'b00010000;//adi r0 r0 6
			dataram[15] <= 8'b00000110;
			dataram[16] <= 8'b01000000;//lw r0 r1 
			dataram[17] <= 8'b01100100;
			dataram[18] <= 8'b00010000;//adi r0 r0 6
			dataram[19] <= 8'b00000110;
			dataram[20] <= 8'b00100000;//ndu r0 r0r2
			dataram[21] <= 8'b01010000;
			dataram[22] <= 8'b01000010;//lw r1 r0 32
			dataram[23] <= 8'b00100000;
			dataram[24] <= 8'b01000100;//lw r2 r0 32
			dataram[25] <= 8'b00100000;
			dataram[26] <= 8'b11000010;//beq r1 r2 20
			dataram[27] <= 8'b10010100;
			//data memory
			dataram[33] <= 10;
			dataram[36] <=8'b11111111;
			dataram[37] <=8'b11111111;
			//jump
			dataram[66] <=8'b10001010;//jal r5 r0 6
			dataram[67] <=8'b00001010;
			dataram[68] <=8'b01100000;//lm r0 00000000
			dataram[69] <=8'b00000000;
			dataram[70] <=8'b01100000;//lm r0 01111111
			dataram[71] <=8'b01111111;
			dataram[72] <=8'b01110000;//sm r0 00000000
			dataram[73] <=8'b00000000;
			dataram[74] <=8'b01110000;//lm r0 01001010
			dataram[75] <=8'b10001010;

			
			//another jump
			dataram[86] <=8'b10011101;//jlr r6 r5
			dataram[87] <=8'b01000000;
			
			
	end
//	always @(negedge reset)
//		begin
//			dataram[0] <= 16'b0100000001011111;
//			dataram[65] <= 10;
//		end
//	
	always @(posedge clk ) begin
		if(writeEn==1'b1)
			begin
			dataram[address] <= inData[15:8];
			dataram[address + 1] <= inData[7:0];
			end
		end
	
	always @(address or readEn) begin
		if (readEn==1'b1)
		    begin
		    outData[15:8] <= dataram[address];
			 outData[7:0]  <= dataram[address +1];
		    end
		else
		    begin
		    outData <= 16'b0;
		    end
		end
//	assign outData =out;
	endmodule
	
	
module lmsmunit( clk,reset,regbits,zeroflag, initialbits,lmsmwrite,lmselect);
	input [7:0] initialbits;
	input lmsmwrite;
	input lmselect;
	input clk;
	input reset;
	output [2:0] regbits;
	output zeroflag;
	wire [7:0] topewire;
	wire [7:0] inwire;
	wire [7:0] changebits;
	wire [7:0] newbits;
	//wire [7:0] notchangebits;
	reg [7:0] notchangebits;
	
	priorityencoder8_3 pe (regbits, topewire);
	
	or or1(o1, topewire[7],topewire[6],topewire[5],topewire[4]);
	or or2(o2, topewire[3],topewire[2],topewire[1],topewire[0]);
	or or3(zeroflag, o1, o2);
	
	dec3to8 dc (changebits, regbits);
	always @(changebits)
		begin
		    notchangebits <= ~changebits;
		end
	mux2_8to8 mx( inwire, initialbits,newbits,lmselect);
	
	and n7 (newbits[7], notchangebits[0], topewire[7]);
	and n6 (newbits[6], notchangebits[1], topewire[6]);
	and n5 (newbits[5], notchangebits[2], topewire[5]);
	and n4 (newbits[4], notchangebits[3], topewire[4]);
	and n3 (newbits[3], notchangebits[4], topewire[3]);
	and n2 (newbits[2], notchangebits[5], topewire[2]);
	and n1 (newbits[1], notchangebits[6], topewire[1]);
	and n0 (newbits[0], notchangebits[7], topewire[0]);
		
//	reg1bit r8(.writeEn(lmsmwrite), .outbit(topewire[7]), .inbit(inwire[7]), .clk(clk), .reset(reset));
//	reg1bit r7(.writeEn(lmsmwrite), .outbit(topewire[6]), .inbit(inwire[6]), .clk(clk), .reset(reset));
//	reg1bit r6(.writeEn(lmsmwrite), .outbit(topewire[5]), .inbit(inwire[5]), .clk(clk), .reset(reset));
//	reg1bit r5(.writeEn(lmsmwrite), .outbit(topewire[4]), .inbit(inwire[4]), .clk(clk), .reset(reset));
//	reg1bit r4(.writeEn(lmsmwrite), .outbit(topewire[3]), .inbit(inwire[3]), .clk(clk), .reset(reset));
//	reg1bit r3(.writeEn(lmsmwrite), .outbit(topewire[2]), .inbit(inwire[2]), .clk(clk), .reset(reset));
//	reg1bit r2(.writeEn(lmsmwrite), .outbit(topewire[1]), .inbit(inwire[1]), .clk(clk), .reset(reset));
//	reg1bit r1(.writeEn(lmsmwrite), .outbit(topewire[0]), .inbit(inwire[0]), .clk(clk), .reset(reset));
	
	dflipflop d8 (.d(inwire[7]), .clk(clk), .reset(reset), .q(topewire[7]));
	dflipflop d7 (.d(inwire[6]), .clk(clk), .reset(reset), .q(topewire[6]));
	dflipflop d6 (.d(inwire[5]), .clk(clk), .reset(reset), .q(topewire[5]));
	dflipflop d5 (.d(inwire[4]), .clk(clk), .reset(reset), .q(topewire[4]));
	dflipflop d4 (.d(inwire[3]), .clk(clk), .reset(reset), .q(topewire[3]));
	dflipflop d3 (.d(inwire[2]), .clk(clk), .reset(reset), .q(topewire[2]));
	dflipflop d2 (.d(inwire[1]), .clk(clk), .reset(reset), .q(topewire[1]));
	dflipflop d1 (.d(inwire[0]), .clk(clk), .reset(reset), .q(topewire[0]));	
	endmodule
	

module mux2_8to8(inwire, newbits, initialbits, sel);
	input [7:0] newbits, initialbits;
	input sel;
	output [7:0] inwire;
	
	mux2to1 m7(sel, {newbits[7], initialbits[7]}, inwire[7]);
	mux2to1 m6(sel, {newbits[6], initialbits[6]}, inwire[6]);
	mux2to1 m5(sel, {newbits[5], initialbits[5]}, inwire[5]);
	mux2to1 m4(sel, {newbits[4], initialbits[4]}, inwire[4]);
	mux2to1 m3(sel, {newbits[3], initialbits[3]}, inwire[3]);
	mux2to1 m2(sel, {newbits[2], initialbits[2]}, inwire[2]);
	mux2to1 m1(sel, {newbits[1], initialbits[1]}, inwire[1]);
	mux2to1 m0(sel, {newbits[0], initialbits[0]}, inwire[0]);

	endmodule
	
	
module register_nc(out, in, clk, reset);
	input clk,reset;
	input [15:0] in;
	output [15:0] out;
	dflipflop d1 (.d(in[0]), .clk(clk), .reset(reset), .q(out[0]));
	dflipflop d2 (.d(in[1]), .clk(clk), .reset(reset), .q(out[1]));
	dflipflop d3 (.d(in[2]), .clk(clk), .reset(reset), .q(out[2]));
	dflipflop d4 (.d(in[3]), .clk(clk), .reset(reset), .q(out[3]));
	dflipflop d5 (.d(in[4]), .clk(clk), .reset(reset), .q(out[4]));
	dflipflop d6 (.d(in[5]), .clk(clk), .reset(reset), .q(out[5]));
	dflipflop d7 (.d(in[6]), .clk(clk), .reset(reset), .q(out[6]));
	dflipflop d8 (.d(in[7]), .clk(clk), .reset(reset), .q(out[7]));
	dflipflop d9 (.d(in[8]), .clk(clk), .reset(reset), .q(out[8]));
	dflipflop d10 (.d(in[9]), .clk(clk), .reset(reset), .q(out[9]));
	dflipflop d11 (.d(in[10]), .clk(clk), .reset(reset), .q(out[10]));
	dflipflop d12 (.d(in[11]), .clk(clk), .reset(reset), .q(out[11]));
	dflipflop d13 (.d(in[12]), .clk(clk), .reset(reset), .q(out[12]));
	dflipflop d14 (.d(in[13]), .clk(clk), .reset(reset), .q(out[13]));
	dflipflop d15 (.d(in[14]), .clk(clk), .reset(reset), .q(out[14]));
	dflipflop d16 (.d(in[15]), .clk(clk), .reset(reset), .q(out[15]));
	endmodule
	
module lmaddress_nc(out, in, clk, reset);
	input clk,reset;
	input [2:0] in;
	output [2:0] out;
	dflipflop d1 (.d(in[0]), .clk(clk), .reset(reset), .q(out[0]));
	dflipflop d2 (.d(in[1]), .clk(clk), .reset(reset), .q(out[1]));
	dflipflop d3 (.d(in[2]), .clk(clk), .reset(reset), .q(out[2]));	
	endmodule
	
module controlunit(clk, opcode, funct, z, c, lmsmflag, eqflag, pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol,MemAddress,	Regdst,	ALUSrcA,	ALUSrcB,RegDat,lmsmwrite, lmselect, reset);
	input clk, reset;
	input lmsmwrite, lmselect;
	input [3:0] opcode;
	input [1:0] funct;
	input z, c, lmsmflag, eqflag;
	output pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol;	
	output [1:0] MemAddress,	Regdst,	ALUSrcA,	ALUSrcB;
	output [2:0] RegDat;
	wire [4:0] presentstate; 
	wire [4:0] nextstate;
	

	currentstate cs (clk, nextstate, presentstate, reset);
	statecontroller sc (nextstate, presentstate, opcode, funct, z, c, lmsmflag, eqflag);
	outputcontroller oc (presentstate,opcode,funct,pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol,MemAddress,	Regdst,	ALUSrcA,	ALUSrcB,RegDat, lmsmwrite, lmselect);
	
	endmodule
	
module statecontroller(nextstate, presentstate, opcode, funct, z, c, lmsmflag, eqflag);
	input [4:0] presentstate;
	input [3:0] opcode;
	input [1:0] funct;
	input z, c, lmsmflag, eqflag;
	output [4:0] nextstate;
	reg [4:0] nextstate;
	
	always @(presentstate, opcode, funct, z, c, lmsmflag, eqflag)
	begin
	case (presentstate)
	0:
		nextstate <= 1; 
	1:
		if(opcode == 4'b0000)
		begin
			if(funct == 2'b00)
			begin
				nextstate <= 2; 
			end
			else if (funct ==2'b01)
			begin
				if( z ==1)
				begin
				nextstate <= 2; 					
				end
				else
				begin
				nextstate <= 0; 
				end
			end
			else if (funct ==2'b10)
			begin	
				if( c ==1)
				begin
				nextstate <= 2; 
				end
				else
				begin
				nextstate <= 0; 				
				end
			end
		end
		else if (opcode ==4'b0001)
		begin
		nextstate <= 4;
		end
		else if (opcode ==4'b0010)
		begin
			if(funct == 2'b00)
			begin
				 nextstate <= 2; 
			end
			else if (funct ==2'b01)
			begin
				if( z ==1)
				begin
					 nextstate <= 2; 					
				end
				else
				begin
					 nextstate <= 0; 
				end
			end
			else if (funct ==2'b10)
			begin
				if( c ==1)
				begin
					 nextstate <= 2; 
				end
				else
				begin
					 nextstate <= 0; 				
				end
			end
		end
		else if (opcode ==4'b0011)
		begin
			 nextstate <=5;
		end
		else if (opcode ==4'b0100)
		begin
			 nextstate <=6;
		end
		else if (opcode ==4'b0101)
		begin
			 nextstate <=6;
		end
		else if (opcode ==4'b0110)
		begin
			if(lmsmflag == 0)
			begin
				 nextstate <= 0;
			end
			else 
			begin
				 nextstate <=17;
			end
		end
		else if (opcode ==4'b0111)
		begin
			if (lmsmflag ==0)
			begin
				 nextstate <= 0;
			end
			else 
			begin
				 nextstate <=14;
			end
		end
		else if (opcode ==4'b1000)
		begin
			 nextstate <=11;
		end
		else if (opcode ==4'b1001)
		begin
			 nextstate <=13;
		end
		else if (opcode ==4'b1100)
		begin
			 nextstate <=2;
		end

		
	2:
		if(opcode == 4'b1100)
		begin
			if( eqflag ==1)
			begin
				 nextstate <= 10;
			end
			else
			begin
				 nextstate <= 0;
			end
		end
		else
		begin
			 nextstate <= 3;
		end
	3:
		 nextstate <= 0; 
	4:
		 nextstate <= 3; 
	5:
		 nextstate <= 0; 
	6:
		if(opcode == 4'b0100) begin
			nextstate <= 8; 
			end
		else begin
			nextstate <= 9;
			end
	7:
		 nextstate <= 0; 
	8:
		 nextstate <= 0; 
	9:
		 nextstate <= 0; 
	10:
		 nextstate <= 0; 
	11:
		if(opcode == 4'b1000)
		begin
			 nextstate <= 12;
		end
		else if (opcode == 4'b1001)
		begin
			 nextstate <= 0; 
		end
	12:
		 nextstate <= 0; 
	13:
		 nextstate <= 11; 
	14:
		if(lmsmflag ==0)
		begin
			 nextstate <= 16; 
		end
		else
		begin
			 nextstate <= 15; 
		end
	15:
		if(lmsmflag ==0)
		begin
			 nextstate <= 16; 
		end
		else
		begin
			 nextstate <= 15; 
		end		
	16:
		 nextstate <= 0; 
	17:
		if(lmsmflag ==0)
		begin
			 nextstate <= 19; 
		end
		else
		begin
			 nextstate <= 18; 
		end
	18:
		if(lmsmflag ==0)
		begin
			 nextstate <= 19; 
		end
		else
		begin
			 nextstate <= 18; 
		end
	19:
		 nextstate <= 0; 
	
	endcase
	end
	endmodule
	

module outputcontroller(presentstate,opcode,funct,pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol,MemAddress,	Regdst,	ALUSrcA,	ALUSrcB,RegDat,lmsmwrite ,lmselect);
	input [4:0] presentstate;
	input [3:0] opcode;
	input [1:0] funct;
	output pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol, lmsmwrite, lmselect;	
	output [1:0] MemAddress,	Regdst,	ALUSrcA,	ALUSrcB;
	output [2:0] RegDat;
	reg pcdashwrite,	memdata,	memread,	memwrite,	Rbadd,	raSrc,	LsIm,	Irwrite,	Regwrite,	Pcread,	setz,	setc,	alucontrol, lmsmwrite, lmselect;	
	reg [1:0] MemAddress,	Regdst,	ALUSrcA,	ALUSrcB;
	reg [2:0] RegDat;


	always @(presentstate or opcode )
		begin
		case(presentstate)
			0: 
			begin
				 pcdashwrite <=1;
				  memdata	<=0;
				  memread	<=1;
				  memwrite	<=0;
				  Rbadd	<=0;
				  raSrc	<=1;
				  LsIm	<=0;
				  Irwrite <=1;	
				  Regwrite <=1;	
				  Pcread	<=1;
				  MemAddress <=2;	
				  Regdst	<=0;
				  ALUSrcA	<=1;
				  ALUSrcB	<=0;
				  RegDat <=2;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=0;

            end
			1: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0	;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

			end
			2: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=2	;
				  ALUSrcB	<=2	;
				  RegDat <=0	;
				  lmsmwrite	<= 1;
				  lmselect <=1;
					if (opcode == 4'b0000) begin
						alucontrol <=0;
					end
					else if(opcode ==4'b0010) begin
						alucontrol <=1;
					end
					else begin
						alucontrol <=0;
					end
			end
			3: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=1	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=3;
					lmsmwrite	<= 1;
				  lmselect <=1;
					alucontrol <=0;
			end
			4: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=2	;
				  ALUSrcB	<=3	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			5: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=2	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=1;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			6: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=1	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=3	;	
				  Regdst	<=0	;
				  ALUSrcA	<=3	;
				  ALUSrcB	<=2	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			7: 
				;
			8: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=2	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

			end
			9: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=1	;
				  memread	<=0	;
				  memwrite	<=1	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=1	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

			end
			10:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=1	;
				  RegDat <=2;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			11:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=2	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=2;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			12:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=1	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=1	;
				  RegDat <=2;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			13: 
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=4;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

		    end
			14:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=1	;
				  raSrc	<=1	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			15:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=1	;
				  Rbadd	<=1	;
				  raSrc	<=0	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=2	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			16:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=1	;
				  Rbadd	<=0	;
				  raSrc	<=0	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			17:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=1	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=0	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=0	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=0	;
				  ALUSrcA	<=2	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			18:
			begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=1	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=0	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=3	;
				  ALUSrcA	<=2	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			19:
			   begin
				  pcdashwrite <=0	;
				  memdata	<=0	;
				  memread	<=0	;
				  memwrite	<=0	;
				  Rbadd	<=0	;
				  raSrc	<=0	;
				  LsIm	<=0	;
				  Irwrite <=0	;	
				  Regwrite <=1	;	
				  Pcread	<=0	;
				  MemAddress <=0	;	
				  Regdst	<=3	;
				  ALUSrcA	<=0	;
				  ALUSrcB	<=0	;
				  RegDat <=0;
				  alucontrol <=0;
				  lmsmwrite	<= 1;
				  lmselect <=1;

            end
			endcase
			end

		always @(presentstate or opcode or funct)
		begin
		case(presentstate)
		2:
			if(opcode == 0000)
			begin
			setc <= 1;
			setz <= 1;
			end

		4:
			if(opcode == 0001)
			begin
			setc <= 1;
			setz <= 1;
			end
		6:
			if(opcode == 0100)
			begin
			setc <=0;
			setz <= 1;
			end
		default:
        begin
			setc <=0;
			setz <= 0;		
		end
		endcase
		end

	endmodule
	
module currentstate(clk, nextstate, presentstate, reset);
	input clk, reset;
	input [4:0] nextstate;
	output [4:0] presentstate;
	reg we;

	initial begin
	we =1;
	end
		
	reg1bit five(we, presentstate[4], nextstate[4], clk, reset);
	reg1bit four(we, presentstate[3], nextstate[3], clk, reset);
	reg1bit three(we, presentstate[2], nextstate[2], clk, reset);
	reg1bit tow(we, presentstate[1], nextstate[1], clk, reset);
	reg1bit one(we, presentstate[0], nextstate[0], clk, reset);
	endmodule
	
module extendzeros( out,in );
	output [15:0] out;
	input [8:0] in;
	reg [15:0] out;
	always @ (in)
		begin
		out <= {in, 7'b0};
		end
	endmodule
	
module regfile8_16(readReg1,readReg2,writeReg,readData1,readData2,writeData,writeEnable, clk, reset);
	input [15:0] writeData;
	input [2:0] readReg1, readReg2, writeReg;
	input writeEnable, clk, reset;
	output [15:0] readData1, readData2;
	wire [15:0] regwire [0:7];
	wire [0:7] decodedWriteRegister;
	wire [0:7] decodedWriteRegisterEnabled;
	dec3to8 dec (decodedWriteRegister, writeReg);
	writeEnabled wrEn (writeEnable, decodedWriteRegister, decodedWriteRegisterEnabled); 
	register reg1 (.writeEn(decodedWriteRegisterEnabled[0]), .out(regwire[0]), .in(writeData), .clk(clk), .reset(reset));
	register reg2 (.writeEn(decodedWriteRegisterEnabled[1]), .out(regwire[1]), .in(writeData), .clk(clk), .reset(reset));
	register reg3 (.writeEn(decodedWriteRegisterEnabled[2]), .out(regwire[2]), .in(writeData), .clk(clk), .reset(reset));
	register reg4 (.writeEn(decodedWriteRegisterEnabled[3]), .out(regwire[3]), .in(writeData), .clk(clk), .reset(reset));
	register reg5 (.writeEn(decodedWriteRegisterEnabled[4]), .out(regwire[4]), .in(writeData), .clk(clk), .reset(reset));
	register reg6 (.writeEn(decodedWriteRegisterEnabled[5]), .out(regwire[5]), .in(writeData), .clk(clk), .reset(reset));
	register reg7 (.writeEn(decodedWriteRegisterEnabled[6]), .out(regwire[6]), .in(writeData), .clk(clk), .reset(reset));
	register reg8 (.writeEn(decodedWriteRegisterEnabled[7]), .out(regwire[7]), .in(writeData), .clk(clk), .reset(reset));
	mux8_16to16 readmux1 (readReg1, regwire[7], regwire[6], regwire[5], regwire[4], regwire[3], regwire[2], regwire[1], regwire[0], readData1);
	mux8_16to16 readmux2 (readReg2, regwire[7], regwire[6], regwire[5], regwire[4], regwire[3], regwire[2], regwire[1], regwire[0], readData2);	
	endmodule	
module writeEnabled( writeEn, decodedRegister, writeEnabledControl);
	input writeEn;
	input [7:0] decodedRegister;
	output [7:0] writeEnabledControl;
	and  a1 (writeEnabledControl[0] , writeEn, decodedRegister[0]);
	and  a2 (writeEnabledControl[1] , writeEn, decodedRegister[1]);
	and  a3 (writeEnabledControl[2] , writeEn, decodedRegister[2]);
	and  a4 (writeEnabledControl[3] , writeEn, decodedRegister[3]);
	and  a5 (writeEnabledControl[4] , writeEn, decodedRegister[4]);
	and  a6 (writeEnabledControl[5] , writeEn, decodedRegister[5]);
	and  a7 (writeEnabledControl[6] , writeEn, decodedRegister[6]);
	and  a8 (writeEnabledControl[7] , writeEn, decodedRegister[7]);	
	endmodule
module register(writeEn, out, in, clk, reset);
	input writeEn, clk,reset;
	input [15:0] in;
	output [15:0] out;
	reg1bit reg1(.writeEn(writeEn),.outbit(out[0]), .inbit(in[0]), .clk(clk), .reset(reset));
	reg1bit reg2(.writeEn(writeEn),.outbit(out[1]), .inbit(in[1]), .clk(clk), .reset(reset));
	reg1bit reg3(.writeEn(writeEn),.outbit(out[2]), .inbit(in[2]), .clk(clk), .reset(reset));
	reg1bit reg4(.writeEn(writeEn),.outbit(out[3]), .inbit(in[3]), .clk(clk), .reset(reset));
	reg1bit reg5(.writeEn(writeEn),.outbit(out[4]), .inbit(in[4]), .clk(clk), .reset(reset));
	reg1bit reg6(.writeEn(writeEn),.outbit(out[5]), .inbit(in[5]), .clk(clk), .reset(reset));
	reg1bit reg7(.writeEn(writeEn),.outbit(out[6]), .inbit(in[6]), .clk(clk), .reset(reset));
	reg1bit reg8(.writeEn(writeEn),.outbit(out[7]), .inbit(in[7]), .clk(clk), .reset(reset));
	reg1bit reg9(.writeEn(writeEn),.outbit(out[8]), .inbit(in[8]), .clk(clk), .reset(reset));
	reg1bit reg10(.writeEn(writeEn),.outbit(out[9]), .inbit(in[9]), .clk(clk), .reset(reset));
	reg1bit reg11(.writeEn(writeEn),.outbit(out[10]), .inbit(in[10]), .clk(clk), .reset(reset));
	reg1bit reg12(.writeEn(writeEn),.outbit(out[11]), .inbit(in[11]), .clk(clk), .reset(reset));
	reg1bit reg13(.writeEn(writeEn),.outbit(out[12]), .inbit(in[12]), .clk(clk), .reset(reset));
	reg1bit reg14(.writeEn(writeEn),.outbit(out[13]), .inbit(in[13]), .clk(clk), .reset(reset));
	reg1bit reg15(.writeEn(writeEn),.outbit(out[14]), .inbit(in[14]), .clk(clk), .reset(reset));
	reg1bit reg16(.writeEn(writeEn),.outbit(out[15]), .inbit(in[15]), .clk(clk), .reset(reset));
	endmodule
module mux8_16to16 (select, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, readreg);
	input [2:0] select;
	input [15:0] reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8;
	output [15:0] readreg;
	mux8to1 m1 (select,{ reg8[0], reg7[0], reg6[0], reg5[0], reg4[0], reg3[0], reg2[0], reg1[0]},readreg[0]);
	mux8to1 m2 (select,{ reg8[1], reg7[1], reg6[1], reg5[1], reg4[1], reg3[1], reg2[1], reg1[1]},readreg[1]);
	mux8to1 m3 (select,{ reg8[2], reg7[2], reg6[2], reg5[2], reg4[2], reg3[2], reg2[2], reg1[2]},readreg[2]);
	mux8to1 m4 (select,{ reg8[3], reg7[3], reg6[3], reg5[3], reg4[3], reg3[3], reg2[3], reg1[3]},readreg[3]);
	mux8to1 m5 (select,{ reg8[4], reg7[4], reg6[4], reg5[4], reg4[4], reg3[4], reg2[4], reg1[4]},readreg[4]);
	mux8to1 m6 (select,{ reg8[5], reg7[5], reg6[5], reg5[5], reg4[5], reg3[5], reg2[5], reg1[5]},readreg[5]);
	mux8to1 m7 (select,{ reg8[6], reg7[6], reg6[6], reg5[6], reg4[6], reg3[6], reg2[6], reg1[6]},readreg[6]);
	mux8to1 m8 (select,{ reg8[7], reg7[7], reg6[7], reg5[7], reg4[7], reg3[7], reg2[7], reg1[7]},readreg[7]);
	mux8to1 m9 (select,{ reg8[8], reg7[8], reg6[8], reg5[8], reg4[8], reg3[8], reg2[8], reg1[8]},readreg[8]);
	mux8to1 m10 (select,{ reg8[9], reg7[9], reg6[9], reg5[9], reg4[9], reg3[9], reg2[9], reg1[9]},readreg[9]);
	mux8to1 m11 (select,{ reg8[10], reg7[10], reg6[10], reg5[10], reg4[10], reg3[10], reg2[10], reg1[10]},readreg[10]);
	mux8to1 m12 (select,{ reg8[11], reg7[11], reg6[11], reg5[11], reg4[11], reg3[11], reg2[11], reg1[11]},readreg[11]);
	mux8to1 m13 (select,{ reg8[12], reg7[12], reg6[12], reg5[12], reg4[12], reg3[12], reg2[12], reg1[12]},readreg[12]);
	mux8to1 m14 (select,{ reg8[13], reg7[13], reg6[13], reg5[13], reg4[13], reg3[13], reg2[13], reg1[13]},readreg[13]);
	mux8to1 m15 (select,{ reg8[14], reg7[14], reg6[14], reg5[14], reg4[14], reg3[14], reg2[14], reg1[14]},readreg[14]);
	mux8to1 m16 (select,{ reg8[15], reg7[15], reg6[15], reg5[15], reg4[15], reg3[15], reg2[15], reg1[15]},readreg[15]);	
	endmodule


module pcdash(pcwrite, out, in, clk, reset);
	input pcwrite, clk,reset;
	input [15:0] in;
	output [15:0] out;
	reg1bit reg1(.writeEn(pcwrite),.outbit(out[0]), .inbit(in[0]), .clk(clk), .reset(reset));
	reg1bit reg2(.writeEn(pcwrite),.outbit(out[1]), .inbit(in[1]), .clk(clk), .reset(reset));
	reg1bit reg3(.writeEn(pcwrite),.outbit(out[2]), .inbit(in[2]), .clk(clk), .reset(reset));
	reg1bit reg4(.writeEn(pcwrite),.outbit(out[3]), .inbit(in[3]), .clk(clk), .reset(reset));
	reg1bit reg5(.writeEn(pcwrite),.outbit(out[4]), .inbit(in[4]), .clk(clk), .reset(reset));
	reg1bit reg6(.writeEn(pcwrite),.outbit(out[5]), .inbit(in[5]), .clk(clk), .reset(reset));
	reg1bit reg7(.writeEn(pcwrite),.outbit(out[6]), .inbit(in[6]), .clk(clk), .reset(reset));
	reg1bit reg8(.writeEn(pcwrite),.outbit(out[7]), .inbit(in[7]), .clk(clk), .reset(reset));
	reg1bit reg9(.writeEn(pcwrite),.outbit(out[8]), .inbit(in[8]), .clk(clk), .reset(reset));
	reg1bit reg10(.writeEn(pcwrite),.outbit(out[9]), .inbit(in[9]), .clk(clk), .reset(reset));
	reg1bit reg11(.writeEn(pcwrite),.outbit(out[10]), .inbit(in[10]), .clk(clk), .reset(reset));
	reg1bit reg12(.writeEn(pcwrite),.outbit(out[11]), .inbit(in[11]), .clk(clk), .reset(reset));
	reg1bit reg13(.writeEn(pcwrite),.outbit(out[12]), .inbit(in[12]), .clk(clk), .reset(reset));
	reg1bit reg14(.writeEn(pcwrite),.outbit(out[13]), .inbit(in[13]), .clk(clk), .reset(reset));
	reg1bit reg15(.writeEn(pcwrite),.outbit(out[14]), .inbit(in[14]), .clk(clk), .reset(reset));
	reg1bit reg16(.writeEn(pcwrite),.outbit(out[15]), .inbit(in[15]), .clk(clk), .reset(reset));
	endmodule
	
module signextend15_6(out,in);
	output [15:0] out;
	input [5:0] in;
	reg [15:0] out;
	always @(in)
		begin
		out = {10'b0,in};
		end
	endmodule
	
module signextend15_9(out, in);
	output [15:0] out;
	input [8:0] in;
	reg [15:0] out;
	always @(in)
		begin
		out = {7'b0,in};
		end
	endmodule
	
	
module aluiitb(i_add_term1,i_add_term2,sel,o_1,z,c,eqflag);
    input [15:0]  i_add_term1;
    input [15:0]  i_add_term2;
	input  sel;
    output [15:0] o_1;
	output z;
	output c;
	output eqflag;
	reg [15:0] outreg;
	wire [15:0] o_result;
	wire [15:0] nandout;
	reg cin;
	reg eqflag;
//	reg c;
	reg z;
	reg [15:0] xo;
	always @(i_add_term1 or i_add_term2)
		begin
		    cin <=0;
		end
	always @(i_add_term1 or i_add_term2)
		begin
		xo = i_add_term1 ^ i_add_term2;
		if(xo==0)
			begin
			    eqflag <= 1;
			end
		else
			begin 
			    eqflag <= 0;
			end
		end
	fullAdder_16 fa1 (.i_add_term1(i_add_term1), .i_add_term2(i_add_term2), .inpcarry(cin), .o_result(o_result),.c(c));
	nandout_16 no (i_add_term1, i_add_term2, nandout);
	always @(sel or o_result or nandout)
	  begin
			case (sel)
			0: outreg = o_result;
			1: outreg = nandout;
			endcase
		end	
	always @(outreg)
		begin
		if(outreg == 0)
		begin
			    z <= 1;
		end
		else
		begin
			 z<= 0;
		end
		end
	assign o_1 = outreg;
	endmodule
module nandout_16 (inp1, inp2, oup);
	input [15:0] inp1;
	input [15:0] inp2;
	output [15:0] oup;
	nand n1 (oup[0], inp1[0], inp2[0]);
	nand n2 (oup[1], inp1[1], inp2[1]);
	nand n3 (oup[2], inp1[2], inp2[2]);
	nand n4 (oup[3], inp1[3], inp2[3]);
	nand n5 (oup[4], inp1[4], inp2[4]);
	nand n6 (oup[5], inp1[5], inp2[5]);
	nand n7 (oup[6], inp1[6], inp2[6]);
	nand n8 (oup[7], inp1[7], inp2[7]);
	nand n9 (oup[8], inp1[8], inp2[8]);
	nand n10 (oup[9], inp1[9], inp2[9]);
	nand n11 (oup[10], inp1[10], inp2[10]);
	nand n12 (oup[11], inp1[11], inp2[11]);
	nand n13 (oup[12], inp1[12], inp2[12]);
	nand n14 (oup[13], inp1[13], inp2[13]);
	nand n15 (oup[14], inp1[14], inp2[14]);
	nand n16 (oup[15], inp1[15], inp2[15]);
	endmodule
module fullAdder_16
	(
   input [15:0]  i_add_term1,
   input [15:0]  i_add_term2,
    input inpcarry,
   output [15:0] o_result,
   output c
	);
	wire [16:0]    w_CARRY;
	wire [15:0]    w_SUM;
	assign w_CARRY[0] = inpcarry;
	fullAdder full_adder_1
    ( 
      .a(i_add_term1[0]),
      .b(i_add_term2[0]),
      .ci(w_CARRY[0]),
      .s(w_SUM[0]),
      .co(w_CARRY[1])
      );
 
	fullAdder full_adder_2
    ( 
      .a(i_add_term1[1]),
      .b(i_add_term2[1]),
      .ci(w_CARRY[1]),
      .s(w_SUM[1]),
      .co(w_CARRY[2])
      );
	fullAdder full_adder_3
    ( 
      .a(i_add_term1[2]),
      .b(i_add_term2[2]),
      .ci(w_CARRY[2]),
      .s(w_SUM[2]),
      .co(w_CARRY[3])
      );
	fullAdder full_adder_4
    ( 
      .a(i_add_term1[3]),
      .b(i_add_term2[3]),
      .ci(w_CARRY[3]),
      .s(w_SUM[3]),
      .co(w_CARRY[4])
      );
	fullAdder full_adder_5
    ( 
      .a(i_add_term1[4]),
      .b(i_add_term2[4]),
      .ci(w_CARRY[4]),
      .s(w_SUM[4]),
      .co(w_CARRY[5])
      );
	fullAdder full_adder_6
    ( 
      .a(i_add_term1[5]),
      .b(i_add_term2[5]),
      .ci(w_CARRY[5]),
      .s(w_SUM[5]),
      .co(w_CARRY[6])
      );
	fullAdder full_adder_7
    ( 
      .a(i_add_term1[6]),
      .b(i_add_term2[6]),
      .ci(w_CARRY[6]),
      .s(w_SUM[6]),
      .co(w_CARRY[7])
      );
	fullAdder full_adder_8
    ( 
      .a(i_add_term1[7]),
      .b(i_add_term2[7]),
      .ci(w_CARRY[7]),
      .s(w_SUM[7]),
      .co(w_CARRY[8])
      );
	fullAdder full_adder_9
    ( 
      .a(i_add_term1[8]),
      .b(i_add_term2[8]),
      .ci(w_CARRY[8]),
      .s(w_SUM[8]),
      .co(w_CARRY[9])
      );
	fullAdder full_adder_10
    ( 
      .a(i_add_term1[9]),
      .b(i_add_term2[9]),
      .ci(w_CARRY[9]),
      .s(w_SUM[9]),
      .co(w_CARRY[10])
      );
	fullAdder full_adder_11
    ( 
      .a(i_add_term1[10]),
      .b(i_add_term2[10]),
      .ci(w_CARRY[10]),
      .s(w_SUM[10]),
      .co(w_CARRY[11])
      );
	fullAdder full_adder_12
    ( 
      .a(i_add_term1[11]),
      .b(i_add_term2[11]),
      .ci(w_CARRY[11]),
      .s(w_SUM[11]),
      .co(w_CARRY[12])
      );
	fullAdder full_adder_13
    ( 
      .a(i_add_term1[12]),
      .b(i_add_term2[12]),
      .ci(w_CARRY[12]),
      .s(w_SUM[12]),
      .co(w_CARRY[13])
      );
	fullAdder full_adder_14
    ( 
      .a(i_add_term1[13]),
      .b(i_add_term2[13]),
      .ci(w_CARRY[13]),
      .s(w_SUM[13]),
      .co(w_CARRY[14])
      );
	fullAdder full_adder_15
    ( 
      .a(i_add_term1[14]),
      .b(i_add_term2[14]),
      .ci(w_CARRY[14]),
      .s(w_SUM[14]),
      .co(w_CARRY[15])
      );
fullAdder full_adder_16
    ( 
      .a(i_add_term1[15]),
      .b(i_add_term2[15]),
      .ci(w_CARRY[15]),
      .s(w_SUM[15]),
      .co(w_CARRY[16])
      );
	assign o_result = w_SUM;   // Verilog Concatenation
	assign c = w_CARRY[16];
	endmodule // ripple_carry_adder_2_FA
module fullAdder(s,co,a,b,ci);
	output s,co;
	input a,b,ci;
	xor u1(s,a,b,ci);
	and u2(n1,a,b);
	and u3(n2,b,ci);
	and u4(n3,a,ci);
	or u5(co,n1,n2,n3);
	endmodule	
	
module reg1bit(writeEn, outbit, inbit, clk, reset);
	input writeEn, inbit, clk, reset;
	output outbit;
	wire in1, in2;
	and  and1(in1, writeEn, inbit);
	and and2(in2, ~writeEn, outbit);
	or  or1(d, in1, in2);
	dflipflop dff1(.d(d),.clk(clk),.reset(reset),.q(outbit));
	endmodule
	
module priorityencoder8_3 (binary_out ,  encoder_in );
	output [2:0] binary_out ;
	input [7:0] encoder_in ; 
	reg [2:0] binary_out ;     
	always @ ( encoder_in)
	begin
		if (encoder_in[7] == 1) begin
			binary_out <= 7; 
		end else if (encoder_in[6] == 1) begin
			binary_out <= 6; 
		end else if (encoder_in[5] == 1) begin
			binary_out <= 5; 
		end else if (encoder_in[4] == 1) begin
			binary_out <= 4; 
		end else if (encoder_in[3] == 1) begin
			binary_out <= 3; 
		end else if (encoder_in[2] == 1) begin
			binary_out <= 2; 
		end else if (encoder_in[1] == 1) begin
			binary_out <= 1; 
		end else if (encoder_in[0] == 1) begin
			binary_out <= 0; 
		end
		else begin
			binary_out <=0;
		end
		
	end
	endmodule  
module dec3to8(decoded,select);
	input [2:0] select;
	output [7:0] decoded;
	wire [2:0] opp; 
	not  nota(opp[0],select[0]);
	not  notb(opp[1],select[1]);
	not  notc(opp[2],select[2]);
	and am1(decoded[7],opp[2],opp[1],opp[0]);
	and am2(decoded[6],opp[2],opp[1],select[0]);
	and am3(decoded[5],opp[2],select[1],opp[0]);
	and am4(decoded[4],opp[2],select[1],select[0]);
	and am5(decoded[3],select[2],opp[1],opp[0]);
	and am6(decoded[2],select[2],opp[1],select[0]);
	and am7(decoded[1],select[2],select[1],opp[0]);
	and am8(decoded[0],select[2],select[1],select[0]);
	endmodule

module dec1to2(select,decoded);
	input  select;
	output [1:0] decoded;
	wire  opp; 
	not  nota(opp,select);
	and am1(decoded[1],opp);
	and am2(decoded[0],select);
	endmodule
	
module dec2to4(select,decoded);
	input [1:0] select;
	output [3:0] decoded;
	wire [1:0] opp; 
	not  nota(opp[0],select[0]);
	not  notb(opp[1],select[1]);
	and am1(decoded[3],opp[1],opp[0]);
	and am2(decoded[2],opp[1],select[0]);
	and am3(decoded[1],select[1],opp[0]);
	and am4(decoded[0],select[1],select[0]);
	endmodule
	
module dflipflop(d, clk, reset, q );
	input clk, reset,d ;
	output q;
	reg q;
	always @(posedge clk or posedge reset)begin
		if(reset ==1'b1)
			q <= 0;
		else
			q = d;
	end
	endmodule

module instruction_register(IRwrite, out, in, clk, reset);
	input IRwrite, clk,reset;
	input [15:0] in;
	output [15:0] out;
	reg1bit reg1(.writeEn(IRwrite),.outbit(out[0]), .inbit(in[0]), .clk(clk), .reset(reset));
	reg1bit reg2(.writeEn(IRwrite),.outbit(out[1]), .inbit(in[1]), .clk(clk), .reset(reset));
	reg1bit reg3(.writeEn(IRwrite),.outbit(out[2]), .inbit(in[2]), .clk(clk), .reset(reset));
	reg1bit reg4(.writeEn(IRwrite),.outbit(out[3]), .inbit(in[3]), .clk(clk), .reset(reset));
	reg1bit reg5(.writeEn(IRwrite),.outbit(out[4]), .inbit(in[4]), .clk(clk), .reset(reset));
	reg1bit reg6(.writeEn(IRwrite),.outbit(out[5]), .inbit(in[5]), .clk(clk), .reset(reset));
	reg1bit reg7(.writeEn(IRwrite),.outbit(out[6]), .inbit(in[6]), .clk(clk), .reset(reset));
	reg1bit reg8(.writeEn(IRwrite),.outbit(out[7]), .inbit(in[7]), .clk(clk), .reset(reset));
	reg1bit reg9(.writeEn(IRwrite),.outbit(out[8]), .inbit(in[8]), .clk(clk), .reset(reset));
	reg1bit reg10(.writeEn(IRwrite),.outbit(out[9]), .inbit(in[9]), .clk(clk), .reset(reset));
	reg1bit reg11(.writeEn(IRwrite),.outbit(out[10]), .inbit(in[10]), .clk(clk), .reset(reset));
	reg1bit reg12(.writeEn(IRwrite),.outbit(out[11]), .inbit(in[11]), .clk(clk), .reset(reset));
	reg1bit reg13(.writeEn(IRwrite),.outbit(out[12]), .inbit(in[12]), .clk(clk), .reset(reset));
	reg1bit reg14(.writeEn(IRwrite),.outbit(out[13]), .inbit(in[13]), .clk(clk), .reset(reset));
	reg1bit reg15(.writeEn(IRwrite),.outbit(out[14]), .inbit(in[14]), .clk(clk), .reset(reset));
	reg1bit reg16(.writeEn(IRwrite),.outbit(out[15]), .inbit(in[15]), .clk(clk), .reset(reset));
	endmodule
	
module leftShift ( SO,DI);  
	input  [15:0] DI;  
	output [15:0] SO;  
	reg[15:0] SO;  
	always @(DI )  
	begin  
      SO <= DI << 1;  
	end  
	endmodule
